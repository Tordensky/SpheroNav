# coding: utf-8
"""
Sphero API Module
"""
import bluetooth
import struct
from threading import Thread, Event
import threading
import time
import select

import request
import response
from response import AsyncMsg
from response import Response
from sphero import streaming
from error import SpheroError, SpheroConnectionError, SpheroFatalError, SpheroRequestError
from constants import MotorMode


class SpheroAPI(object):
    """
    A class that implements the Sphero API
    One instance represents on Sphero Device
    """

    response_timeout = 25.0

    def __init__(self, bt_name=None, bt_addr=None):
        self._dev = 0x00

        self._seq = 0x00
        self._seq_lock = threading.RLock()

        self.bt_name = bt_name
        self.bt_addr = bt_addr

        self._bt_socket = None
        self._connecting = False

        # FOR THE ASYNC RECEIVER
        self._response_lock = threading.RLock()

        self._receiver_crashed = False
        self._receiver_thread = None
        self._run_receive = True
        self._packages = []
        self._responses = []

        self._requests_waiting_response = {}

        # Sensor streaming config
        self._ssc = None
        self.sensors = None
        """ :type sensor_data: streaming.SensorStreamingResponse"""

        # async callbacks
        self._streaming_cb = None
        self._collision_cb = None
        self._power_state_cb = None

        # For the sphero manager
        self._is_taken = False
        self._take_lock = threading.RLock()

    @property
    def in_use(self):
        """
        Check if this sphero instance is set to be in use by

        :return: True if claimed False else
        :rtype: bool
        """
        with self._take_lock:
            return self._is_taken

    def claim(self):
        """
        Use this method to claim the device
        :return: True if successfully claimed, false else
        :rtype: bool
        """
        with self._take_lock:
            if not self._is_taken:
                self._is_taken = True
            return self._is_taken

    def release(self):
        """
        Release this device so it could be claimed by another user
        """
        with self._take_lock:
            self._is_taken = False

    @property
    def seq(self):
        """
        A thread-safe method for creating sequence numbers.
        :return: a new seq number
        """
        with self._seq_lock:
            self._seq += 1
            if self._seq > 0xFF:
                self._seq = 0x00
            return self._seq

    def connect(self, retries=10):
        """
        Connect the sphero device
        :param retries: Number of connection retries
        :return: None
        """
        if self.bt_addr is None:
            raise SpheroError("No device address is set for the connection")

        if self._connecting:
            raise SpheroError("Device is already trying to connect")

        if self.connected():
            return True
            # raise SpheroError("Device is already connected")

        self._connecting = True

        return self._connect(retries)

    def _connect(self, retries):
        """
        A Helper method for connecting the sphero. This is where the actual connection is executed
        """
        for _ in xrange(retries):
            try:
                self._bt_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                self._bt_socket.connect((self.bt_addr, 1))

                # If connection was established, start listening for incoming packages
                self._start_receiver()
                break
            except bluetooth.btcommon.BluetoothError:
                time.sleep(1.0)
        else:
            self._connecting = False
            raise SpheroConnectionError('Failed to connect after %d retries. Is the device turned on?' % retries)
        self._connecting = False
        return True

    def _start_receiver(self):
        """
        Starts the asynchronous package receiver
        """
        if not self._receiver_thread:
            self._run_receive = True
            self._receiver_thread = Thread(target=self._receiver, name="SpheroReceiverThread")
            self._receiver_thread.daemon = True
            self._receiver_thread.start()

    def _stop_receiver(self):
        """
        Stops the asynchronous package receiver
        """
        self._run_receive = False
        self._receiver_thread = None

    def disconnect(self):
        """
        Closes the sphero connection
        :return: True if the connection was closed
        """
        if self._bt_socket is not None:
            self._stop_receiver()
            self._bt_socket.close()
            self._bt_socket = None
            return True
        return False

    def connected(self):
        """
        Returns a bool if the sphero is connected
        :return: True if the sphero is connected
        """
        if self._bt_socket is not None and not self._connecting:
            return True
        return False

    def _get_request(self, seq):
        """
        Helper method
        Returns the request package with the given sequence number
        Raises a indexError if the seq number does not exist
        :param seq: The request sequence number
        :raise: IndexError
        :return: The request object
        """
        return filter(lambda a: a.seq == seq, self._packages)[0]

    def _get_response(self, seq):
        """
        Helper method
        Returns the response package with the given sequence number
        Raises a indexError if the seq number does not exist
        :param seq: The response sequence number
        :raise: IndexError
        :return: The request object
        """
        with self._response_lock:
            res = next((res for res in self._responses if res.seq == seq), None)
            self._responses.remove(res)
            return res

    def _clean_up_failed_request(self, packet):
        try:
            self._packages.remove(packet)
        except ValueError:
            pass
        try:
            self._requests_waiting_response.pop(packet.seq)
        except KeyError:
            pass

    def _send_package(self, packet):
        """
        Sends the given package to the connected sphero
        :param packet: The request package to send to the connected device
        """
        event = Event()
        if not self._requests_waiting_response.keys in [packet.seq]:
            self._requests_waiting_response[packet.seq] = event
            self._packages.append(packet)
        else:
            raise SpheroError("To many outgoing packages in send queue")

        try:
            self._bt_socket.send(str(packet))
        except bluetooth.BluetoothError as bt:
            # self._clean_up_failed_request(packet)
            raise SpheroConnectionError("Could not send msg, device is not connected", bt.message)
        return event

    def _write(self, packet):
        """
        Sends a message to the connected device
        :param packet: The request to send. A subclass of the Request class
        :type packet: request.Request
        :return: A response class or :raise SpheroError: if no response received
        :rtype: response.Response

        :raise SpheroConnectionError: If device is not connected
        """
        if not self.connected():
            raise SpheroConnectionError('Device is not connected')

        if self._receiver_crashed:
            raise SpheroError('FATAL Error, could not receive data from sphero. (receiver crashed)')

        event = self._send_package(packet)
        if event.wait(self.response_timeout):
            res = self._get_response(packet.seq)

            if res.success:
                return res
            else:
                raise SpheroRequestError('Request failed: ' + res.msg)
        else:
            self._clean_up_failed_request(packet)
            if self._receiver_crashed:
                self.disconnect()
                raise SpheroFatalError('FATAL Error, could not receive data from sphero. (receiver crashed)')
            raise SpheroRequestError('No response received from device before timeout')

    def _something_to_receive(self):
        """
        Helper method Checks if there is something to receive from the bt_socket
        :return Returns True if anything to receive from socket
        :rtype: bool
        """
        ready_to_receive = select.select([self._bt_socket], [], [], 0.1)[0]
        return self._bt_socket in ready_to_receive

    def _receive_header(self, fmt='5B', length=5):
        """
        Helper method, receives the header of the package from the bt socket and converts it into a tuple

        Start to read from the first 0xFF received. This is a fix to remove broken data in the incoming buffer
        :param fmt: The format of the header
        :return: tuple
        """
        # NOTE: ADDED THIS TO REMOVE SOME WRONG BYTES
        # THAT PERIODICALLY APPEARED IN THE START OF RECEIVE
        start_of_header = False
        while not start_of_header:
            raw_data = self._receive_data(1)
            first_byte = struct.unpack('B', raw_data)[0]
            start_of_header = first_byte == 0xFF
            if not start_of_header:
                print "NOTE! Removes wrong byte in start of header! Byte: ", first_byte

        raw_data += self._receive_data(length - 1)
        return struct.unpack(fmt, raw_data)

    def _handle_async_msg(self, body, header):
        """
        Helper method for parsing incoming async packages from sphero.
        Find the type of message received and triggers an appropriate event for this
        message

        :param body: The raw body of the received package
        :type body: str or raw_data
        :param header: The header of the received package
        :type header: tuple
        """
        if AsyncMsg.is_collision_notification(header):
            msg = response.CollisionDetected(header, body)
            self._on_collision(msg)

        elif AsyncMsg.is_power_state_notification(header):
            msg = response.PowerNotification(header, body)
            self._on_power_state_cb(msg)

        elif AsyncMsg.is_level_one_diagnostics(header):
            msg = response.LevelOneDiagnostics(header, body)
            print msg  # TODO implement cb

        elif AsyncMsg.is_sensor_streaming_package(header):
            if self._ssc:
                msg = streaming.SensorStreamingResponse(header, body, self._ssc)
                self._on_streaming(msg)

        else:
            # TODO implement other types of async messages
            print "Received unknown async msg! Header: ", header

    def _create_response_object(self, body, header):
        """
        Helper method: Creates a response instance from the received package
        :param header: The header of the package received
        :param body: The body of the package received
        :return: response.Response
        """
        seq = header[response.Response.SEQ]
        try:
            request_obj = self._get_request(seq)
        except IndexError:
            return None
        self._packages.remove(request_obj)
        response_obj = request_obj.response(header, body)
        return response_obj

    def _add_received_response(self, response_object):
        if response_object is None:
            raise SpheroFatalError("RESPONSE IS NONE NONE NONE")
        with self._response_lock:
            self._responses.append(response_object)
        self._notify_request_received(response_object.seq)

    def _notify_request_received(self, seq):
        try:
            self._requests_waiting_response.pop(seq).set()
        except KeyError:
            # Probably received the message to late
            pass

    def _handle_msg_response(self, body, header):
        """
        Helper method for parsing incoming sync response messages.
        Creates the correct response objects and adds this to the
        list of incoming responses

        :param body: The raw body of the received package
        :type body: str or raw_data
        :param header: The header of the received package
        :type header: tuple
        """
        response_object = self._create_response_object(body, header)
        if response_object:
            self._add_received_response(response_object)
        else:
            print "received a message with no sender?"

    def _receive_data(self, length):
        """
        Helper method to receive the given amount of data from the device
        :param length: The length of the data to receive
        :return:
        :raise SpheroError: Raises a sphero error if there is any issues with receiving data from the device
        """
        data = ''
        for _ in xrange(length):
            try:
                data += self._bt_socket.recv(1)
            except bluetooth.BluetoothError as e:
                raise SpheroError("Failed to receive data from device:" + e.message)
        return data

    def _receiver(self):
        """
        Asynchronous receiver of incoming data.
        Converts incoming data to the appropriate response instances and add these to
        the correct incoming message queues.

        This method is started when the device is successfully connected, and stopped when the device is
        disconnected.
        :raise SpheroError:
        """

        while self._run_receive:
            try:
                if self._something_to_receive():
                    header = self._receive_header()
                    if Response.is_msg_response(header):
                        body = self._receive_data(header[Response.DLEN])
                        self._handle_msg_response(body, header)

                    elif AsyncMsg.is_async_msg(header):
                        header = AsyncMsg.convert_to_async_header(header)
                        body = self._receive_data(header[AsyncMsg.DLEN])
                        self._handle_async_msg(body, header)
                    else:
                        raise SpheroError("Unknown data received from sphero. Header: {}".format(header))
            except SpheroError:
                # TODO release all blocked threads waiting to receive data
                print "RECEIVER CRASHED"
                self._receiver_crashed = True
            #time.sleep(0.01)

    @staticmethod
    def prep_str(s):
        """
        Helper method to take a string and give a array of "bytes"
        """
        return [ord(c) for c in s]

    # CORE COMMANDS

    def ping(self):
        return self._write(request.Ping(self.seq))

    def set_rgb(self, r, g, b, persistent=False):
        # TODO verify values in range
        return self._write(request.SetRGB(self.seq, r, g, b, 0x01 if persistent else 0x00))

    def get_rgb(self):
        return self._write(request.GetRGB(self.seq))

    def get_version(self):
        raise NotImplementedError

    def get_device_name(self):
        # GET_DEVICE_NAME is not really part of the api,
        # it has changed to GET_BLUETOOTH_INFO.
        # Which returns both name and Bluetooth mac address.
        return self.get_bluetooth_info().name

    def set_device_name(self, new_name):
        """ Sets internal device name. (not announced bluetooth name).
        requires utf-8 encoded string. """
        return self._write(request.SetDeviceName(self.seq, *self.prep_str(new_name)))

    def get_bluetooth_info(self):
        return self._write(request.GetBluetoothInfo(self.seq))

    def set_auto_reconnect(self):
        raise NotImplementedError

    def get_auto_reconnect(self):
        raise NotImplementedError

    def get_power_state(self):
        return self._write(request.GetPowerState(self.seq))

    def set_power_notification(self, activated=True):
        return self._write(request.SetPowerNotification(self.seq, 0x01 if activated else 0x00))

    def sleep(self, wakeup=0, macro=0, orbbasic=0):
        return self._write(request.Sleep(self.seq, wakeup, macro, orbbasic))

    def get_voltage_trip_points(self):
        raise NotImplementedError

    def set_voltage_trip_points(self):
        raise NotImplementedError

    def set_inactivity_timeout(self):
        raise NotImplementedError

    def jump_to_bootloader(self):
        raise NotImplementedError

    def perform_level_1_diagnostics(self):
        return self._write(request.PerformLevel1Diagnostics(self.seq))

    def perform_level_2_diagnostics(self):
        raise NotImplementedError

    def clear_counters(self):
        raise NotImplementedError

    # SPHERO COMMANDS

    def set_time_value(self):
        raise NotImplementedError

    def poll_packet_times(self):
        raise NotImplementedError

    def set_heading(self, value):
        """value can be between 0 and 359"""
        return self._write(request.SetHeading(self.seq, value))

    def set_stabilization(self, state):
        """
        Turns off or on the internal stabilization of the sphero

        :param state: Sets stabilization on or off
        :type state: bool
        :rtype: response.Response
        :return: SimpleResponse
        """
        return self._write(request.SetStabilization(self.seq, state))

    def set_rotation_rate(self, val):
        """ value ca be between 0x00 and 0xFF:
            value is a multiplied with 0.784 degrees/s except for:
            0   --> 1 degrees/s
            255 --> jumps to 400 degrees/s

            :param val: Sets the new rotation rate
            :type val: int
            :rtype: response.Response
            :return: SimpleResponse
        """
        return self._write(request.SetRotationRate(self.seq, val))

    def set_application_configuration_block(self):
        raise NotImplementedError

    def get_application_configuration_block(self):
        raise NotImplementedError

    def reenable_demo_mode(self):
        raise NotImplementedError

    def get_chassis_id(self):
        raise NotImplementedError

    def set_chassis_id(self):
        raise NotImplementedError

    def self_level(self):
        # TODO: Implement self leveling
        raise NotImplementedError

    def set_data_streaming(self, new_ssc):
        # TODO WRITE DOCUMENTATION
        n = new_ssc.n
        m = new_ssc.m
        mask = new_ssc.mask1
        mask2 = new_ssc.mask2
        packet_cnt = new_ssc.num_packets
        result = self._write(request.SetDataStreaming(self.seq, n, m, mask, packet_cnt, mask2))
        if result.success:
            self._ssc = new_ssc
        return result

    def stop_data_streaming(self):
        """
        High level method to disable data streaming
        :return: response.SimpleResponse
        """
        stop_ssc = streaming.SensorStreamingConfig()
        stop_ssc.stream_none()
        return self.set_data_streaming(stop_ssc)

    def configure_collision_detection(self, meth=0x01, x_t=0x64, y_t=0x64, x_spd=0x64, y_spd=0x64, dead=0x64):
        # TODO WRITE DOCS
        return self._write(request.ConfigureCollisionDetection(self.seq, meth, x_t, y_t, x_spd, y_spd, dead))

    def set_back_led_output(self, value):
        """value can be between 0x00 and 0xFF"""
        return self._write(request.SetBackLEDOutput(self.seq, value))

    def roll(self, speed, heading, state=1):
        """
        :param speed: speed can have value between 0x00 and 0xFF
        :param heading: heading can have value between 0 and 359

        State:
        As of the 1.13 firmware version.
        State   Speed   Result
        1       > 0     Normal driving
        1       0       Rotate in place
        2       x       Force fast rotation
        0       x       Commence optimal braking to zero speed
        :return: SimpleResponse
        :rtype: response.Response
        """
        return self._write(request.Roll(self.seq, speed, heading, state))

    def set_boost_with_time(self, activate=True):
        return self._write(request.SetBoostWithTime(self.seq, activate))

    def set_raw_motor_values(self, left_mode=MotorMode.MOTOR_IGNORE, left_power=0x00,
                             right_mode=MotorMode.MOTOR_IGNORE, right_power=0x00):
        """
        Sets a raw value to one or both of Spheros engines.

        NOTE: This command will disable stabilization if booth modes aren't MotorMode.MOTOR_IGNORE. This would have the
        be re-enabled with the set_stabilization command

        :param left_mode: Sets the mode of the engine (see the MotorMode class)
        :type left_mode: MotorMode
        :param left_power: Sets the motor power for the left engine. Value in range 0x00 - 0xFF
        :param right_mode: Sets the mode of the engine (see the MotorMode class)
        :type right_mode: MotorMode
        :param right_power: Sets the motor power for the right engine. Value in range 0x00 - 0xFF
        :rtype: response.Response
        :return: SimpleResponse
        """
        return self._write(request.SetRawMotorValues(self.seq, left_mode, left_power, right_mode, right_power))

    def set_motion_timeout(self, timeout):
        # TODO WRITE DOC
        return self._write(request.SetMotionTimeout(self.seq, timeout))

    def set_option_flags(self, stay_on=False, vector_drive=False, leveling=False, tail_led=False, motion_timeout=False,
                         demo_mode=False, tap_light=False, tap_heavy=False, gyro_max=False):
        """
        Assigns the permanent option flags to the provided value and writes them to the config block for
        persistence across power cycles. See below for the bit definitions.

        :param stay_on: Set to prevent Sphero from immediately going to sleep when placed in the charger and connected \
        over Bluetooth

        :type stay_on: bool
        :param vector_drive: Set to enable Vector Drive, that is, when Sphero is stopped and a new roll command is issued it achieves the heading before moving along it
        :param leveling: Set to disable self-leveling when Sphero is inserted into the charger
        :param tail_led: Set to force the tail LED always on
        :param motion_timeout: Set to enable motion timeouts (see DID 02h, CID 34h)
        :param demo_mode: Set to enable retail Demo Mode (when placed in the charger ball runs a slow rainbow macro for 60 minutes and then goes to sleep).
        :param tap_light: Set double tap sensitivity to light
        :param tap_heavy: Set double tap sensitivity to heavy
        :param gyro_max: Enable gyro max async message

        :rtype: response.Response

        :return: SimpleResponse
        """
        flags = 0x0000
        flags |= 0x0001 if stay_on else 0x0000
        flags |= 0x0002 if vector_drive else 0x0000
        flags |= 0x0004 if leveling else 0x0000
        flags |= 0x0008 if tail_led else 0x0000
        flags |= 0x0010 if motion_timeout else 0x0000
        flags |= 0x0020 if demo_mode else 0x0000
        flags |= 0x0040 if tap_light else 0x0000
        flags |= 0x0080 if tap_heavy else 0x0000
        flags |= 0x0100 if gyro_max else 0x0000

        return self._write(request.SetOptionFlags(self.seq, flags))

    def get_option_flags(self):
        return self._write(request.GetOptionFlags(self.seq))

    def get_configuration_block(self):
        raise NotImplementedError

    def set_device_mode(self):
        raise NotImplementedError

    def run_macro(self):
        raise NotImplementedError

    def save_temporary_macro(self):
        raise NotImplementedError

    def reinit_macro(self):
        raise NotImplementedError

    def abort_macro(self):
        raise NotImplementedError

    def get_macro_status(self):
        raise NotImplementedError

    def set_macro_parameter(self):
        raise NotImplementedError

    def append_macro_chunk(self):
        raise NotImplementedError

    def erase_orbbasic_storage(self):
        raise NotImplementedError

    def append_orbbasic_fragment(self):
        raise NotImplementedError

    # BOOTLOADER COMMANDS (still looking for actual docs on these)

    def run_orbbasic_program(self):
        raise NotImplementedError

    def abort_orbbasic_program(self):
        raise NotImplementedError

    def begin_bootloader_reflash(self):
        raise NotImplementedError

    def set_bootloader_page(self):
        raise NotImplementedError

    def leave_bootloader(self):
        raise NotImplementedError

    def is_bootloader_page_blank(self):
        raise NotImplementedError

    def erase_user_config(self):
        raise NotImplementedError

    # Additional "higher-level" commands

    def configure_locator(self, x_pos, y_pos, yaw_tare=0x00, auto=True):
        """
        :param x_pos: in the range 0x00 - 0xff sets the new x position
        :param y_pos: in the range 0x00 - 0xff sets the new y position
        :param yaw_tare: in the range 0x00 - 0xff sert yaw tare
        :return: simple response
        """
        flags = 0x01 if auto else 0x00  # Could make the user set this
        return self._write(request.ConfigureLocator(self.seq, flags, x_pos, y_pos, yaw_tare))

    def read_locator(self):
        """
        This reads spheros current X, Y position, component velocities
        and SOG(speed over ground). Position is a signed value in cm.
        The component velocities are signed cm/sec while the SOG is
        unsigned cm/sec.
        :return: response.Response
        """
        return self._write(request.ReadLocator(self.seq))

    def stop(self):
        return self.roll(0, 0)

    # ASYNC CALLBACKS

    def set_collision_cb(self, collision_cb):
        """
        Used to set the callback method triggered when a collision is detected from the sphero.
        Configure_collision_detection() must be called to activate collision detection on the Sphero device

        The callback will be called with the collision data set as a parameter
        :param collision_cb: The callback method
        :type collision_cb: method or function
        """
        self._collision_cb = collision_cb

    def _on_collision(self, collision_data):
        """
        Helper method that is triggered on a collision
        :param collision_data:
        """
        if self._collision_cb:
            self._collision_cb(collision_data)

    def set_sensor_streaming_cb(self, streaming_cb):
        """
        Used to set the callback method triggered when sensor data is received from the sphero.
        set_data_streaming() must be called to activate sensor streaming on the Sphero device

        The callback will be called with the sensor data set as a parameter
        :param streaming_cb: The callback that should be triggered when data arrives from the sphero
        :type streaming_cb: method or function
        """
        self._streaming_cb = streaming_cb

    def _on_streaming(self, streaming_data):
        """
        Helper method that is triggered when sensor data is received from the sphero
        :param streaming_data:
        """
        self.sensors = streaming_data
        if self._streaming_cb:
            self._streaming_cb(streaming_data)

    def set_power_state_cb(self, power_state_cb):
        """
        Used to set the callback method triggered when power state data is received from the sphero.
        set_power_notification() must be called to activate power state streaming on the Sphero device

        The callback will be called with the power state data set as a parameter
        :param power_state_cb: The callback that should be triggered when power state data arrives from the sphero
        :type power_state_cb: method or function
        """
        self._power_state_cb = power_state_cb

    def _on_power_state_cb(self, power_state_data):
        """
        Helper method that is triggered when power state data is received from the sphero
        :param power_state_data:
        """
        if self._power_state_cb:
            self._power_state_cb(power_state_data)


if __name__ == '__main__':
    # FOR TESTING
    def power_cb(msg):
        print "CB:", msg

    def stream_cb(data):
        #print "CB:", data.sensor_data

        try:
            print "yaw", data.sensor_data[streaming.KEY_STRM_IMU_YAW_ANGLE]
        except KeyError:
            print "KEY ERROR"

    def collision_test_cb(data):
        print "collision", data

    # s1 = SpheroAPI(bt_name="Sphero-YGY", bt_addr="68:86:e7:03:24:54")  # SPHERO-YGY NO: 5
    # s2 = SpheroAPI(bt_name="Sphero-YGY", bt_addr="68:86:e7:03:22:95")  # SPHERO-ORB NO: 4
    s3 = SpheroAPI(bt_name="Sphero-YGY", bt_addr="68:86:e7:02:3a:ae")  # SPHERO-RWO NO: 2
    s3.connect()

    ssc = streaming.SensorStreamingConfig()
    ssc.stream_imu_angle()
    ssc.num_packets = 0
    ssc.sample_rate = 1

    s3.set_sensor_streaming_cb(stream_cb)
    #s3.set_power_state_cb(power_cb)
    #s3.set_collision_cb(collision_cb)
    #s3.configure_collision_detection()

    #s3.set_power_notification(True)

    s3.set_option_flags(motion_timeout=True)
    print "set timeout", s3.set_motion_timeout(5000).success

    s3.set_heading(0)

    s3.configure_locator(0, 0, yaw_tare=90, auto=False)
    s3.set_data_streaming(ssc)

    # DRIVE FORWARD
    s3.roll(50, 0)
    time.sleep(3.0)
    s3.roll(0, 0)
    print "fwd\n", s3.read_locator()
    time.sleep(3.0)

    # DRIVE FORWARD
    s3.roll(50, 180)
    time.sleep(3.0)
    s3.roll(0, 180)
    print "bcw\n", s3.read_locator()
    time.sleep(3.0)



    time.sleep(5)
    s3.disconnect()

