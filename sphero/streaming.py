from collections import namedtuple
from sphero import response
from error import SpheroError
from constants import *
import time


class MaskUtil(object):
    """
    Helper class for managing streaming flags for sphero streaming config
    """
    ZERO_MASK = 0x00000000
    ALL_MASK = 0xFFFFFFFF

    @staticmethod
    def add_field(mask, bitfield):
        return mask | bitfield

    @staticmethod
    def rem_field(mask, bitfield):
        return mask - bitfield

    @staticmethod
    def empty_mask():
        return MaskUtil.ZERO_MASK

    @staticmethod
    def set_all():
        return MaskUtil.ALL_MASK

    @staticmethod
    def set_value_active(mask, activate, bit_field):
        if activate:
            mask = MaskUtil.add_field(mask, bit_field)
        else:
            mask = MaskUtil.rem_field(mask, bit_field)
        return mask

    @staticmethod
    def is_set(mask, base_mask):
        return bool(base_mask & mask)

    @staticmethod
    def value_state_list(mask):
        iter_mask = 0x80000000
        state_list = []
        for _ in xrange(iter_mask.bit_length()):
            state_list.append((iter_mask, MaskUtil.is_set(iter_mask, mask)))
            iter_mask >>= 1
        return state_list

    @staticmethod
    def add_names(value_state_list, value_names):
        named_state_list = []
        for value_id, value in value_state_list:
            try:
                named_state_list.append((value_names[value_id], value))
            except KeyError:
                pass
        return named_state_list

    @staticmethod
    def print_mask(mask, value_names):
        named_state = MaskUtil.add_names(MaskUtil.value_state_list(mask), value_names)
        for name, state in named_state:
            print "{} set: {}".format(name, state)


class Mask1(object):
    ACC_X_RAW = 0x80000000
    ACC_Y_RAW = 0x40000000
    ACC_Z_RAW = 0x20000000

    GYRO_X_RAW = 0x10000000
    GYRO_Y_RAW = 0x08000000
    GYRO_Z_RAW = 0x04000000

    EMF_RAW_RIGHT_MOTOR = 0x00400000
    EMF_RAW_LEFT_MOTOR = 0x00200000

    PWM_RAW_LEFT_MOTOR = 0x00100000
    PWM_RAW_RIGHT_MOTOR = 0x00080000

    IMU_PITCH_ANGLE = 0x00040000
    IMU_ROLL_ANGLE = 0x00020000
    IMU_YAW_ANGLE = 0x00010000

    ACC_X = 0x00008000
    ACC_Y = 0x00004000
    ACC_Z = 0x00002000

    GYRO_X = 0x00001000
    GYRO_Y = 0x00000800
    GYRO_Z = 0x00000400

    EMF_RIGHT_MOTOR = 0x00000040
    EMF_LEFT_MOTOR = 0x00000020

    mask1_names = {
        ACC_X_RAW: KEY_STRM_X_RAW,
        ACC_Y_RAW: KEY_STRM_Y_RAW,
        ACC_Z_RAW: KEY_STRM_Z_RAW,

        GYRO_X_RAW: KEY_STRM_GYRO_X_RAW,
        GYRO_Y_RAW: KEY_STRM_GYRO_Y_RAW,
        GYRO_Z_RAW: KEY_STRM_GYRO_Z_RAW,

        EMF_RAW_RIGHT_MOTOR: KEY_STRM_EMF_RAW_RIGHT_MOTOR,
        EMF_RAW_LEFT_MOTOR: KEY_STRM_EMF_RAW_LEFT_MOTOR,

        PWM_RAW_LEFT_MOTOR: KEY_STRM_PWM_RAW_LEFT_MOTOR,
        PWM_RAW_RIGHT_MOTOR: KEY_STRM_PWM_RAW_RIGHT_MOTOR,

        IMU_PITCH_ANGLE: KEY_STRM_IMU_PITCH_ANGLE,
        IMU_ROLL_ANGLE: KEY_STRM_IMU_ROLL_ANGLE,
        IMU_YAW_ANGLE: KEY_STRM_IMU_YAW_ANGLE,

        ACC_X: KEY_STRM_ACC_X,
        ACC_Y: KEY_STRM_ACC_Y,
        ACC_Z: KEY_STRM_ACC_Z,

        GYRO_X: KEY_STRM_GYRO_X,
        GYRO_Y: KEY_STRM_GYRO_Y,
        GYRO_Z: KEY_STRM_GYRO_Z,

        EMF_LEFT_MOTOR: KEY_EMF_LEFT_MOTOR,
        EMF_RIGHT_MOTOR: KEY_EMF_RIGHT_MOTOR
    }

    def __init__(self):
        super(Mask1, self).__init__()
        self.mask1 = MaskUtil.empty_mask()

    def stream_acc_raw(self, activate=True):
        """
        Activate/deactivate streaming of raw accelerometer data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        acc_raw_mask = Mask1.ACC_X_RAW | Mask1.ACC_Y_RAW | Mask1.ACC_Z_RAW
        self.mask1 = MaskUtil.set_value_active(self.mask1, activate, acc_raw_mask)

    def stream_acc(self, activate=True):
        """
        Activate/deactivate streaming of filtered accelerometer data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        acc_mask = Mask1.ACC_X | Mask1.ACC_Y | Mask1.ACC_Z
        self.mask1 = MaskUtil.set_value_active(self.mask1, activate, acc_mask)

    def stream_gyro_raw(self, activate=True):
        """
        Activate/deactivate streaming of raw gyro data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        gyro_raw_mask = Mask1.GYRO_X_RAW | Mask1.GYRO_Y_RAW | Mask1.GYRO_Z_RAW
        self.mask1 = MaskUtil.set_value_active(self.mask1, activate, gyro_raw_mask)

    def stream_gyro(self, activate=True):
        """
        Activate/deactivate streaming of filtered gyro data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        gyro_mask = Mask1.GYRO_X | Mask1.GYRO_Y | Mask1.GYRO_Z
        self.mask1 = MaskUtil.set_value_active(self.mask1, activate, gyro_mask)

    def stream_motor_data_raw(self, activate=True):
        """
        Activate/deactivate streaming of raw motor data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        motor_raw_mask = Mask1.EMF_RAW_LEFT_MOTOR | Mask1.EMF_RAW_RIGHT_MOTOR | Mask1.PWM_RAW_LEFT_MOTOR | \
                         Mask1.PWM_RAW_RIGHT_MOTOR
        self.mask1 = MaskUtil.set_value_active(self.mask1, activate, motor_raw_mask)

    def stream_motor_data(self, activate=True):
        """
        Activate/deactivate streaming of filtered motor data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        motor_mask = Mask1.EMF_LEFT_MOTOR | Mask1.EMF_RIGHT_MOTOR
        self.mask1 = MaskUtil.set_value_active(self.mask1, activate, motor_mask)

    def stream_imu_angle(self, activate=True):
        """
        Activate/deactivate streaming of imu angle data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        imu_mask = Mask1.IMU_PITCH_ANGLE | Mask1.IMU_ROLL_ANGLE | Mask1.IMU_YAW_ANGLE
        self.mask1 = MaskUtil.set_value_active(self.mask1, activate, imu_mask)

    def stream_all(self):
        self.stream_acc_raw()
        self.stream_acc()
        self.stream_gyro_raw()
        self.stream_gyro()
        self.stream_motor_data_raw()
        self.stream_motor_data()
        self.stream_imu_angle()

    def stream_none(self):
        self.mask1 = MaskUtil.empty_mask()

    def get_values(self):
        state_lst = MaskUtil.value_state_list(self.mask1)
        return MaskUtil.add_names(state_lst, self.mask1_names)

    def print_mask(self):
        MaskUtil.print_mask(self.mask1, self.mask1_names)


class Mask2(object):
    Q0 = 0x80000000
    Q1 = 0x40000000
    Q2 = 0x20000000
    Q3 = 0x10000000

    ODOMETER_X = 0x08000000
    ODOMETER_Y = 0x04000000

    ACCEL_ONE = 0x02000000

    VELOCITY_X = 0x01000000
    VELOCITY_Y = 0x00800000

    mask2_names = {
        Q0: KEY_STRM_Q0,
        Q1: KEY_STRM_Q1,
        Q2: KEY_STRM_Q2,
        Q3: KEY_STRM_Q3,

        ODOMETER_X: KEY_STRM_ODOMETER_X,
        ODOMETER_Y: KEY_STRM_ODOMETER_Y,

        ACCEL_ONE: KEY_STRM_ACCEL_ONE,

        VELOCITY_X: KEY_STRM_VELOCITY_X,
        VELOCITY_Y: KEY_STRM_VELOCITY_Y
    }

    def __init__(self):
        super(Mask2, self).__init__()
        self.mask2 = MaskUtil.empty_mask()

    def stream_odometer(self, activate=True):
        """
        Activate/deactivate streaming of odometer data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        odometer = Mask2.ODOMETER_X | Mask2.ODOMETER_Y
        self.mask2 = MaskUtil.set_value_active(self.mask2, activate, odometer)

    def stream_velocity(self, activate=True):
        """
        Activate/deactivate streaming of velocity data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        velocity = Mask2.VELOCITY_X | Mask2.VELOCITY_Y
        self.mask2 = MaskUtil.set_value_active(self.mask2, activate, velocity)

    def stream_acceleration_one(self, activate=True):
        """
        Activate/deactivate streaming of velocity on acc from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        self.mask2 = MaskUtil.set_value_active(self.mask2, activate, Mask2.ACCEL_ONE)

    def stream_quaternion(self, activate=True):
        """
        Activate/deactivate streaming of quaternion data from device
        :param activate: True to activate False to deactivate
        :type activate: bool
        """
        quaternion = Mask2.Q0 | Mask2.Q1 | Mask2.Q2 | Mask2.Q3
        self.mask2 = MaskUtil.set_value_active(self.mask2, activate, quaternion)

    def stream_all(self):
        self.stream_odometer()
        self.stream_velocity()
        self.stream_acceleration_one()
        self.stream_quaternion()

    def stream_none(self):
        self.mask2 = MaskUtil.empty_mask()

    def get_values(self):
        state_lst = MaskUtil.value_state_list(self.mask2)
        return MaskUtil.add_names(state_lst, self.mask2_names)

    def print_mask(self):
        MaskUtil.print_mask(self.mask2, self.mask2_names)


class SensorStreamingConfig(Mask1, Mask2):
    """
    Class used to create a streaming config for the sphero device. It is used to specify which sensor data and the
    frequency and number of packages that should be streamed from the device
    """

    STREAM_FOREVER = 0
    MAX_SAMPLE_RATE_SPHERO = 400

    def __init__(self):
        super(SensorStreamingConfig, self).__init__()
        self.n = 420
        self.m = 1
        self.num_packets = 0

    @property
    def sample_rate(self):
        """
        Returns the current number of samples set to be streamed per/sec from the device in this config
        :return: Samples/sec
        :rtype: float or int
        """
        return ssc.MAX_SAMPLE_RATE_SPHERO / self.n

    @sample_rate.setter
    def sample_rate(self, packets_sec):
        """
        Set the number of samples per second to stream
        :param packets_sec: The number of packets the device should stream each second
        :type packets_sec: int or float
        :raise SpheroError: If a number is 0
        """
        try:
            if packets_sec <= 0:
                raise SpheroError("Sample rate must be a number larger than 0")
            self.n = int(self.MAX_SAMPLE_RATE_SPHERO / packets_sec)
        except ZeroDivisionError:
            self.n = 0

    def stream_all(self):
        """
        Is used to activate streaming of all sensors from the sphero
        """
        Mask1.stream_all(self)
        Mask2.stream_all(self)

    def stream_none(self):
        """
        Is used to set streaming of no sensors from the sphero
        """
        Mask1.stream_none(self)
        Mask2.stream_none(self)

    print_mask1 = Mask1.print_mask
    print_mask2 = Mask2.print_mask

    def print_streaming_config(self):
        """
        Prints the current config of the streaming set in this class
        """
        print "MASK 1"
        Mask1.print_mask(self)
        print "MASK 2"
        Mask2.print_mask(self)

    def get_streaming_config(self):
        """
        Return the current state of the streaming config
        :return: Streaming config
        """
        return Mask1.get_values(self) + Mask2.get_values(self)


class SensorBase(object):
    """
    Base class for all sensor classes
    """

    _data = {}

    def set_data(self, streaming_data):
        """
        Helper method: Parses sensor data to the set keys in _data. Used to set data to each sensor class
        from the received sensor data
        :param streaming_data: The dict that holds all of the sensor data
        :type streaming_data: dict
        """
        for key in self._data:
            try:
                self._data[key] = streaming_data[key]
            except KeyError:
                pass


class Motor(SensorBase):
    """
    Holds motor data streamed from device
    """
    def __init__(self):
        super(Motor, self).__init__()
        self._emf_raw_tuple = namedtuple("EmfRaw", "left, right")
        self._pwm_raw_tuple = namedtuple("PwmRaw", "left, right")
        self._emf_filtered_tuple = namedtuple("EmfFiltered", "left, right")

        self._data = {
            KEY_STRM_EMF_RAW_LEFT_MOTOR: None,
            KEY_STRM_EMF_RAW_RIGHT_MOTOR: None,
            KEY_STRM_PWM_RAW_LEFT_MOTOR: None,
            KEY_STRM_PWM_RAW_RIGHT_MOTOR: None,
            KEY_EMF_LEFT_MOTOR: None,
            KEY_EMF_RIGHT_MOTOR: None
        }

    @property
    def emf_raw(self):
        """
        Returns the motor back EMF(Electromotive Force) data received from the device.

        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device

        :return: Motor EMF in unit cm
        :rtype: collections.namedtuple
        """

        left, right = None, None
        unit = 22.5
        try:
            left = self._data[KEY_STRM_EMF_RAW_LEFT_MOTOR] * unit
            right = self._data[KEY_STRM_EMF_RAW_RIGHT_MOTOR] * unit
        except TypeError:
            pass
        return self._emf_raw_tuple(left, right)

    @property
    def pwm_raw(self):
        """
        Return the motor PWM(Pulse Width Modulation) data received from the device.

        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device

        :return: motor PWM
        :rtype: collections.namedtuple
        """
        left = self._data[KEY_STRM_PWM_RAW_LEFT_MOTOR]
        right = self._data[KEY_STRM_PWM_RAW_RIGHT_MOTOR]
        return self._pwm_raw_tuple(left, right)

    @property
    def emf_filtered(self):
        """
        Returns the filtered motor back EMF(Electromotive Force) data received from the device.

        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device

        :return: Motor EMF filtered in unit cm
        :rtype: collections.namedtuple
        """

        left, right = None, None
        unit = 22.5
        try:
            left = self._data[KEY_EMF_LEFT_MOTOR] * unit
            right = self._data[KEY_EMF_LEFT_MOTOR] * unit
        except TypeError:
            pass
        return self._emf_filtered_tuple(left, right)


class Quaternion(SensorBase):
    """
    Holds data for streamed quaternion position
    """
    def __init__(self):
        super(Quaternion, self).__init__()
        self._quaternion_tuple = namedtuple("Quaternion", "q1 q2 q3 q4")

        self._data = {
            KEY_STRM_Q0: None,
            KEY_STRM_Q1: None,
            KEY_STRM_Q2: None,
            KEY_STRM_Q3: None
        }

    def __str__(self):
        return str(self.__repr__())

    def __repr__(self):
        return self.values

    @property
    def values(self):
        """
        Returns the quaternion position of the device.

        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device
        :return: quaternion position of the device
        :rtype: collections.namedtuple
        """
        q0, q1, q2, q3 = None, None, None, None
        unit = 1.0/10000.0
        try:
            q0 = self._data[KEY_STRM_Q0] * unit
            q1 = self._data[KEY_STRM_Q1] * unit
            q2 = self._data[KEY_STRM_Q2] * unit
            q3 = self._data[KEY_STRM_Q3] * unit
        except TypeError:
            pass
        return self._quaternion_tuple(q0, q1, q2, q3)


class Velocity(SensorBase):
    """
    Holds data for Velocity data streamed from device
    """
    def __init__(self):
        super(Velocity, self).__init__()
        self._velocity_tuple = namedtuple("Velocity", "x, y, acc")

        self._data = {
            KEY_STRM_VELOCITY_X: None,
            KEY_STRM_VELOCITY_Y: None,
            KEY_STRM_ACCEL_ONE: None
        }

    def __str__(self):
        return str(self.__repr__())

    def __repr__(self):
        return self.velocity

    @property
    def velocity(self):
        """
        Return the velocity data received from the device.
        Velocity is mm/s and AccelOne is in the unit 1mG

        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device

        :return: Imu angles in degrees
        :rtype: collections.namedtuple
        """
        x = self._data[KEY_STRM_VELOCITY_X]
        y = self._data[KEY_STRM_VELOCITY_Y]
        one = self._data[KEY_STRM_ACCEL_ONE]
        return self._velocity_tuple(x, y, one)


class Odometer(SensorBase):
    """
    Holds data streamed from spheros locator position
    """
    def __init__(self):
        super(Odometer, self).__init__()
        self._location_tuple = namedtuple("Odometer", "x, y")

        self._data = {
            KEY_STRM_ODOMETER_X: None,
            KEY_STRM_ODOMETER_Y: None
        }

    def __str__(self):
        return str(self.__repr__())

    def __repr__(self):
        return self.pos

    @property
    def pos(self):
        """
        Return the internal position data received from the device.
        Location unit is cm

        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device

        :return: Internal sphero locator position
        :rtype: collections.namedtuple
        """
        x = self._data[KEY_STRM_ODOMETER_X]
        y = self._data[KEY_STRM_ODOMETER_X]
        return self._location_tuple(x, y)


class Imu(SensorBase):
    def __init__(self):
        super(Imu, self).__init__()
        self._imu_tuple = namedtuple("IMU", "pitch roll yaw")

        self._data = {
            KEY_STRM_IMU_PITCH_ANGLE: None,
            KEY_STRM_IMU_ROLL_ANGLE: None,
            KEY_STRM_IMU_YAW_ANGLE: None,
        }

    def __str__(self):
        return str(self.__repr__())

    def __repr__(self):
        return self.angle

    @property
    def angle(self):
        """
        Returns imu angle in deg -180 to 180.

        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device

        :return: Imu angles in degrees
        :rtype: collections.namedtuple
        """
        pitch = self._data[KEY_STRM_IMU_PITCH_ANGLE]
        roll = self._data[KEY_STRM_IMU_ROLL_ANGLE]
        yaw = self._data[KEY_STRM_IMU_YAW_ANGLE]
        return self._imu_tuple(pitch, roll, yaw)


class Accelerometer(SensorBase):
    """
    Holds a sample of streamed Accelerometer data
    """

    def __init__(self):
        super(Accelerometer, self).__init__()
        self._acc_tuple = namedtuple("AxisRaw", "x y z")
        self._acc_mg_tuple = namedtuple("AxisMilliG", "x y z")
        self._acc_filtered_tuple = namedtuple("AxisFilteredG", "x y z")

        self._data = {
            KEY_STRM_X_RAW: None,
            KEY_STRM_Y_RAW: None,
            KEY_STRM_Z_RAW: None,
            KEY_STRM_ACC_X: None,
            KEY_STRM_ACC_Y: None,
            KEY_STRM_ACC_Z: None
        }

    @property
    def acc_raw(self):
        """
        Returns raw accelerometer data.

        Values are set to None if no data is received from sphero.
        Streaming of Accelerometer data must be activated on the device

        :return: Accelerometer raw data
        :rtype: collections.namedtuple
        """
        x, y, z = None, None, None
        try:
            x = self._data[KEY_STRM_X_RAW]
            y = self._data[KEY_STRM_Y_RAW]
            z = self._data[KEY_STRM_Z_RAW]
        except TypeError:
            pass
        return self._acc_tuple(x, y, z)

    @property
    def acc_mg(self):
        """
        Returns accelerometer data in mg.

        Values are set to None if no data is received from sphero.
        Streaming of Accelerometer data must be activated on the device

        :return: Accelerometer data in mG
        :rtype: collections.namedtuple
        """
        x, y, z = self.acc_raw
        try:
            x *= 4.0
            y *= 4.0
            z *= 4.0
        except TypeError:
            pass
        return self._acc_mg_tuple(x, y, z)

    @property
    def acc_filtered(self):
        """
        Returns filtered accelerometer data in G.

        Values are set to None if no data is received from sphero.
        Streaming of Accelerometer data must be activated on the device

        :return: Filtered Accelerometer data in G
        :rtype: collections.namedtuple
        """
        x, y, z, = None, None, None
        unit = 1.0 / 4096.0
        try:
            x = self._data[KEY_STRM_ACC_X] * unit
            y = self._data[KEY_STRM_ACC_Y] * unit
            z = self._data[KEY_STRM_ACC_Z] * unit
        except TypeError:
            pass
        return self._acc_filtered_tuple(x, y, z)


class Gyro(SensorBase):
    """
    Holds a sample of streamed Gyro data
    """
    def __init__(self):
        super(Gyro, self).__init__()
        self._gyro_raw_tuple = namedtuple("GyroRaw", "x y z")
        self._gyro_raw_degrees_tuple = namedtuple("GyroDegrees", "x y z")
        self._gyro_filtered_tuple = namedtuple("GyroDPS", "x y z")

        self._data = {
            KEY_STRM_GYRO_X_RAW: None,
            KEY_STRM_GYRO_Y_RAW: None,
            KEY_STRM_GYRO_Z_RAW: None,
            KEY_STRM_GYRO_X: None,
            KEY_STRM_GYRO_Y: None,
            KEY_STRM_GYRO_Z: None
        }

    @property
    def gyro_dps(self):
        """
        Returns gyro value as Degrees per sec.
        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device
        :return: Degrees per sec
        :rtype: collections.namedtuple
        """
        x, y, z = None, None, None
        unit = 0.1
        try:
            x = self._data[KEY_STRM_GYRO_X] * unit
            y = self._data[KEY_STRM_GYRO_Y] * unit
            z = self._data[KEY_STRM_GYRO_Z] * unit
        except TypeError:
            pass
        return self._gyro_filtered_tuple(x, y, z)

    @property
    def gyro_raw(self):
        """
        Returns raw gyro data
        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device
        :return: gyro data
        :rtype: collections.namedtuple
        """
        x = self._data[KEY_STRM_GYRO_X_RAW]
        y = self._data[KEY_STRM_GYRO_Y_RAW]
        z = self._data[KEY_STRM_GYRO_Z_RAW]
        return self._gyro_raw_tuple(x, y, z)

    @property
    def gyro_degrees(self):
        """
        Returns gyro data in degrees
        Values are set to None if no data is received from sphero.
        Streaming of Gyro data must be activated on the device
        :return: gyro data in degrees
        :rtype: collections.namedtuple
        """
        x, y, z = self.gyro_raw
        try:
            x *= 0.068
            y *= 0.068
            z *= 0.068
        except TypeError:
            pass
        gyro_raw_degrees = self._gyro_raw_degrees_tuple(x, y, z)
        return gyro_raw_degrees


class SensorStreamingResponse(response.AsyncMsg):
    # TODO: Add docs for this class - 4/16/14

    def __init__(self, header, data, ssc):
        super(SensorStreamingResponse, self).__init__(header, data)
        self.timestamp = time.time()

        self.raw_sensor_data = {}
        self.ssc = ssc
        self.raw_sensor_data = self._parse_sensor_data(ssc)

        # SENSOR data holders
        self.gyro = Gyro()
        self.accelerometer = Accelerometer()
        self.imu = Imu()
        self.motor = Motor()
        self.odometer = Odometer()
        self.velocity = Velocity()
        self.quaternion = Quaternion()

        # Parse received sensor data
        self._map_sensor_data(self.raw_sensor_data)

    def _parse_sensor_data(self, ss_conf):
        """
        Parse raw data from sphero into a dict where keys are DataName and Value is Sensor Data
        :param ss_conf:
        :return:
        """
        sensor_data = {}

        streaming_config = ss_conf.get_streaming_config()

        data_offset = 0
        for sensor_name, is_activated in streaming_config:
            if is_activated:
                sensor_data[sensor_name] = self.body[data_offset]
                data_offset += 1
        return sensor_data

    def _map_sensor_data(self, sensor_data):
        """
        Helper method: maps data to the correct classes
        :param sensor_data:
        :return:
        """
        for key, value in self.__dict__.iteritems():
            if isinstance(value, SensorBase):
                value.set_data(sensor_data)

    @property
    def fmt(self):
        return '!%dhb' % ((self.dlen - 1) / 2)


if __name__ == "__main__":
    def on_data(streaming):
        """:type streaming: SensorStreamingResponse"""
        print "DATA"
        print streaming.gyro.gyro_dps
        print streaming.gyro.gyro_degrees
        print streaming.gyro.gyro_raw

        print streaming.accelerometer.acc_raw
        print streaming.accelerometer.acc_mg
        print streaming.accelerometer.acc_filtered
        print streaming.imu

        print streaming.motor.emf_raw
        print streaming.motor.pwm_raw
        print streaming.motor.emf_filtered

        print streaming.odometer
        print streaming.velocity.velocity.x

        print streaming.quaternion

    import sphero
    import time
    manager = sphero.SpheroManager()
    s1 = manager.get_available_device()
    if s1 is None:
        print "No device found"
        exit()

    #s1 = sphero.SpheroAPI(bt_name="Sphero-YGY", bt_addr="68:86:e7:03:22:95")  # 4
    ssc = SensorStreamingConfig()
    ssc.sample_rate = 1
    ssc.num_packets = 1

    ssc.stream_gyro()
    ssc.stream_gyro_raw()

    ssc.stream_acc_raw()
    ssc.stream_acc()

    ssc.stream_imu_angle()

    ssc.stream_motor_data_raw()
    ssc.stream_motor_data()

    ssc.stream_odometer()
    ssc.stream_velocity()
    ssc.stream_acceleration_one()

    ssc.stream_quaternion()

    try:
        s1.connect()
    except sphero.SpheroConnectionError:
        print "cant connect to device"
    else:
        s1.set_sensor_streaming_cb(on_data)
        s1.set_data_streaming(ssc)
    time.sleep(10)
    s1.disconnect()



