import random

from functools import wraps
from threading import Thread
import time
import math
from SpheroController import vectormovement

from SpheroController.tracablesphero import TraceableSphero
import ps3
import sphero
from sphero import SensorStreamingConfig
from util import Vector2D
from tracker import ImageGraphics as Ig, DrawError, FilterSpheroBlueCover
import util


class ControllableSphero(TraceableSphero):
    """
    A helper class for controlling spheros using PS3 controllers
    """
    def __init__(self, device):
        """

        :param device: The device to control
        :type device: sphero.SpheroAPI
        """
        super(ControllableSphero, self).__init__(device, name=device.bt_name)

        self.is_calibrating = False
        self.device = device
        self.filter = FilterSpheroBlueCover()
        self._ps3_controller = None

        # MOVEMENT CONTROL
        self.vector_control = vectormovement.SpheroVectorController(self.device)
        self.vector_control.start()
        self._motion_timeout = 5000

        self._sphero_clean_up_cb = None
        self._cmd_retries = 5

        # SENSOR STREAMING
        self._ssc = SensorStreamingConfig()

        # COLLISION DETECTION
        self.collision_data = None
        self.collision_detected = False

        # LIGHTS
        self.lights = True

        # Virtual Dot
        self.dot_draw_radius = 2
        self.is_inside_dot = False
        self.inside_dot_threshold = 15
        self.dot_pos = Vector2D(10, 10)
        self.dot_drive = False
        self.dot_speed_y = 0.0
        self.dot_speed_x = 0.0

        # BOUNCE
        self.bounce_thread = None
        self._run_bounce = True
        self.border = 130

        # SETUP
        self._setup_sphero()

    def _setup_sphero(self):
        self.device.set_option_flags(
            motion_timeout=True,
            tail_led=True,
            vector_drive=True
        )
        self.device.set_motion_timeout(self._motion_timeout)
        self.device.set_rgb(0xFF, 0xFF, 0xFF, True)

        self._configure_sensor_streaming()

        # Collision detection
        self.device.configure_collision_detection(x_t=0x80, y_t=0x80)
        self.device.set_collision_cb(self._on_collision_cb)

    def _configure_sensor_streaming(self):
        self._ssc.num_packets = SensorStreamingConfig.STREAM_FOREVER
        self._ssc.sample_rate = 20
        self._ssc.stream_odometer()
        self._ssc.stream_imu_angle()
        self._ssc.stream_velocity()
        self._ssc.stream_gyro()
        self._ssc.stream_gyro_raw()
        self.device.set_data_streaming(self._ssc)

    def handle_exceptions(cmd):
        @wraps(cmd)
        def inner(self, *args, **kwargs):
            try:
                return cmd(self, *args, **kwargs)
            except sphero.SpheroRequestError:
                # TODO: What todo here? - 5/3/14
                print "sphero request error", cmd.__name__
            except sphero.SpheroConnectionError:
                print "Connection error"
                self._on_sphero_disconnected()
            except sphero.SpheroFatalError:
                print "Sphero Fatal error"
                self._on_sphero_disconnected()
            except sphero.SpheroError:
                print "Sphero error"
        return inner

    def draw_graphics(self, image):
        if self.collision_detected:
            self.collision_detected = False
            border = 10
            width = self.screen_size[0] - border
            height = self.screen_size[1] - border
            try:
                Ig.draw_rectangle(image, (10, 10), width, height, util.Color((255, 0, 0)), border)
            except DrawError:
                pass

        super(ControllableSphero, self).draw_graphics(image)

        if self.is_calibrating:
            try:
                Ig.draw_text(image, "CALIBRATING", (self.pos+(-30, -20)), 0.3, util.Color((0, 255, 255)))
            except (DrawError, TypeError):
                pass

        if self.dot_drive:
            try:
                self._update_dot()
                red = util.Color((255, 0, 0))
                green = util.Color((0, 255, 0))
                color = green if self.is_inside_dot else red
                Ig.draw_circle(image, self.dot_pos, self.dot_draw_radius, color)
                Ig.draw_circle(image, self.dot_pos, self.inside_dot_threshold, color, border=1)
            except DrawError:
                pass

        if self.bounce_thread:
            width = self.screen_size[0] - (2*self.border)
            height = self.screen_size[1] - (2*self.border)

            try:
                Ig.draw_rectangle(image, (self.border, self.border), width, height, util.Color((0, 0, 200)))
            except DrawError:
                pass

    def dot_x(self, value):
        self.dot_speed_x = value * 20.0

    def dot_y(self, value):
        self.dot_speed_y = -value * 20.0

    def toggle_dot_drive(self):
        if not self.dot_drive:
            self.dot_drive = True
        else:
            self.dot_drive = False

    def _update_dot(self):
        self.dot_pos.x += self.dot_speed_x
        if self.dot_pos.x >= self.screen_size[0]:
            self.dot_pos.x = self.screen_size[0]
        elif self.dot_pos.x <= 0:
            self.dot_pos.x = 0

        self.dot_pos.y += self.dot_speed_y
        if self.dot_pos.y >= self.screen_size[1]:
            self.dot_pos.y = self.screen_size[1]
        elif self.dot_pos.y <= 0:
            self.dot_pos.y = 0

        if self.pos:
            path_to_dot = (self.dot_pos - self.pos)

            if self.dot_drive:
                if path_to_dot.magnitude > self.inside_dot_threshold:
                    self.is_inside_dot = False
                    self.vector_control.direction = path_to_dot.angle
                    self.vector_control.speed = min(path_to_dot.magnitude / 3.0, 80)
                else:
                    self.is_inside_dot = True
                    self.vector_control.speed = 0

    def _map_controls(self, ps3_controller):
        ps3_controller.set_events(
            button_press={
                ps3.BUTTON_START: self.disconnect,

                ps3.BUTTON_CIRCLE: self.calibrate,
                ps3.BUTTON_SQUARE: self.toggle_lights,
                ps3.BUTTON_CROSS: self.lights_random_color,
                ps3.BUTTON_TRIANGLE: self.ping,

                ps3.BUTTON_JOY_PAD_UP: self.toggle_bouncing_ball,
                ps3.BUTTON_JOY_PAD_RIGHT: self.toggle_dot_drive
            },
            button_release={

            },
            axis={
                ps3.AXIS_JOYSTICK_R_VER: self.set_y,
                ps3.AXIS_JOYSTICK_R_HOR: self.set_x,
                ps3.AXIS_JOYSTICK_L_HOR: self.dot_x,
                ps3.AXIS_JOYSTICK_L_VER: self.dot_y
            }
        )

    def _on_collision_cb(self, collision_data):
        self.collision_detected = True
        self.collision_data = collision_data

    def toggle_bouncing_ball(self):
        if self.bounce_thread is None:
            self._run_bounce = True
            self.bounce_thread = Thread(target=self.bouncing_ball, name="Sphero Bounce")
            self.bounce_thread.daemon = True
            self.bounce_thread.start()
        else:
            self._run_bounce = False
            self.bounce_thread = None

    def bouncing_ball(self):
        screen_x = self.screen_size[0]
        screen_y = self.screen_size[1]

        self.vector_control.direction = 45
        center = Vector2D(screen_x/2, screen_y/2)

        while self._run_bounce:
            self.vector_control.speed = 60
            pos = self.last_valid_pos
            path_to_center = (center - pos)
            if pos.x >= screen_x-self.border:
                self.vector_control.direction = path_to_center.angle
                # self.vector_control.vector.x = -abs(self.vector_control.vector.x)

            elif pos.x <= self.border:
                self.vector_control.direction = path_to_center.angle

                # self.vector_control.vector.x = abs(self.vector_control.vector.x)

            if pos.y >= screen_y - self.border:
                self.vector_control.direction = path_to_center.angle

                # self.vector_control.vector.y = -abs(self.vector_control.vector.y)

            elif pos.y <= self.border:
                # self.vector_control.vector.y = abs(self.vector_control.vector.y)
                self.vector_control.direction = path_to_center.angle

            time.sleep(0.01)
        self.vector_control.speed = 0.0

    def set_ps3_controller(self, ps3_controller):
        """
        Used to set and map the PS3 controller to run the sphero commands

        :param ps3_controller:
        :type ps3_controller: ps3.PS3C
        """
        self._ps3_controller = ps3_controller
        self._map_controls(ps3_controller)

    @handle_exceptions
    def calibrate(self):
        self.is_calibrating = True
        self.vector_control.stop()
        self.calibrate_direction()
        self.vector_control.start()
        self.is_calibrating = False

    def set_x(self, value):
        #self.vector_control.turn_rate = math.tan(value) * -5

        self.vector_control.vector.x = value * 75.0

    def set_y(self, value):
        #self.vector_control.speed = abs(value * 255.0)

        self.vector_control.vector.y = value * -75.0

    @handle_exceptions
    def disconnect(self):
        self.device.disconnect()
        self._on_sphero_disconnected()

    @handle_exceptions
    def lights_random_color(self):
        r = random.randrange(0, 255)
        g = random.randrange(0, 255)
        b = random.randrange(0, 255)
        print "Lights random color: ", self.device.set_rgb(r, g, b, True).success

    @handle_exceptions
    def toggle_lights(self):
        if not self.lights:
            self.device.set_rgb(255, 255, 255, True)
            self.lights = True
            return
        self.device.set_rgb(0, 0, 0, True)
        self.lights = False

    @handle_exceptions
    def ping(self):
        print "Ping success:", self.device.ping().success

    @handle_exceptions
    def get_battery_state(self):
        print self.device.get_power_state()

    def set_sphero_disconnected_cb(self, cb):
        self._sphero_clean_up_cb = cb

    def _on_sphero_disconnected(self):
        self.vector_control.stop()
        print "PS3 / SPHERO clean up"
        if self._ps3_controller:
            self._ps3_controller.free()
        self._sphero_clean_up_cb(self.device)