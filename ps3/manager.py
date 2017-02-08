import pygame
from pygame import joystick
from threading import Thread
from ps3 import PS3C
import ps3
import time


class PS3manager(object):
    def __init__(self):
        super(PS3manager, self).__init__()
        self._controllers = []
        self._listen = False

        pygame.display.init()

        self._init_controllers()

    def start(self):
        """
        Start listening for PS3 controller events
        """
        if not self._listen:
            thread = Thread(target=self._run_event_loop, name="PS3ManagerEventThread")
            thread.daemon = True
            thread.start()

    def stop(self):
        """
        Stops listening for PS3 controller events
        """
        self._listen = False

    @property
    def num_controllers(self):
        return len(self._controllers)

    def get_available_controller(self):
        for ctrl in self.controllers():
            if not ctrl.in_use:
                ctrl.in_use = True
                return ctrl
        else:
            return None

    def controllers(self):
        """
        Returns a list of all the available PS3 controllers
        :return: list of PS3C objects of available controllers
        :rtype: list
        """
        return self._controllers

    def _init_controllers(self):
        """
        Helper method: Setup available PS3 controllers
        """
        if not joystick.get_init():
            joystick.init()
        for joy_id in xrange(joystick.get_count()):
            js = joystick.Joystick(joy_id)
            if PS3C.is_ps3_controller(js.get_name()):
                js.init()
                ps3ctrl = PS3C(js)
                self._controllers.append(ps3ctrl)

    @staticmethod
    def _ps3_event(event):
        """
        Helper method: checks if pygame.event is one of the supported joystick event / ps3 controller event
        """
        return event.type in [pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP, pygame.JOYAXISMOTION]

    @staticmethod
    def _event_for_this_controller(ctrl, event):
        return ctrl.id == event.joy

    def _handle_ps3_event(self, event):
        """
        Helper method: handles pygame ps3 event to be handled by the correct ps3 device
        """
        for ctrl in self._controllers:
            if self._event_for_this_controller(ctrl, event):
                ctrl.handle_event(event)

    def _run_event_loop(self):
        """
        Helper method: runs the event loop than listens for incoming ps3 events
        """
        self._listen = True
        while self._listen:
            for event in pygame.event.get():
                if self._ps3_event(event):
                    self._handle_ps3_event(event)
            time.sleep(1.0 / 50.0)


# EXAMPLE CODE

if __name__ == "__main__":
    # EXAMPLE CODE FOR PS3 MANAGER

    # EXAMPLE CB's
    def button_down_cb():
        print "Button down"

    def button_up_cb():
        print "Button up"

    def axis_cb(value):
        print "axis:", value

    # INIT MANAGER
    manager = PS3manager()

    # SETUP CALLBACKS FOR EACH CONTROLLER
    for controller in manager.controllers():
        # EXAMPLE SET SINGLE CB EVENT
        controller.set_button_press_event(ps3.BUTTON_CIRCLE, button_down_cb)
        controller.set_button_release_event(ps3.BUTTON_CIRCLE, button_up_cb)
        controller.set_axis_change_event(ps3.AXIS_JOYSTICK_R_VER, axis_cb)

        # EXAMPLE SET MULTIPLE CB SAME EVENT TYPE
        controller.set_axis_change_events({
            ps3.AXIS_JOY_PAD_LEFT: axis_cb,
            ps3.AXIS_JOY_PAD_UP: axis_cb,
            ps3.AXIS_JOY_PAD_DOWN: axis_cb,
            ps3.AXIS_JOY_PAD_RIGHT: axis_cb
        })

        # EXAMPLE SET MULTIPLE EVENTS ALL TYPES
        controller.set_events(
            button_press={
                ps3.BUTTON_SQUARE: button_down_cb
            },
            button_release={
                ps3.BUTTON_JOY_PAD_DOWN: button_up_cb
            },
            axis={
                ps3.AXIS_JOY_PAD_RIGHT: axis_cb
            }
        )

    manager.start()
    time.sleep(10)

    manager.controllers()[0].disabled = True

    manager.stop()
    time.sleep(10)

    manager.start()
    time.sleep(10)

    manager.stop()
