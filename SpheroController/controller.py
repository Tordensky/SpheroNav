import time
from controllable import ControllableSphero
import ps3
import sphero
import tracker


class SpheroPS3Controls(object):
    """
    Demonstration application of the Sphero NAV library
    """

    def __init__(self):
        super(SpheroPS3Controls, self).__init__()
        self._ps3_manager = ps3.PS3manager()
        self._sphero_manager = sphero.SpheroManager()

        self._tracker = tracker.ColorTracker()

        self._controllable_devices = []

        self._init_sphero_manager()

    def _init_sphero_manager(self):
        self._sphero_manager.set_sphero_found_cb(self.on_new_sphero)

    def run(self):
        self._sphero_manager.start_auto_search()
        self._ps3_manager.start()
        while True:
            traceable_objects = []
            for controllable in self._controllable_devices:
                traceable_objects.append(controllable)

            if len(traceable_objects) > 1:
                if traceable_objects[0].pos:
                    traceable_objects[1].dot_pos = traceable_objects[0].pos

            self._tracker.track_objects(traceable_objects)
            time.sleep(1.0 / 25.0)

    @staticmethod
    def set_tracking_filter(controllable_sphero, device):
        if device.bt_name == "Sphero-YGY":
            controllable_sphero.filter = tracker.FilterSpheroBlueCover()
            print "SAME SPHERO"

        elif device.bt_name == "Sphero-ORB":
            controllable_sphero.filter = tracker.FilterGlow()

        elif device.bt_name == "Sphero-RWO":
            controllable_sphero.filter = tracker.FilterSpheroYellowCover()

        else:
            print device.bt_name

    def on_new_sphero(self, device):
        """
        Callback when new spheros are found
        :param device:
        :type device: sphero.SpheroAPI
        """
        print "NEW Sphero: ", device.bt_name

        if device.connect():
            controllable_sphero = ControllableSphero(device)
            controllable_sphero.set_sphero_disconnected_cb(self.clean_up_sphero_dev)

            ps3_ctrl = self._ps3_manager.get_available_controller()

            if ps3_ctrl:
                controllable_sphero.set_ps3_controller(ps3_ctrl)

            else:
                print "No free PS3 controller available"

            self.set_tracking_filter(controllable_sphero, device)

            self._controllable_devices.append(controllable_sphero)
            print "Controls successfully setup"
            return

        self.clean_up_sphero_dev(device)

    def clean_up_sphero_dev(self, device):
        #device.disconnect()
        try:
            print device, self._controllable_devices
            for controllable in self._controllable_devices:
                if controllable.device == device:
                    self._controllable_devices.remove(controllable)
                    break

            else:
                print "NOT FOUND"
        except ValueError:
            print "could not remove sphero"
            pass
        self._sphero_manager.remove_sphero(device)


if __name__ == "__main__":
    sphero_ps3 = SpheroPS3Controls()
    sphero_ps3.run()