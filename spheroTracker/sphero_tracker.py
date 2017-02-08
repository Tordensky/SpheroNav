import sphero
from sphero import SpheroError
import time
import tracker
import random


class SpheroTracker(object):
    """
    Example code of a tracker application skeleton
    """

    def __init__(self):
        self.object_tracker = tracker.ColorTracker()

        self._sphero_manager = sphero.SpheroManager()
        self._spheros = []

    def start_tracking(self):
        self._sphero_manager.set_sphero_found_cb(self.on_new_sphero)
        self._sphero_manager.start_auto_search()

        while True:
            r, g, b = (random.randrange(0, 255),
                       random.randrange(0, 255),
                       random.randrange(0, 255))
            for sphero_dev in self._spheros:
                try:
                    sphero_dev.set_rgb(r, g, b, True)

                except SpheroError as e:
                    print "Sphero Error: ", e
                    self._remove_sphero(sphero_dev)

            time.sleep(0.5)

    def on_new_sphero(self, device):
        """
        :param device: The found sphero device
        :type device: sphero.SpheroAPI
        """
        self._connect_new_sphero(device)

    def _connect_new_sphero(self, device):
        """
        Helper method to try to connect a given sphero.
        Removes sphero from sphero manager if connection fails
        :param device: sphero.SpheroAPI
        """
        try:
            print "Found ", device.bt_name, "tries to connect"
            device.connect()
            self._spheros.append(device)
            print "connected", device.bt_name
        except SpheroError as e:
            print e, device.bt_name
            self._remove_sphero(device)

    def _remove_sphero(self, device):
        """
        Helper method to remove sphero from sphero manager and self spheros
        :param device: The device to remove
        :type device: sphero.SpheroAPI
        """
        try:
            self._spheros.remove(device)
        except ValueError:
            pass
        self._sphero_manager.remove_sphero(device)

if __name__ == "__main__":
    sphero_tracker = SpheroTracker()
    sphero_tracker.start_tracking()