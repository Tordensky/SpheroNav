import bluetooth
from threading import Thread
import threading
import time
from sphero import SpheroAPI


class SpheroManager:
    """
    A class for handling multiple Spheros
    :version 0.2
    """

    BT_AUTO_SEARCH_INTERVAL_SEC = 5
    BT_NAME_LOOKUP_TIMEOUT_SEC = 10
    BT_NAME_LOOKUP_NUM_RETRIES = 10

    SPHERO_BASE_NAME = "Sphero-"

    def __init__(self):
        self._name_cache = {"68:86:E7:02:3A:AE": "Sphero-RWO",
                            "68:86:E7:03:22:95": "Sphero-ORB",
                            "68:86:E7:03:24:54": "Sphero-YGY"}

        self._spheros = {}

        self._sphero_lock = threading.RLock()
        self._run_auto_search = True
        self._search_thread = None

        self._sphero_found_cb = None

    def get_device_by_name(self, name):
        """
        Gets a device by its name
        :param name: The name of the device
        :type name: str
        :return: The device instance
        :rtype: SpheroAPI or None
        """
        for key in self._spheros:
            sphero = self._spheros[key]
            if sphero.bt_name == name:
                return sphero
        return None

    def get_device_by_addr(self, addr):
        """
        Gets a device by its address
        :param addr: The addr of the device
        :type addr: str
        :return: The device instance
        :rtype: SpheroAPI or None
        """
        for key in self._spheros:
            sphero = self._spheros[key]
            if sphero.bt_addr == addr:
                return sphero
        return None

    def get_all_available_devices(self):
        """
        Returns a list of all Spheros that are registered as nearby and are not in use.
        :return: List of registered nearby devices
        :rtype: list
        """
        with self._sphero_lock:
            return [sphero for sphero in self._spheros.values() if not sphero.in_use]

    def get_connected_spheros(self):
        """
        Returns a list of the spheros that are available and connected.
        :return: List of connected devices
        :rtype: list
        """
        return [sphero for sphero in self.get_all_available_devices() if sphero.connected()]

    def start_auto_search(self):
        """
        Starts a thread that runs a auto search for nearby spheros

        """
        print "Starts auto search"
        self._run_auto_search = True
        if self._search_thread is None:
            self._search_thread = Thread(target=self._auto_search_loop, name="BtManagerDiscoveryThread")
            self._search_thread.daemon = True
            self._search_thread.start()

    def stop_auto_search(self):
        """
        Stops auto search if auto search is activated
        :return:
        """
        if self._search_thread is not None:
            self._run_auto_search = False

    def _auto_search_loop(self):
        """
        Helper method that runs the asynchronous automatic search loop
        """
        while self._run_auto_search:
            self.search()
            time.sleep(SpheroManager.BT_AUTO_SEARCH_INTERVAL_SEC)

        self._search_thread = None

    def search(self):
        """
        Starts a search for nearby spheros. When nearby spheros is found
        the pre set found_nearby_sphero_cb is triggered
        :return: Returns true if some device was found
        """
        found_devices = False
        for bd_addr in self._find_nearby_bt_devices():
            device_name = self.lookup_device_name(bd_addr)

            if self._is_sphero(device_name):
                found_devices = True
                self.add_sphero(bd_addr, device_name)

        return found_devices

    def get_available_device(self):
        """
        Returns an available sphero device if there are any devices nearby

        :return: An available device or None if no device is available or nearby
        :rtype: SpheroAPI or None
        """
        for _ in xrange(2):
            for key in self._spheros:
                sphero = self._spheros[key]
                if not sphero.in_use:
                    sphero.claim()
                    return sphero
            self.search()
        return None

    def add_sphero(self, bt_addr, bt_name):
        """
        Creates a new spheroAPI instance and adds this to the collection of spheros
        :param bt_addr: The Sphero bt_addr
        :type bt_addr: str
        :param bt_name: The Sphero device name
        :type bt_name: str
        """
        if bt_name not in self._spheros:
            new_sphero = SpheroAPI(bt_name, bt_addr)
            self._spheros[bt_name] = new_sphero
            self._notify_sphero_found(new_sphero)

    def remove_sphero(self, sphero):
        """
        Removes the given sphero device from connected and nearby devices and disconnects it.
        :param sphero: The sphero object that should be removed
        """
        with self._sphero_lock:
            self._spheros.pop(sphero.bt_name)
        sphero.disconnect()
        sphero.release()

    @staticmethod
    def _find_nearby_bt_devices():
        """
        Helper method that finds nearby bluetooth devices
        :return: A list of tuples of nearby device (bt_addr, bt_name)
        :rtype: list
        """
        nearby_devices = []
        for _ in xrange(5):
            try:
                nearby_devices = bluetooth.discover_devices()
            except bluetooth.BluetoothError as e:
                print "Error when searching for nearby devices:", e
            return nearby_devices

    def flush_name_cache(self):
        """
        Flush the bt device name cache
        """
        self._name_cache = {}

    def lookup_device_name(self, bd_addr):
        """
        Method for looking up bt device names. Implements a cache so previously looked up names
        are cached to minimize lookup time.
        :param bd_addr: BlueTooth address of the device
        :type bd_addr: str
        :return: The name of the device
        :rtype: str or None
        """
        if bd_addr in self._name_cache.iterkeys():
            return self._name_cache[bd_addr]

        sleep = 0.1
        for _ in xrange(SpheroManager.BT_NAME_LOOKUP_NUM_RETRIES):
            device_name = bluetooth.lookup_name(bd_addr, timeout=SpheroManager.BT_NAME_LOOKUP_TIMEOUT_SEC)
            if device_name is not None and len(device_name):
                self._name_cache[bd_addr] = device_name
                return device_name
            time.sleep(sleep)
        return None

    @staticmethod
    def _is_sphero(device_name):
        """
        Helper method that checks if the given name matches the one of a sphero.
        :param device_name: The name of the device
        :return: True if device name matches the sphero name pattern
        :rtype: bool
        """
        return device_name is not None and SpheroManager.SPHERO_BASE_NAME in device_name

    def set_sphero_found_cb(self, cb):
        """
        Allows for the uses of the class to set a callback when a new sphero is detected from the search method
        :param cb: the callback method that should be called when a new sphero is detected
        """
        self._sphero_found_cb = cb

    def _notify_sphero_found(self, new_sphero):
        """
        Helper method that triggers the cb set to be triggered when a new Sphero is discovered
        :param new_sphero: The instance of the Sphero found
        :type new_sphero: SpheroAPI
        """
        if self._sphero_found_cb is not None:
            self._sphero_found_cb(new_sphero)


if __name__ == "__main__":
    ## FOR TESTING
    def callback(sphero):
        print "CALLBACK", sphero.bt_name
        sphero.connect()
        print "ALL:", sm.get_all_available_devices()
        print "CON:", sm.get_connected_spheros()

    devices = []
    sm = SpheroManager()
    while True:
        dev = sm.get_available_device()
        if dev:
            print dev.bt_name
            devices.append(dev)
            if len(devices) >= 3:
                for dev in devices:
                    dev.release()
        else:
            break


#    sm.set_sphero_found_cb(callback)
#    sm.start_auto_search()