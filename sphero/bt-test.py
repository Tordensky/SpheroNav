## TODO - Implement Sphero manager to support async discovery- This is the example code to use


import bluetooth
import select

class MyDiscoverer(bluetooth.bluez.DeviceDiscoverer):

    def pre_inquiry(self):
        self.done = False

    def device_discovered(self, address, device_class, name):
        print "%s - %s" % (address, name)

    def inquiry_complete(self):
        self.done = True

d = MyDiscoverer()
d.find_devices(lookup_names=True)

readfiles = [d, ]

while True:
    rfds = select.select(readfiles, [], [])[0]

    if d in rfds:
        #print d
        d.process_event()

    if d.done: break