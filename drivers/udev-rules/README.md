# UDEV Rules

To allow us to communicate with our USB devices attached to the Odroid easily, we have set up some UDEV rules to make sure the ports are interchanged on startup.  

Start by running the following command to create a new rules file:

``` bash
sudo nano /etc/udev/rules.d/99-sensor.rules
```

Then paste in the following rules (Ctrl+Shift+V):

``` bash
SUBSYSTEMS=="tty", KERNEL=="ttyS1" ACTION=="add", MODE="0666", GROUP="dialout", SYMLINK+="sensor/gps"

SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8057", MODE="0666", GROUP="dialout", SYMLINK+="sensor/arduino"

SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE="0666", GROUP="dialout", SYMLINK+="sensor/lidar"

SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", SYMLINK+="sensor/pico"
```

Alternatively, you can copy and paste this [99-sensor.rules](./99-sensor.rules) into `/etc/udev/rules.d/`

Save and exit the file. Then run the following:
`sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger`

You will need to replug all your USB ports or reboot the Odroid for the Udev rules to take effect.

