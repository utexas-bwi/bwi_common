#!/bin/sh

echo copying $(rospack find bwi_led)/udev to /etc/udev/rules.d
/usr/bin/sudo /bin/cp -r $(rospack find bwi_led)/udev/* /etc/udev/rules.d
echo reboot for changes to take effect