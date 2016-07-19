Creating udev Rule

To retrieve the attributes of the microcontroller to create the proper udev rule for it you first will need to monitor udev events to dtermine the serial port name for the device. 

Excute the command: udevadm monitor

Then plug in the device and annotate what /dev/ name it was assigned.

Then excute the command: udevadm info --attribute-walk --name=/dev/?????

and replace the question marks with the name it was assigned by the system.

Retrive the follwing attribute values, for use in the udev rule.

ATTRS{idVendor}
ATTRS{idProduct}
ATTRS{serial}