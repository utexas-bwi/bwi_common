### Configuring udev Rules

Retrieve the attributes of the microcontroller to create the proper udev rule for the specific device. 

* You will need to monitor udev events to determine the serial port name for the device. Execute the command: 

```
udevadm monitor
```

* Plug in the device and annotate what /dev/ name it was assigned.

* Execute the command below and replace the question marks with the name given to your device.

```
udevadm info --attribute-walk --name=/dev/?????
```

* Retrive the follwing attribute values and replace the values in 99-metromini.rules.

	* ATTRS{idVendor}
	* ATTRS{idProduct}
	* ATTRS{serial}
