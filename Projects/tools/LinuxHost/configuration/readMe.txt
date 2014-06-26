This folder contains files specific to different platforms

x86, BeagleBone Black and BeagleBoard

Each platform has its own configuration files used as input
to the NPI_Server driver.

x86 typically only support USB as transport layer. An RNP on 
CC2531 will enumerate as either an ttyACM, ttyCDC or ttyUSB 
serial-over-USB device. Hence, the x86 folder only contains
configuration files for ttyACM and ttyUSB.

BeagleBone Black support SPI, UART, I2C in addition to
Serial-over-USB. There are configuration files for each
serial interface. Two configuration files exist for UART on
different pins. Two configuration files exist for each of
SPI and I2C. The difference is the addition of enabling
debug interface access through GPIOs.
The BeagleBone Black version supported uses Kernel 3.8
with DeviceTree. Overlays exist in the folder devicetree
for each of the configurations. In addition, if the system
use upstart there is a upstart script in the upstart folder.
If upstart is not used one must manually execute the 
commands the upstart configuration file contains. This includes
enabling device tree overlays, as well as exporting GPIOs.
To learn more: 
	upstart.ubuntu.com/wiki
	derekmolloy.ie/beaglebone

BeagleBoard is no longer a default supported platform. One
must manually export the correct pins to use the different
serial interfaces. Otherwise the same as for BeagleBone Black.


