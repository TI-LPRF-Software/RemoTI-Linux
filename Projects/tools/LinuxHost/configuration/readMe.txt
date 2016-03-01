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

Overlays exist in the folder devicetree for each of the configurations.

If the system uses systemd there are two scripts for systemd. The following
steps explain how to enable the startup scripts
1. Copy the NPI Server executable to /etc/init.d/npi_server, make sure
root has execution rights.
2. Copy systemd/npi_server.conf    to /etc/init.d/npi_server.conf
3. Copy systemd/npi_server.start   to /etc/init.d/npi_server.start
4. Copy systemd/npi_config.service to /lib/systemd/system/npi_config.service
5. Copy systemd/npi_server.service to /lib/systemd/system/npi_server.service
6. Enable both services
	root@beaglebone:~$ systemctl enable npi_config.service
	root@beaglebone:~$ systemctl enable npi_server.service
7. Have changes take effect via a reboot
	root@beaglebone:~$ reboot

If the system uses Upstart there is an Upstart script in the upstart folder.
This includes enabling device tree overlays, as well as exporting GPIOs.
To learn more: 
	upstart.ubuntu.com/wiki
	derekmolloy.ie/beaglebone

BeagleBoard is no longer a default supported platform. One
must manually export the correct pins to use the different
serial interfaces. Otherwise the same as for BeagleBone Black.


