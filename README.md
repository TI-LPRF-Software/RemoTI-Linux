RemoTI-Linux
============

Texas Instruments RF4CE application for a Linux host using a CC253x RNP

This repository contains the UART, SPI and I2C drivers, and associated make files and header files
for interfacing with a Texas Instruments CC253x RF4CE device configured in RemoTI Network Processor
(RNP) mode. 
It also contains the RemoTI (RTI) API to be used by the application developer when communicating 
with the CC253x, and a sample application to illustrate the use of the RTI API. All these files are
delivered under the TI BSD license with the license terms included at the top of each file.

RemoTI is the implementation of the RF4CE specification by Texas Instruments.
The RemoTI software installer can be downloaded at http://www.ti.com/remoti. 
Note that this installer is delivered under different terms than the TI BSD license.
You must accept the RemoTI terms and conditions separately during the installation process. 


2013, June 11th.

Architecture
------------
RemoTI-Linux is a multiprotocol driver. It supports all physical serial protocols supported by 
CC253x devices, s.a. UART, I2C, SPI and USB. The driver is divided in two parts. The first part,
the NPI Server, converts the physical serial protocol to a socket. The second part, generally
called NPI Client, constitutes converting the serial commands to C API. These C APIs can be 
added to your own application. Multiple NPI Clients can connect to a single NPI Server. This
allows multiple applications to use RF4CE data, s.a. Consumer Control, Keyboard and Mouse.

Please see http://processors.wiki.ti.com/index.php/RF4CE,_simple_linux_target_application for 
more details.

Compilation
-----------
The makefile in <code>/Projects/tools/LinuxHost/</code> will build the NPI_Server for the following 
platforms:
- Current host platform using default gcc
- BeagleBoard xM using arm-none-linux-gnueabi-gcc
- BeagleBone using arm-arago-linux-gnueabi-gcc

If you are missing any of the arm compilers the compilation will exit with error. However, the 
current platform is still built.

<code>$ cd Projects/tools/LinuxHost</code><br>
<code>$ make</code>

The makefile in <code>/Projects/tools/LinuxHost/application/Simple</code> will build a simple console 
application to configure and control a RemoTI Network Processor (RNP).

<code>$ cd Projects/tools/LinuxHost/application/simple</code><br>
<code>$ make</code><br>


Execution
---------
If you are testing the driver for an RNP built on CC2531 then first lookup what device it has 
enumerated as. Typically this will be ttyACM0. You can have RNP on CC2531 by flashing the image 
<code>C:\Texas Instruments\RemoTI-CC253xDK-1.3.1\bin\RNP-CC2531F256.hex</code>. See http://www.ti.com/remoti
and http://www.ti.com/tool/packet-sniffer. To modify the device change the path inside the .cfg file. 
Then to execute the driver;

<code>$ ./out/NPI_Server RemoTI_ACM0.cfg</code>

If you have an NPI_Server running locally you can run the <code>simple</code> application like this;

<code>$ ./out/SIMPLE_lnx_x86_client -D 127.0.0.1:2530

If you want to connect and control an NPI_Server running on another platform you need to its IP address
and port. This address and port is found in the output from the NPI_Server at startup. Alternatively
you can find the port by reading the configuration file used by the NPI_Server, and find the IP address
by typing <code>$ ifconfig</code> on the platform running the NPI_Server.

