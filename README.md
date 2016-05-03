[Unified NPI](processors.wiki.ti.com/index.php/Unified_Network_Processor_Interface)
============

Texas Instruments driver for a Linux host.

This repository contains the UART, SPI and I2C drivers, and associated make files and header files
for interfacing with a Texas Instruments CC26xx or CC31xx that supports Unified NPI.

It also contains the RemoTI (RTI) API to be used by the application developer when communicating 
with the CC2620/50, and a sample application to illustrate the use of the RTI API. All these files are
delivered under the TI BSD license with the license terms included at the top of each file.

RemoTI is the implementation of the RF4CE specification by Texas Instruments.
The RemoTI software installer can be downloaded at http://www.ti.com/remoti. 
Note that this installer is delivered under different terms than the TI BSD license.
You must accept the RemoTI terms and conditions separately during the installation process. 


2016, May 3rd.

Architecture
------------
RemoTI-Linux is a multiprotocol driver. It supports all physical serial protocols supported by 
CC26xx and CC13xx devices, s.a. UART and SPI. The driver is divided in two parts. The first part,
the NPI Server, converts the physical serial protocol to a socket. The second part, generally
called NPI Client, constitutes converting the serial commands to C API. These C APIs can be 
added to your own application. Multiple NPI Clients can connect to a single NPI Server. This
allows multiple applications to use RF4CE data, s.a. Consumer Control, Keyboard and Mouse.

Please see processors.wiki.ti.com/index.php/Unified_Network_Processor_Interface for 
more details.

![Alt text](/Documents/RemoTI-Linux_Architecture-NPI_Server.png "NPI Server")

![Alt text](/Documents/RemoTI-Linux_Architecture-Simple.png "NPI Server")

Compilation
-----------
The makefile in <code>/Projects/tools/LinuxHost/</code> will build the NPI_Server for the following 
platforms:
- Current host platform using default gcc
- BeagleBone using arm-arago-linux-gnueabi-gcc

If you are missing any of the arm compilers the compilation will exit with error. However, the 
current platform is still built.

<code>$ cd Projects/tools/LinuxHost</code><br>
<code>$ make</code>

The makefile in <code>/Projects/tools/LinuxHost/application/zrc20</code> will build a simple console 
application to configure and control a RemoTI Network Processor (RNP).

<code>$ cd Projects/tools/LinuxHost/application/simple</code><br>
<code>$ make</code><br>


Execution
---------
If you are testing the driver for an RNP built on [CC2650 LaunchPad](http://www.ti.com/tool/LAUNCHXL-CC2650) then first lookup what device it has enumerated as. Typically this will be ttyACM0. You can have RNP on CC2650 by flashing the image <code>C:/ti/simplelink/remoti_2_00_00_13/examples/hex/cc2650em_rnp_rti_recipient_uart_super.</code>. See http://www.ti.com/remoti and http://www.ti.com/tool/packet-sniffer. To modify the device change the path inside the .cfg file. 
Then to execute the driver;

<code>$ sudo ./out/NPI_lnx_x86_server configuration/x86/RemoTI_RNP_ACM0.cfg</code>

If you have an NPI_Server running locally you can run the <code>simple</code> application like this;

<code>$ ./out/SIMPLE_lnx_x86_client -D 127.0.0.1:2530

If you want to connect and control an NPI_Server running on another platform you need to its IP address
and port. This address and port is found in the output from the NPI_Server at startup. Alternatively
you can find the port by reading the configuration file used by the NPI_Server, and find the IP address
by typing <code>$ ifconfig</code> on the platform running the NPI_Server.

