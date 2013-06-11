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

<b>NPI Server</b> can be built from /Projects/tools/LinuxHost/

A sample application which incorporates an <b>NPI Client</b> can be built from 
/Projects/tools/LinuxHost/applications/simple

Please see http://processors.wiki.ti.com/index.php/RF4CE,_simple_linux_target_application for 
more details.
