Communication Protocol
=======================
An overview of the communication protocol utilized by the Launch Vehicle Controller.

##Summary
The Launch Vehicle Controller utilizes two types of packets, described within this document. Protocol implementation can be viewed in [payload_def.h](/onboard_controller/payload_def.h).

* **Outgoing Packet** - These are longer packets delivered from the Launch Vehicle Controller to the Ground Station Controller. These packets contain the latitude, longitude, altitude, and other boolean states associated with the launch vehicle controller.
* **Incoming Packet (Directive)** - These are single byte commands issued by the Ground Station Controller, directing the Launch Vehicle Controller to take a certain action.

##Outgoing Packet
Below is a sample of an outgoing packet, broken down into components:
```
FB 61 7A 60 00 									latitude:  424545
               FC 5D AB B0 00					longitude: 768725
			                  FD F1 00			altitude:  31
						               FE 10	gps_fix: true
```
Each packet is composed of a 1-byte marker followed by a certain number of bytes defined by the marker. That is, each marker-data pair consists of `<1 byte MARKER><N bytes of DATA>`. It is critically important to remember that the DATA component of each marker-data pair is in **little-endian** format. Failure to account for this when parsing packets may lead to catastrophic results. Below is a list of currently possible markers:

|Marker		|Name			|Size (bytes) 	|Description		  					|
|:---------:|---------------|:-------------:|---------------------------------------|
|`0xFB`		|`MARKER_LAT`	|4				|Latitude data (unsigned, decimal point stripped)|
|`0xFC`		|`MARKER_LON`	|4				|Longitude data	(unsigned, decimal point stripped)|
|`0xFD`		|`MARKER_ALT`	|2				|Latitude data (unsigned)				|
|`0xFE`		|`MARKER_FLAG`	|1				|Boolean bit array (more details below)	|

Note that the *Size* descriptor does not include the 1-byte marker. Both latitude and longitude are unsigned and converted to integer numbers by multiplication with powers of 10. Modifications must be made if the latitude or longitude change sign during the course of normal operation.

Using the marker system, each outgoing packet does not need to have a fixed size. Instead, the packet can self-describe the data it contains. It is up to the parser to handle interpreting markers and reading the correct number of bytes following each marker. However, in the current implementation, each packet has a fixed size of 15 bytes.

The `MARKER_FLAG` denotes the start of a 1-byte (8 bit) boolean bit array, which represents specific states as documented in the table below:

|Position 	|Name				|Size (bits)	|Description																	|
|:---------:|-------------------|:-------------:|-------------------------------------------------------------------------------|
|LSB		|`gps_fix`			|1				|Whether the GPS has a fix on the location.										|
|2 			|`payload_abort`	|1				|Whether to actively prevent the payload from deploying.						|
|3			|`main_launch`		|1				|Active when the rocket is in flight.											|
|4			|`landed`			|1				|Active when the rocket has landed (currently not implemented).					|
|5			|`test`				|1				|Test flag to ensure that transmission is functioning correctly.				|
|6 - MSB	|`padding`			|3				|Bits which are currently not used. Ensures that the bit array is 8-bit aligned.|

Flags can also be utilized as part of an autonomous feedback loop for directive acknowledgement. To ensure that a critical directive from the Ground Station Controller is received by the Launch Vehicle Controller, the Ground Station Controller can observe relevant changes in one or more of the boolean flags, resending the directive or deferring to alternate behavior if a change in state is not observed.

##Incoming Packet (Directive)
Current implementation restricts Incoming Packets to single byte commands, or directives:

|Directive	|Name						|Description			|
|:---------:|---------------------------|-----------------------|
|`0xAB` 	|`DIR_TEST`					|Toggle the `test` flag and invoke any test behavior.|
|`0xAC`		|`DIR_BEGIN_LAUNCH`			|Trigger the `main_launch` flag and invoke any launch behavior.|
|`0xAD`		|`DIR_PAYLOAD_ABORT`		|Trigger the `payload_abort` flag and actively prevent payload from deploying.|
|`0xAE`		|`DIR_PAYLOAD_ABORT_CANCEL` |De-trigger the `payload_abort` flag and do not actively prevent payload from deploying.|

##Notes:
* In an outgoing packet, the DATA component of each marker-data pair is in **little-endian** format.
* To reduce confusion, try to prevent assigning the same value to both markers and directives.

🚀