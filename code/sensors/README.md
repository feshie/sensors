Feshie Sensor Driver
====================

Authors
--------
Jonathon Hare | jonhare | jsh2@ecs.soton.ac.uk

Phil Basford  | pjb304 | pjb@ecs.soton.ac.uk

Summary
-------

This is the AVR driver code for the Feshie sensor chains. Multiple sensors are connected to an AVR, which then talks to the Zolertia Z1 via serial communication over RS485. This driver code implements the functionality required for the AVR to listen for commands from the Z1 and perform operations on (usually reading data from) the sensors.

Z1<->AVR Communication Protocol
-------------------------------
The Z1 is connected to one or more AVR sensor drivers via RS485. Each AVR connected to a Z1 has an ID (single byte) to identify it. This ID is burned into the EEPROM, together with a listing of the sensors attached to the AVR. The XXX software is used to program the AVR EEPROM with the required data. 

### Framing format ###

The serial communication protocol for sending commands to the AVR is designed to be both simple and compact and takes the following framing format:

	<AVR_ADDRESS:byte><OPCODE:byte>(<PAYLOAD>)<CRC16:uint16>

The `<AVR_ADDRESS>` is a single byte that indicates the address of the sensor driver device that the command is destined for. The `<OPCODE>` is a single byte representing the operation that the AVR is to perform. The `<PAYLOAD>` is only required for certain `<OPCODES>` and represents data needed to perform the action specified by the `<OPCODE>`. The `<CRC16>` is the CRC-16-IBM	cyclic redundancy check over the address, opcode and payload byte sequence.

The AVRs connected to the RS485 bus all listen for commands given by the `<OPCODE>`. Commands are ignored if the `<AVR_ADDRESS>` doesn't match the ID of the AVR in question. If the ID does match, then the AVR will perform an action corresponding to the `<OPCODE>` and return a response to the Z1. Response are sent using the same framing format as above, with a special response `<OPCODE>` of 0xFF. The form of the response payload is tied to the specific `<OPCODE>`.

The currently supported commands, together with the responses are detailed in the following table:

| Name        | `<OPCODE>` | `<PAYLOAD>`                    | Expected response payload           |
|-------------|------------|--------------------------------|-------------------------------------|
| Echo        | 0x00	   | A null terminated ASCII string | the `<PAYLOAD>`                     |
| ListSensors | 0x01       | none                           | protocol-buffer encoded data struct: `<LENGTH:uint16><DATA:byte[LENGTH]>` |
| GetData     | 0x02       | none                           | protocol-buffer encoded data struct: `<LENGTH:uint16><DATA:byte[LENGTH]>` |
| SetGain     | 0x03       | The new gain (single byte)     | 0x01 on success; 0x00 on failure    |
| _RESPONSE_  | 0xFF       | The response 					| N/A                      			  |

### CRC16 ###
The CRC16 variant used is the `crc16_update` [described here.](http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#ga95371c87f25b0a2497d9cba13190847f).

### Timing ###



AVR Sensor Connections
----------------------


Source Code Files
-----------------

