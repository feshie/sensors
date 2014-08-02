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

The serial communication protocol for sending commands to the AVR is designed to be both simple and compact and takes the following form:

	<OPCODE:byte><AVR_ADDRESS:byte>(<PAYLOAD>)

The `<OPCODE>` is a single byte representing the operation that the AVR is to perform. The `<AVR_ADDRESS>` is a single byte that indicates the address of the sensor driver device that the command is destined for. The `<PAYLOAD>` is only required for certain `<OPCODES>` and represents data needed to perform the action specified by the `<OPCODE>`. 

The AVRs connected to the RS485 bus all listen for commands. Commands are ignored if the `<AVR_ADDRESS>` doesn't match the ID of the AVR in question. If the ID does match, then the AVR will perform an action corresponding to the `<OPCODE>` and return a response to the Z1. The form of the response is tied to the specific `<OPCODE>`.

The currently supported commands, together with the responses are detailed in the following table:

| Name    | `<OPCODE>` | `<PAYLOAD>`                    | Expected response                   |
|---------|------------|--------------------------------|-------------------------------------|
| Echo    | 0x00	   | A null terminated ASCII string | the `<PAYLOAD>`                     |
| GetData | 0x01       | none                           | protocol-buffer encoded data struct: `<LENGTH:uint16><DATA:byte[LENGTH]><CRC32:int32>` |
| SetGain | 0x02       | The new gain to set (8 bytes?) | ACK byte: 0x01                      |


AVR Sensor Connections
----------------------


Source Code Files
-----------------

