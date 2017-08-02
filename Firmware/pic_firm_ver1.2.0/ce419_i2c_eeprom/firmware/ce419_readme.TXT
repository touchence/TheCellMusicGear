                     Readme File for Code Example:
               CE419 - I2C Serial EEPROM Interface
               ---------------------------------------------------

This file contains the following sections:
1. Code Example Description
2. Folder Contents
3. Suggested Development Resources
4. Reconfiguring the project for a different dsPIC33E device
5. Revision History


1. Code Example Description:
----------------------------

In this code examples, 10bytes of data is written to I2C serial EEPROM and 
then read back using I2C peripheral. 

I2C EEPROM driver module takes two inputs viz., Command and Data Object
Driver supports two commands viz., Read and Write 

// EEPROM DRIVER Module
typedef struct { 
        unsigned int	cmd; 		// Command Input
	I2CEMEM_DATA	*oData;	       	// I2C Serial EEPROM Data Object
        void (*init)(void *);                   
        void (*tick)(void *); 
        }I2CEMEM_DRV; 

I2C Serial EEPROM data object contains EEPROM address location, data buffer, size
of the data buffer and chip select bits for device addressing
// Data Object
typedef struct { 
        unsigned int *buff;     	// Data Buffer 
        unsigned int n;        		// Size of the Data Buffer
        unsigned int addr;       	// EEPROM Address
        unsigned int csel;            	// Chip Select bits (A2,A1,A0 bits)
}I2CEMEM_DATA; 


* I2C Serial EEPROM read/write operation begins with transmitting control byte first. 
This control byte contains 8bits as shown below
 ------------------------------------
| 1 | 0 | 1 | 0 | A2 | A1 | A0 | R/W |
 ------------------------------------

1010 is the code used for I2C Serial EEPROM peripheral and A2,A1,A0 is used for chip select (csel).


* After the control byte, address of serial EEPROM is sent for read/write operation
Small memory I2C EEPROM will use 1byte address and large memory I2C EEPROM will need 2byte addressing. 
User must select either 1byte memory address or 2byte memory address using 
i2cEmem.h file. 

// EEPROM ADDRESS SIZE
#define ADDRWIDTH   TWO_BYTE    

2. Folder Contents:
-------------------
a. firmware
        This folder contains all the C, Assembler source files and include files(*.c,
        *.s, *.h) and project specific files used in demonstrating the described example. 
b. system_config
		This folder contains the chipset specific configuration code. More specifically it inturn contains a folder called exp16/ 
		which holds configuration files.
c. exp16/
		This folder contains various folders like dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506 depending on the platform.Each platform folder contain,configuration 
		specific source files.

3. Suggested Development Resources:
-----------------------------------
        a. Explorer 16 Demo board with dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506 controller
        b. 24AA512 EEPROM from Serial I2C EEPROM PICTail Pack AC243003

4. Reconfiguring the project for a different dsPIC33E device:
-------------------------------------------------------------
The Project/Workspace can be easily reconfigured for dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506 device.
Please use the following general guidelines:
        a. Change device selection within MPLAB® IDE to dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506 device of
        your choice by using the following menu option:
        MPLAB X>>Configuration drop-down option>><Listed Device Configuration>

        b. Re-build the MPLAB® project using the menu option:
        MPLAB X>>Build Main Project

        c. Download the hex file into the device and run.

5. Revision History :
---------------------
    07/09/2006 - Initial Release of the Code Example
    07/01/2010 - Code Example updated for dsPIC33E
    6/18/2012  - Added ACK Polling states and connections for I2C PICtail board- J.H.
    01/30/2014 - Code Example updated for dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506