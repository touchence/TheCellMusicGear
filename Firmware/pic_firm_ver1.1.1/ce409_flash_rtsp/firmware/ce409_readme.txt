
		Readme File for Code Example:
              ce409 - run time self programming (rtsp) code example
             ----------------------------------------

This file contains the following sections:
1. Code Example Description
2. Folder Contents
3. Suggested Development Resources
4. Reconfiguring the project for a different dsPIC33E device
5. Revision History


1. Code Example Description:
----------------------------
In this code example for dsp33ep512mu810, a page of Flash memory (1024 instructions or 8 rows of 128 instruction) is read first.
Then the page is erased fully. One row of the page is modified and written back to the flash.

Following RTSP Application Program Interface (APIs) are used to perform the operation.

Flash Memory is organised into ROWs of 128 instructions or 384 bytes
RTSP allows the user to erase a PAGE of memory which consists of EIGHT ROWs (1024 instructions or 3072 bytes) at a time.
RTSP allows the user to program a ROW (128 instructions or 384 bytes) at a time

;-------------------------------------
/*
 * FLASH PAGE READ
 *
 * Parameters Definition:
 * nvmAdru:	Selects the upper 8bits of the location to read in program flash memory
 * nvmAdr:  Selects the location to read in program flash memory
 *          It must be aligned to 1024 instruction boundary, LSB 10bits of address must be zero
 * pageBufPtr: Pointer to the data array in which read data will be stored

 
 * Return Value:
 * Function returns ERROREE (or -1), if it is not successful
 * Function return ZERO, if successful
*/

extern int flashPageRead(u16 nvmAdru, u16 nvmAdr, i16 *pageBufPtr);
;----------------------------------------

;----------------------------------------
/*
 * FLASH PAGE MODIFY
 *
 * Parameters Definition:
 * row:		Selects the row in the the Flash page that will be modified
 * rowBuf:  Selects the location to read in program flash memory
 * pageBufPtr: Pointer to the page data array, that will be modified
 
 * Return Value:
 * Function returns ERROREE (or -1), if it is not successful
 * Function return ZERO, if successful
*/

extern int flashPageModify(u16 row, u16 size, i16 *rowBuf, i16 *pageBufPtr);
;----------------------------------------

;----------------------------------------
/*
 * FLASH PAGE ERASE
 *
 * Parameters Definition:
 * nvmAdru:	Selects the upper 8bits of the location to program or erase in program flash memory
 * nvmAdr:  Selects the location to program or erase in program flash memory
*           It must be aligned to 1024 instruction boundary, LSB 10bits of address must be zero
 
 * Return Value:
 * Function returns ERROREE (or -1), if it is not successful
 * Function return ZERO, if successful
*/
extern int flashPageErase(u16 nvmAdru, u16 nvmAdr);
;----------------------------------------

;----------------------------------------
/*
 * FLASH PAGE WRITE
 *
 * Parameters Definition:
 * nvmAdru:	Selects the upper 8bits of the location to program or erase in program flash memory
 * nvmAdr:  Selects the location to program or erase in program flash memory
 *          It must be aligned to 1024 instruction boundary, LSB 10bits of address must be zero
 * pageBufPtr: Pointer to the data array that needs to be programmed 


 * Return Value:
 * Function returns ERROREE (or -1), if it is not successful
 * Function return ZERO, if successful
*/
extern int flashPageWrite(u16 nvmAdru, u16 nvmAdr, i16 *pageBufPtr);
;----------------------------------------


2. Folder Contents:
-------------------
a. firmware
        This folder contains all the C, Assembler source files and include files(*.c,
        *.s, *.h) and project specific files used in demonstrating the described example. 
b. system_config
		This folder contains the chipset specific configuration code. More specifically it inturn contains a folder called exp16/ 
		which holds configuration files.
c. exp16/
		This folder contains various folders like dspic33ep512mu810/dspic33ep256gp506 depending on the platform.Each platform folder contain,configuration 
		specific source files.

3. Suggested Development Resources:
-----------------------------------
        a. Explorer 16 Demo board with dsPIC33EP512MU810/dspic33ep256gp506 controller

4. Reconfiguring the project for a different dsPIC33E device:
-------------------------------------------------------------
The Project/Workspace can be easily reconfigured for dspic33ep512mu810/dspic33ep256gp506 device.
Please use the following general guidelines:
        a. Change device selection within MPLAB® IDE to dspic33ep512mu810/dspic33ep256gp506 device of
        your choice by using the following menu option:
        MPLAB X>>Configuration drop-down option>><Listed Device Configuration>

        b. Re-build the MPLAB® project using the menu option:
        MPLAB X>>Build Main Project

        c. Download the hex file into the device and run.


5. Revision History :
---------------------
        04/01/2006 - Initial Release of the Code Example
	07/01/2010 - Code Example updated for dsPIC33E
	3/6/2014  - Code Example updated for dspic33ep512mu810/dspic33ep256gp506