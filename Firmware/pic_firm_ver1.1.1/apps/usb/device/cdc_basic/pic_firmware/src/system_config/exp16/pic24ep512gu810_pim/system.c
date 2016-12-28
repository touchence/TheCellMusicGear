/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/

#include <p24EP256GU810.h>
#include <system.h>
//#include <system_config.h>
//#include <usb/usb.h>
#include <leds.h>
#include <buttons.h>
#include <adc.h>

#include <app_device_cdc_basic.h>

unsigned int check_2[100] = {0};
unsigned int number = 0;

/*********************************************************************
* Function: void SYSTEM_Initialize( SYSTEM_STATE state )
*
* Overview: Initializes the system.
*
* PreCondition: None
*
* Input:  SYSTEM_STATE - the state to initialize the system into
*
* Output: None
*
********************************************************************/
void SYSTEM_Initialize( SYSTEM_STATE state )
{
    switch(state)
    {
        case SYSTEM_STATE_USB_START:
            ANSELA = 0x0000;
            ANSELB = 0x0000;
            ANSELC = 0x0000;
            ANSELD = 0x0000;
            ANSELE = 0x0000;
            ANSELG = 0x0000;
            break;
        case SYSTEM_STATE_USB_SUSPEND:
            break;

        case SYSTEM_STATE_USB_RESUME:
            break;
    }
}

#if defined(USB_INTERRUPT)
void __attribute__((interrupt,auto_psv)) _USB1Interrupt()
{
    USBDeviceTasks();
}
#endif
/*
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void){

    writeBuffer[number] = U1RXREG;

    number++;

    if(number == 64){
        putUSBUSART(writeBuffer,number);
        number = 0;
    }

    IFS0bits.U1RXIF = 0;	// Clear the Recieve Interrupt Flag

}
*/
        
