/*******************************************************************************
  I2C with EEPROM source file

  Company:
    Microchip Technology Inc.

  File Name:
    i2c_emem.c

  Summary:
    Handles the operation of I2C with EEPROM.

  Description:
    This source file contains a state machine that enables the I2C module to
    communicate with an external EEPROM module. The state machine also has the
    ACK polling feature that is necessary when using an external EEPROM.
*******************************************************************************/
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
//#include <xc.h>
#include <p24Exxxx.h>
#include "i2c_emem.h"

#define CPU_CLOCK 7370000L // [Hz]
#define CPU_PLL 52
#define FCY ( CPU_CLOCK * CPU_PLL / 8)

#include <libpic30.h>

extern I2CEMEM_DRV i2cmem;

int16_t  state = 0;
int16_t  counter = 0;
uint8_t  data_buf = 0;
int16_t  error_counter = 0;
int16_t  error_state = 0;
int16_t  pre_state = 0;


void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    static int temp = 0;
    static int temp_2 = 0;
    static int temp_3 = 0;

    if(temp_3 == 99){   // 10 ms ?
        temp_3 = 0;
        temp_2 = 0;
        temp = 14;
    } else {
        temp_3++;
    }

    if (temp == 14) { // 1.5 ms ?
        if(temp_2 < 4){
            // Start Condition
            I2C2CONbits.SEN = 1;
            state = 2;
            temp = 0;
            temp_2++;
        }
    } else {
        temp++;
    }

    IFS0bits.T1IF = 0; // Clear Timer 1 interrupt
}

/******************************************************************************
 * Function:  void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This serves the I2C Master Interrupt Service Routine.
 *****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _MI2C2Interrupt(void) {
    if (pre_state == state) {
        if (error_counter >= 2000) {
            I2C1CONbits.PEN = 1; // ????
            error_counter = 0;
            state = 0;

            PORTDbits.RD4 = 1;
            __delay_us(500); // 500us delay
            PORTDbits.RD4 = 0;
            __delay_us(500); // 500us delay
            PORTDbits.RD4 = 1;
            __delay_us(500); // 500us delay
        } else {
            __delay_us(10); // 10us
            error_counter++;
        }
    } else {
        pre_state = state;
        error_counter = 0;
    }

    switch (state) {
        case 2:
            // Start Byte with device select id
            state = 3;
            I2C2TRN = (0x0010 << 1) | 0x0001;
            break;
        case 3:
            // Send address byte 1, if ack is received. Else Retry
            if (I2C2STATbits.ACKSTAT == 1) {
                I2C1CONbits.PEN = 1; // ????
                state = 4;
            } else {
                // Receive Enable
                I2C2CONbits.RCEN = 1;
                state = 12;
            }
            break;
        case 4:
            state = 0;
            break;
        case 11:
            // Receive Enable
            I2C2CONbits.RCEN = 1;
            state = 12;
            break;
        case 12:
            // Receive data
            error_counter = 0;

            if (I2C1STATbits.ACKSTAT == 1) {
                I2C1CONbits.PEN = 1; // error,restart
                state = 4;
            } else {
                data_buf = I2C2RCV;
            }

            counter++;
            if (counter < 9) {
                state = 11;
                I2C2CONbits.ACKDT = 0; // ACK
                I2C2CONbits.ACKEN = 1;
            } else {
                counter = 0;
                state = 30;
                I2C2CONbits.ACKDT = 1; // NACK
                I2C2CONbits.ACKEN = 1;
            }
            break;
        case 30:
            state = 31;
            I2C2CONbits.PEN = 1;
            break;
        case 31:
            break;
        default:
            break;
    }

    pre_state = state;

    IFS3bits.MI2C2IF = 0; //Clear the DMA0 Interrupt Flag;
}

/******************************************************************************
 * Function:  void __attribute__((interrupt, no_auto_psv)) _SI2C1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This serves the I2C Slave interrupt.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _SI2C2Interrupt( void )
{
    IFS3bits.SI2C2IF = 0;   //Clear the DMA0 Interrupt Flag
}

/******************************************************************************
 * Function:        void I2CEMEMinit(I2CEMEM_DRV *i2cMem)
 *
 * PreCondition:    None
 *
 * Input:           *i2cMem
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        I2C initialization/configuration function.
 *                  This function is used to configure the I2C module as the
 *                  master and use 8-bit mode for communication
 *                  with the serial EEPROM.
 *****************************************************************************/
void I2CEMEMinit( I2CEMEM_DRV *i2cMem )
{
    i2cMem->cmd = 0;
    i2cMem->oData = 0;

    ODCFbits.ODCF4 = 0;
    ODCFbits.ODCF5 = 0;

    I2C2CONbits.A10M = 0;
    I2C2CONbits.SCLREL = 1;
    I2C2BRG = 105;

    I2C2ADD = 0;
    I2C2MSK = 0;

    I2C2CONbits.I2CEN = 1;
    IEC3bits.MI2C2IE = 1;
    IFS3bits.MI2C2IF = 0;
}

/******************************************************************************
 * Function:        void I2CEMEMdrv(I2CEMEM_DRV *i2cMem)
 *
 * PreCondition:    None
 *
 * Input:           *i2cMem
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        I2C communication state machine.
 *                  This function is a state machine based on which different
 *                  set of actions is performed by the I2C master while
 *                  reading/writing from/to the slave device which is in this
 *                  case the serial EEPROM
 *****************************************************************************/
void I2CEMEMdrv( I2CEMEM_DRV *i2cMem )
{

}
