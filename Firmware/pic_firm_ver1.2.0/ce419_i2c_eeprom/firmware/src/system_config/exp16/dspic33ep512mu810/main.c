/*******************************************************************************
  ce419 main source file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Carries out the read and write operations on EEPROM using I2C.

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
#include <stdint.h>
#include "i2c_emem.h"


#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Macros for Configuration Fuse Registers:
// *****************************************************************************
// *****************************************************************************
/* Invoke macros to set up  device configuration fuse registers.The fuses will
   select the oscillator source, power-up timers, watch-dog timers etc. The
   macros are defined within the device header files. The configuration fuse
   registers reside in Flash memory.
 */
// DSPIC33EP512MU810 Configuration Bit Settings
// 'C' source line config statements
int FGS __attribute__((space(prog), address(0xF80004))) = 0xFFCF ;
//_FGS(
//    GWRP_OFF &           // General Segment Write-Protect bit (General Segment may be written)
//    GSS_OFF &            // General Segment Code-Protect bit (General Segment Code protect is disabled)
//    GSSK_OFF             // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)
//);
 int FOSCSEL __attribute__((space(prog), address(0xF80006))) = 0xFFF8 ;
//_FOSCSEL(
//    FNOSC_FRC &          // Initial Oscillator Source Selection bits (Internal Fast RC (FRC))
//    IESO_ON              // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)
//);
 int FOSC __attribute__((space(prog), address(0xF80008))) = 0xFF1B ;
//_FOSC(
//    POSCMD_NONE &        // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
//    OSCIOFNC_ON &        // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
//    IOL1WAY_OFF &        // Peripheral pin select configuration (Allow multiple reconfigurations)
//    FCKSM_CSECME         // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are enabled)
//);
 int FWDT __attribute__((space(prog), address(0xF8000A))) = 0xFF1F ;
//_FWDT(
//    WDTPOST_PS32768 &    // Watchdog Timer Postscaler bits (1:32,768)
//    WDTPRE_PR128 &       // Watchdog Timer Prescaler bit (1:128)
//    PLLKEN_OFF &         // PLL Lock Wait Enable bit (Clock switch will not wait for the PLL lock signal.)
//    WINDIS_ON &          // Watchdog Timer Window Enable bit (Watchdog Timer in Window mode)
//    FWDTEN_OFF           // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)
//);
 int FPOR __attribute__((space(prog), address(0xF8000C))) = 0xFFF7 ;
//_FPOR(
//    FPWRT_PWR128 &       // Power-on Reset Timer Value Select bits (128ms)
//    BOREN_OFF &          // Brown-out Reset (BOR) Detection Enable bit (BOR is disabled)
//    ALTI2C1_OFF &        // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)
//    ALTI2C2_OFF          // Alternate I2C pins for I2C2 (SDA2/SCK2 pins are selected as the I/O pins for I2C2)
//);
 int FICD __attribute__((space(prog), address(0xF8000E))) = 0xFFDA ;
//_FICD(
//    ICS_PGD2 &           // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)
//    RSTPRI_AF &          // Reset Target Vector Select bit (Device will obtain reset instruction from Aux flash)
//    JTAGEN_OFF           // JTAG Enable bit (JTAG is disabled)
//);
 int FAS __attribute__((space(prog), address(0xF80010))) = 0xFFCF ;
//_FAS(
//    AWRP_OFF &           // Auxiliary Segment Write-protect bit (Aux Flash may be written)
//    APL_OFF &            // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
//    APLK_OFF             // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)
//);

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
// Instantiate Drive and Data objects
I2CEMEM_DRV i2cmem = I2CSEMEM_DRV_DEFAULTS;
I2CEMEM_DATA    wData;
I2CEMEM_DATA    rData;

uint16_t        wBuff[10], rBuff[10];
uint16_t        enable;
extern uint16_t jDone;

#define CPU_CLOCK 7370000L // [Hz]
#define CPU_PLL 52
#define FCY ( CPU_CLOCK * CPU_PLL / 8)

#include <libpic30.h>

/******************************************************************************
 * Function:        int main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       Sets operating clock frequency and initializes I2C.
 *****************************************************************************/
int main( void )
{
    int i = 0;

        //PLLFBD = 58;                        /* M  = 60  */
    PLLFBD = 50; /* M  = 52.1  */
    CLKDIVbits.PLLPOST = 0; /* N1 = 2   */
    CLKDIVbits.PLLPRE = 0; /* N2 = 2   */
    OSCTUN = 0;

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to Primary
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching
    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) {
    };
    /*	Initiate Clock Switch to Primary
     *	Oscillator with PLL (NOSC= 0x3)*/

    // Configuring the auxiliary PLL, since the primary
    // oscillator provides the source clock to the auxiliary
    // PLL, the auxiliary oscillator is disabled. Note that
    // the AUX PLL is enabled. The input 8MHz clock is divided
    // by 2, multiplied by 24 and then divided by 2. Wait till
    // the AUX PLL locks.

    ACLKCON3bits.SELACLK = 0; // Select Fvco
    ACLKCON3bits.APLLPOST = 0x5; // Divided by 4
    
    ANSELD = 0x0000;        // for Amp wake up
    TRISD = 0x7f00; // D0-7:O(for_SS) D8-11:I D12-13:NC D14:I(SDI) D15:O(SDO)   // for AD CPU

    PORTDbits.RD4 = 1;
    __delay_us(500);         // 500us delay
    PORTDbits.RD4 = 0;
    __delay_us(500);         // 500us delay
    PORTDbits.RD4 = 1;
    __delay_us(500);         // 500us delay

    // Initialise I2C peripheral and Driver
    i2cmem.init( &i2cmem );

    // Initialise Data to be written to serial EEPROM
    for( i = 0; i < 10; i++ )
    {
        wBuff[i] = i + 54;
    }

    // Initialise I2C Data object for Write operation
    wData.buff = wBuff;
    wData.n = 1;                        //10;
    wData.addr = 0x00;
    wData.csel = 0x00;

    // Initialise I2C Data Object for Read operation
    rData.buff = rBuff;
    rData.n = 1;                        //10;
    rData.addr = 0x00;
    rData.csel = 0x00;

    initTmr1();

    T1CONbits.TON = 1; //Start Timer 1

    // Enable data write to I2C serial EEPROM
    enable = 1;

    while( 1 )
    {
        if( enable == 1 )
        {
            enable = 0;

            // Read Data
            i2cmem.oData = &rData;
            i2cmem.cmd = I2C_READ;      /* comment this line to omit the read phase */

            while( i2cmem.cmd != I2C_IDLE )
            {
                i2cmem.tick( &i2cmem );
            }
        }
    };
}

void initTmr1() {
    TMR1 = 0x0000;
//    PR1 = 48000;               // Trigger ADC1 every 1 msec // 47.9MHz
    PR1 = 4800;               // Trigger ADC1 every 100 usec // 47.9MHz

    IFS0bits.T1IF = 0;      // Clear Timer 1 interrupt
    IEC0bits.T1IE = 1;      // Enable Timer 1 interrupt
}

/*******************************************************************************
 End of File
*/
