/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file is used to call the flash API

  Description:
    The main.c includes the header files that have the flash API declarations
    and is used to call the RTSP flash APIs for Erase, Read and Write operations.
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
#include <xc.h>
#include <stdint.h>
#include "rtspapi.h"
#include "testdata.h"
#include <p24Exxxx.h>

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

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
// pageMirrorBuff is a 128 bytes array, used to temorarily modify the data after which it will
// written on flash
int16_t pageMirrorBuff[128 * 8];

// *****************************************************************************
/* RTSP variables
*/
uint16_t    nvmAdr,                 //Selects the location to read in program flash memory
nvmAdru,            //Selects the upper 8bits of the location to read in program flash memory
nvmAdrPageAligned,  //variable used to access the alignned addr
nvmRow,             //Selects the row in the the Flash page that will be modified
nvmSize;            //holds the memory size

// *****************************************************************************
/* Data to be written to Flash
*/
int16_t     myRowDataInRam[128] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127 };

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
 * Overview:        main function
 ***** main************************************************************************/
int main( void )
{
    int16_t i;

    int16_t temp;

    // Configure the device PLL to obtain 60 MIPS operation. The crystal
    // frequency is 8MHz. Divide 8MHz by 2, multiply by 60 and divide by
    // 2. This results in Fosc of 120MHz. The CPU clock frequency is
    // Fcy = Fosc/2 = 60MHz. Wait for the Primary PLL to lock and then
    // configure the auxilliary PLL to provide 48MHz needed for USB
    // Operation.

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

    nvmAdru = __builtin_tblpage( &myRowData8InFlash[0] );
    nvmAdr = __builtin_tbloffset( &myRowData8InFlash[0] );
    nvmAdrPageAligned = nvmAdr & 0xF800;        // Get the Flash Page Aligned address
    nvmRow = ( (nvmAdr >> 7) & 7 );             // Row in the page
    nvmSize = 64;

    // Read the page and place the data into pageMirrorBuf array
    temp = FlashPageRead( nvmAdru, nvmAdrPageAligned, pageMirrorBuff );

    // Modify the pageMirrorBuf array
    temp = FlashPageModify( nvmRow, nvmSize, myRowDataInRam, pageMirrorBuff );

    // Erase the page in Flash
    temp = FlashPageErase( nvmAdru, nvmAdrPageAligned );

    // Program the page with modified data
    temp = FlashPageWrite( 0, nvmAdrPageAligned, pageMirrorBuff );

    //temp = FlashPageWrite(0,0x0A00,pageMirrorBuff);
    // User can add code here to verify that flash is programmed correctly */
    // Clear Page Mirror Buffer
    for( i = 0; i < (128 * 8); i++ )
    {
        pageMirrorBuff[i] = 0;
    }

    //Read the page and place the data into pageMirrorBuf array
    temp = FlashPageRead( nvmAdru, nvmAdrPageAligned, pageMirrorBuff );

    while( 1 );
}

/*****************************************************************************
* Function:         void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the Watch Dog Timer on interrupt
 */
void __attribute__ ( (interrupt, no_auto_psv) ) _DefaultInterrupt( void )
{
    while( 1 )
    {
        ClrWdt();
    }
}

/*******************************************************************************
 End of File
*/
