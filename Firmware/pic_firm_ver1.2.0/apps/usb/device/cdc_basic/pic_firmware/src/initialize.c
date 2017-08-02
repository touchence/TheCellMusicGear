#include <usb/usb.h>

#include <p24Exxxx.h>
#include <user_define.h>
#include <libpic30.h>

void system_ini(void);

void clock_setting(void);
void port_setting(void);
void USB_setting(void);
void PWM_setting(void);

void initTmr1(void);
void initTmr2(void);
void initTmr3(void);

void initDma0(void);
void initDma1(void);
void initAdc1(void);

extern __eds__ uint8_t freq[(LED_UNIT * 24) + 1] __attribute__((eds,space(dma),aligned(4096)));
extern __eds__ int BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((eds,space(dma),aligned(256)));
extern __eds__ int BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((eds,space(dma),aligned(256)));

void system_ini(void){
    clock_setting();
    __delay_ms(50);
    USB_setting();
    __delay_ms(50);
    port_setting();
    __delay_ms(50);

    initTmr1();
    initTmr2();
    initTmr3();
    initDma0();
    initDma1();
    InitUART_1(12);

    __delay_ms(50);
    initAdc1(); // Initialize the A/D converter to convert Channel 5
    __delay_ms(50);

    PWM_setting();
}

void clock_setting(void){
    PLLFBD = 50;                /* M  = 52.1  */
    CLKDIVbits.PLLPOST = 0;     /* N1 = 2   */
    CLKDIVbits.PLLPRE = 0;      /* N2 = 2   */
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

    ACLKCON3bits.SELACLK = 0; // Select Fvco
    ACLKCON3bits.APLLPOST = 0x5; // Divided by 4
}

void port_setting(void){
    ANSELD = 0x0000;
    TRISD = 0x70e0;             // D0-3:O(LED1_1-4) D8-11:O(LED2_1-4) D4-7:O(for debug)
    PORTD = 0x00e0;             // for Amp wake up    // for bug action

    ANSELA = 0x00c0;
    ANSELB = 0xffff;
    ANSELC = 0x001e;
    ANSELE = 0x03ff;
    ANSELG = 0x0000;

    // Set I/O Direction (Output or Input)
    TRISA = 0xfff3;         // A0-1:NC A2-3:O(debug) A4:NC A5:I A6-7:I A9-10:NC A14-15 ; NC  			// set to all outputs
    TRISB = 0xffff;         // B0-15:I(for ADC)
    TRISC = 0xffff;         // C1-4:I C12:NC C13-14:I(for_ICSP) C15:I
    TRISE = 0xffff;         // E0-9:botton input                                          // for AD
    TRISG = 0x5ebf;         // G0:O(DEBUG) G1:O(DEBUG) G2(D+) G3(D-) G6:O(SCK2) G7:I(SDI2) G8:O(SDO2) G9:NC
    TRISF = 0xeef9;         // F0:I(U1RX) F1:O(U1TX) F2:0(LED PWM) F3:I(USB_ID) F4:O(DEBUG 1) F5:O(DEBUG 2)
                            // F8:O(FPGA_RESET) F12:0(ZEAL RTS) F13:I(ZEAL CTS)

    // Port Select
    RPOR9bits.RP101R = 0;       // DEFAULT PORT (tied to Default Pin / for I2C CLK pin)
    RPOR9bits.RP100R = 0;       // DEFAULT PORT (tied to Default Pin / for I2C DAT pin)

    RPINR18bits.U1RXR = 96;     // U1RX to RP96/RF0
    RPOR7bits.RP97R = 1;        // U1TX to RP97/RF1

    RPOR14bits.RP120R = 8;      // SDO2 to RP120/RG8 (for Chip)    // for bug action
    RPOR13bits.RP118R = 9;      // SCK2 to RP118/RG6 (for Chip)    // for bug action

    RPOR8bits.RP98R = 16;       // OC1 to RP64 LED1
}

// for USB
void USB_setting(void) {
    // for initializing of USB
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);
    USBDeviceInit();
    USBDeviceAttach();
    IPC21bits.USB1IP = 7; // highest interrupt
}

// for led setting
void PWM_setting(void) {
    // for initializing of PWM Unit
    OC1CON1bits.OCSIDL = 0;
    OC1CON1bits.OCTSEL = 7;
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON1bits.OCM = 6;
    OC1RS = 59;
    OC1R = 0; // duty = 0;
}

// for led measure
void initTmr1() {
    TMR1 = 0x0000;
    PR1 = 4800; // Trigger ADC1 every 100 usec // 47.9MHz

    IFS0bits.T1IF = 0; // Clear Timer 1 interrupt
    IEC0bits.T1IE = 1; // Enable Timer 1 interrupt
}

// for main loop
void initTmr2() {
    TMR2 = 0x0000;
    // PR2 = 48000; // Trigger ADC1 every 100 usec // 47.9MHz
    PR2 = 47400; // Trigger ADC1 every 100 usec // 47.9MHz

    IPC1bits.T2IP = 7;	// highest interrupt

    IFS0bits.T2IF = 0; // Clear Timer 2 interrupt
    IEC0bits.T2IE = 1; // Enable Timer 2 interrupt
}

// for adc trigger
void initTmr3() {
    TMR3 = 0x0000;
    PR3 = 480; // Trigger ADC1 every 10 usec // main // 47.95MHz

    IFS0bits.T3IF = 0; // Clear Timer 3 interrupt
    IEC0bits.T3IE = 1; // enable Timer 3 interrupt
    //  IEC0bits.T3IE = 0;      // disable Timer 3 interrupt
}

// for led signal
void initDma0(void){
//    DMA0CON = 0x6001; // One-Shot, Post-Increment, RAM-to-Peripheral

    DMA0CONbits.AMODE = 0;// Configure DMA for Register Indirect mode
    // with post-increment
    DMA0CONbits.MODE = 0;
    DMA0CONbits.SIZE = 1;   // byte
    // Configure DMA for Continuous mode
    DMA0CONbits.DIR = 1;// RAM-to-Peripheral data transfers
    DMA0PAD = (volatile unsigned int)&OC1R; // Point DMA to OC1RS
    DMA0CNT = LED_UNIT * 24;  // 8 LED x 24 bit = 192 (+ 1)DMA request

    DMA0REQ = 2;    // Select Compare Match OC1 as DMA request source
    // DMA0REQ = 8;    // Select Compare Match timer3 as DMA request source
    // DMA0REQ = 7;    // Select Compare Match timer2 as DMA request source

    DMA0STAL = __builtin_dmaoffset(freq);
    DMA0STAH = 0x0000;

    IFS0bits.DMA0IF = 0;    // Clear the DMA Interrupt Flag bit
    IEC0bits.DMA0IE = 1;    // Set the DMA Interrupt Enable bit

    IPC1bits.DMA0IP = 7;	// highest interrupt

    // IPC1bits.DMA0IP = 0;	// lowest interrupt

    DMA0CONbits.CHEN = 1;   // Enable DMASet up Timer2 for Output Compare PWM mode:
}

// for get adc value
void initDma1(void) {
    DMA1CONbits.AMODE = 2; // Configure DMA for Peripheral indirect mode
    DMA1CONbits.MODE = 0; // Configure DMA for Continuous Ping-Pong mode
    DMA1PAD = (int) &ADC1BUF0;
    // DMA0CNT = (SAMP_BUFF_SIZE * NUM_CHS2SCAN) - 1;       // main
    DMA1CNT = (SAMP_BUFF_SIZE * 32) - 1;
    DMA1REQ = 13; // Select ADC1 as DMA Request source

    DMA1STAL = __builtin_dmaoffset(&BufferA);
    DMA1STAH = __builtin_dmapage(&BufferA);

    DMA1STBL = __builtin_dmaoffset(&BufferB);
    DMA1STBH = __builtin_dmapage(&BufferB);

    // IPC3bits.DMA1IP = 7; // highest interrupt
    // IPC3bits.DMA1IP = 0;	// lowest interrupt

    IFS0bits.DMA1IF = 0; //Clear the DMA interrupt flag bit
    IEC0bits.DMA1IE = 1; //Set the DMA interrupt enable bit

    DMA1CONbits.CHEN = 1; // Enable DMA
}

// for communication with master unit
void InitUART_1(uint16_t baud) {

        U1MODEbits.UARTEN = 0;
        // 1 = UARTx is enabled; UARTx pins are controlled by UARTx as defined by the UEN<1:0> and UTXEN control bits
        // 0 = UARTx is disabled; UARTx pins are controlled by the corresponding PORTx, LATx and TRISx bits
        U1MODEbits.USIDL = 0;
        // 1 = Discontinues operation when the device enters Idle mode
        // 0 = Continues operation in Idle mode
        U1MODEbits.IREN = 0;
        // 1 = IrDA encoder and decoder are enabled
        // 0 = IrDA encoder and decoder are disabled
        U1MODEbits.RTSMD = 0;
        // 1 = UxRTS is in Simplex mode
        // 0 = UxRTS is in Flow Control mode
        U1MODEbits.UEN = 0;
        // 11 = UxTX, UxRX and BCLKx pins are enabled and used; UxCTS pin is controlled by port latches
        // 10 = UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
        // 01 = UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin is controlled by port latches
        // 00 = UxTX and UxRX pins are enabled and used; UxCTS, UxRTS and BCLKx pins are controlled by port latches
        U1MODEbits.WAKE = 0;
        // 1 = Wake-up is enabled
        // 0 = Wake-up is disabled
        U1MODEbits.LPBACK = 0;
        // 1 = Enables Loopback mode
        // 0 = Loopback mode is disabled

        U1MODEbits.ABAUD = 0;
        // 1 = Enables baud rate measurement on the next character, requires reception of a Sync field (0x55); cleared in hardware upon completion
        // 0 = Baud rate measurement is disabled or complete
        U1MODEbits.URXINV = 0;
        // 1 = UxRX Idle state is ?0?
        // 0 = UxRX Idle state is ?1?
        U1MODEbits.BRGH = 1;
        // 1 = BRG generates 4 clocks per bit period (4x baud clock, High-Speed mode)
        // 0 = BRG generates 16 clocks per bit period (16x baud clock, Standard Speed mode)
        U1MODEbits.PDSEL = 0;
        // 11 = 9-bit data, no parity
        // 10 = 8-bit data, odd parity
        // 01 = 8-bit data, even parity
        // 00 = 8-bit data, no parity
        U1MODEbits.STSEL = 0;
        // 1 = 2 Stop bits
        // 0 = 1 Stop bit

        U1STAbits.URXISEL = 0;
        // 11 = Reserved; do not use
        // 10 = Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty
        // 01 = Interrupt when the last character is shifted out of the Transmit Shift Register; all transmit operations are completed
        // 00 = Interrupt when a character is transferred to the Transmit Shift Register (this implies there is at least one character open in the transmit buffer)
        U1STAbits.UTXISEL0 = 0;
        U1STAbits.UTXISEL1 = 0;

        U1STAbits.UTXINV = 0;
        // If IREN = 0:
        // 1 = UxTX Idle state is ?0?
        // 0 = UxTX Idle state is ?1?
        // If IREN = 1:
        // 1 = IrDA encoded, UxTX Idle state is ?1?
        // 0 = IrDA encoded, UxTX Idle state is ?0?
        U1STAbits.UTXBRK = 0;
        // 1 = Sends Sync Break on next transmission ? Start bit, followed by twelve ?0? bits, followed by Stop bit; cleared by hardware upon completion
        // 0 = Sync Break transmission is disabled or completed
        U1STAbits.UTXEN = 1;
        // 1 = Transmit is enabled, UxTX pin is controlled by UARTx
        // 0 = Transmit is disabled, any pending transmission is aborted and the buffer is reset; UxTX pin controlled by port
        U1STAbits.URXISEL = 0;
        // 11 = Interrupt is set on UxRSR transfer making the receive buffer full (i.e., has 4 data characters)
        // 10 = Interrupt is set on UxRSR transfer making the receive buffer 3/4 full (i.e., has 3 data characters)
        // 0x = Interrupt is set when any character is received and transferred from the UxRSR to the receive buffer; receive buffer has one or more characters
        U1STAbits.ADDEN = 0;
        // 1 = Address Detect mode is enabled; if 9-bit mode is not selected, this control bit has no effect
        // 0 = Address Detect mode is disabled

	//  BaudRate = Fcy/(4*(U1BRG + 1))
    //  6M = (48M/52)
    //  9600 = (48M/5000)

    U1BRG = baud; // 48Mhz osc,  9600 Baud (9600 baud)

    // 48MHz osc
    //      U2BRG = 1249    9600 Baud (9600 baud)
    //	U2BRG = 12      923.07k Baud (921.6 baud)
    // 	U2BRG = 103     115.38k Baud (115.2 baud)


    //	IPC2bits.U1RXIP = 7;	// highest interrupt
    //	IPC3bits.U1TXIP = 7;	// highest interrupt
    //	IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
    //	IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupts
    //	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
    //	IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts
    IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 1; // Enable Recieve Interrupts

    U1MODEbits.UARTEN = 1; // And turn the peripheral on
    U1STAbits.UTXEN = 1;

}

// for adc
void initAdc1(void) {

    AD1CON1bits.FORM = 0; // Data Output Format: Signed Fraction (Q15 format)
    AD1CON1bits.SSRCG = 0;
    AD1CON1bits.SSRC = 7; // Sample Clock Source: Timer 3 starts conversion
    AD1CON1bits.ASAM = 0; // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0; // 10-bit ADC operation
    AD1CON2bits.CSCNA = 1; // Scan Input Selections for CH0+ during Sample A bit
    AD1CON2bits.CHPS = 0; // Converts CH0

    AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
    // AD1CON3bits.ADCS = 2; // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*16 = 0.4us (625Khz) // main
    // ADC Conversion Time for 10-bit Tc=12*Tab = 4.05us

    AD1CON3bits.ADCS = 2; // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/48M)*16 = 0.4us (625Khz)
    // ADC Conversion Time for 10-bit Tc=12*Tab = 4.05us

    AD1CON1bits.ADDMABM = 0; // DMA buffers are built in scatter/gather mode
//  AD1CON2bits.SMPI = (NUM_CHS2SCAN - 1); // 4 ADC Channel is scanned      // main
    AD1CON2bits.SMPI = (32 - 1); // 4 ADC Channel is scanned

    AD1CON4bits.DMABL = 0; // Each buffer contains 1 words
    AD1CON4bits.ADDMAEN = 1; // Conversion results stored in ADCxBUF0 register
    AD1CSSH = 0xffff; // Enable AN16 - AN31  A/D Input Scan
    AD1CSSL = 0xffff;// Enable AN0 - AN15  A/D Input Scan

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // disable A/D interrupt

    __delay_ms(5);
    AD1CON1bits.ADON = 1; // Turn on the A/D converter
    __delay_ms(5);
}
