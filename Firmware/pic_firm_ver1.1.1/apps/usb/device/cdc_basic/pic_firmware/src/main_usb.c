/** INCLUDES *******************************************************/
#include <usb/usb.h>
#include <usb/usb_device.h>
#include <usb/usb_device_midi.h>

#include <user_value.h>
#include <user_define.h>
#include <MIDI_Scaler.h>
#include "CMG_LIB.h"

USB_HANDLE USBTxHandle = 0;
USB_HANDLE USBRxHandle = 0;

unsigned char ReceivedDataBuffer[64] RX_BUFFER_ADDRESS_TAG;
unsigned char ToSendDataBuffer[64] TX_BUFFER_ADDRESS_TAG;
USB_AUDIO_MIDI_EVENT_PACKET midiData MIDI_EVENT_ADDRESS_TAG;

USB_AUDIO_MIDI_EVENT_PACKET midiData_b[1024];
USB_AUDIO_MIDI_EVENT_PACKET midiData_clk[1024];

__eds__ uint8_t freq[(LED_UNIT * 24) + 1] __attribute__((eds,space(dma),aligned(4096)));
__eds__ int BufferA[MAX_CHNUM+1] __attribute__((eds,space(dma),aligned(256)));
__eds__ int BufferB[MAX_CHNUM+1] __attribute__((eds,space(dma),aligned(256)));

// Number of locations for ADC buffer = 14 (AN0 to AN13) x 8 = 112 words
// Align the buffer to 128 words or 256 bytes. This is needed for peripheral indirect mode
#include <libpic30.h>

void pattern(void);
void ProcessADCSamples(__eds__ int * AdcBuffer);
void set_led(int j,uint8_t R,uint8_t G,uint8_t B);
void ProcessIO(void);
void led_off(int j);
void midi_send(uint8_t val,uint8_t index,uint8_t data_0,uint8_t data_1,uint8_t data_2);
uint8_t ScaleMapper(uint16_t meas);
uint8_t FullScaleCalc(uint16_t meas);

extern void system_ini(void);

#include <p24Exxxx.h>
 int FGS __attribute__((space(prog), address(0xF80004))) = 0xFFCF ;
 int FOSCSEL __attribute__((space(prog), address(0xF80006))) = 0xFFF8 ;
 int FOSC __attribute__((space(prog), address(0xF80008))) = 0xFF1B ;
 int FWDT __attribute__((space(prog), address(0xF8000A))) = 0xFF1F ;
 int FPOR __attribute__((space(prog), address(0xF8000C))) = 0xFFF7 ;
 int FICD __attribute__((space(prog), address(0xF8000E))) = 0xFFDA ;
 int FAS __attribute__((space(prog), address(0xF80010))) = 0xFFCF ;

/********************************************************************
 * Function:        void main(void)
 *******************************************************************/

// led 8 / 24 bit : 8 * 73 = 192 (+ 1)

uint16_t check_16[70] = {0};                // for ADC result
uint8_t send_data[200] = {0};                // for UART
uint16_t meas_value[64] = {0};              // minimum value among four detection point including a sensor
uint16_t pre_meas_value[64] = {};           // previous value of "meas_value"

uint8_t note_flg[16][16] = {{}};            // if measured value is active, this flag becomes active
uint8_t note_state[16][16] = {{}};          // if note is sent to midi buffer, this flag becomes active
uint8_t note_value[16][16] = {{}};          // sent note
uint8_t note_value_clk[16][16] = {{}};      // sent note when the clk_1_4_on == 1

uint8_t send_note[16][16] = {{}};
uint8_t send_note_log[16][100] = {{}};

uint8_t clk24_on = 0;                       // sensor position (0,4,8,12) flag
uint8_t clk_1_4_on = 0;                     // sensor position (0-15) flag

uint16_t *p_seq;
uint16_t *p_scale;
float scale_number;

uint16_t ad_offset[64] = {};                // for auto calibration
float coef[64] = {};                      // for auto calibration
float check_float[64] = {};                 // for auto calibration

uint16_t add_value[49] = {};                // save place for add calculation.

static uint8_t on_flag[LED_UNIT] = {};
uint8_t on_flag_2[LED_UNIT] = {};

uint8_t on_state[49] = {};

uint16_t led_num;
int16_t  temp_3 = 0;
uint8_t button_debug = 0;

uint16_t midi_send_count = 0;
uint16_t midi_send_num = 0;
uint16_t midi_send_mes = 0;

uint16_t midi_send_count_clk = 0;
uint16_t midi_send_num_clk = 0;
uint16_t midi_send_mes_clk = 0;

uint8_t adc_count;
uint8_t meas_count;

uint8_t sensor_num = 0;
uint8_t pre_sensor = 14;
uint8_t sensor_color = 0;

uint8_t R_ref = 255;
uint8_t B_ref = 0;
uint8_t G_ref = 0;

uint8_t auto_calib;

// midi function
uint8_t main_mode = MODE_SEQUENCE;          // button mode
uint8_t track = 0;                          // track
uint8_t looper[10] = {};                    // looper
uint8_t midi_scale = 0;                     // scale
uint8_t pad_seq = 0;                        // sequence

uint8_t pic_clk_mode = 1;                   // internal clock mode
uint8_t sequencer_refresh = 1;              // internal clock mode

uint16_t pic_clk_cont = 0;                  // internal clock mode
uint16_t pic_trig_on = 0;                   // internal clock mode
uint16_t pic_clk_set = 20;                  // internal clock mode. 20ms / 1step bpm120

uint16_t pic_clk_cont_tap1 = 0;             // internal clock mode
uint16_t serial_clk_cont = 0;               // serial clock mode
uint16_t timing_log = 0;                    // tap mode log
uint16_t double_click = 0;                  // for double click
uint16_t double_click_cnt = 0;              // for double click

uint16_t octave_m = 0;                      // for changing octave
uint8_t in_measure = 0;                     // measurement flag
uint8_t timing_error = 0;                   // error flag

uint8_t serial_flg = 0;

static uint8_t mode2_value = 0;
static uint8_t comm_send[16] = {};

static uint8_t sensor_count = 0;
static uint16_t clk_pic_mode_cont = 0;

MAIN_RETURN main(void) {

    // initilize variables
    auto_calib = 0;                     // for initialization
    p_seq = sensorseq_def[0];           // pointer of sequence
    p_scale = scale_address[0];         // pointer of scale
    scale_number = scale_num_def[0];
    for (led_num = 0; led_num < LED_UNIT; led_num++) {
        on_flag[led_num] = 0;
        on_flag_2[led_num] = 0;
    }
    for (led_num = 0; led_num < (LED_UNIT * 24); led_num++) {
        freq[led_num] = LOW;
    }
    for (led_num = 0; led_num < 64; led_num++) {
        pre_meas_value[led_num] = 10000;
    }
    freq[LED_UNIT * 24] = 0;
    // drum pad note set
    for(led_num = 0; led_num < 16; led_num++){
        note_value[led_num][7] = led_num + 35;
    }
 
    // initialize MCU setting
    system_ini();

    __delay_ms(50);         // 50ms delay

    //Start Timer 2 (main loop)
    T2CONbits.TON = 1; 

    while(1)
    {
//      SYSTEM_Tasks();

        #if defined(USB_POLLING)
            // Interrupt or polling method.  If using polling, must call
            // this function periodically.  This function will take care
            // of processing and responding to SETUP transactions
            // (such as during the enumeration process when you first
            // plug in).  USB hosts require that USB devices should accept
            // and process SETUP packets in a timely fashion.  Therefore,
            // when using polling, this function should be called
            // regularly (such as once every 1.8ms or faster** [see
            // inline code comments in usb_device.c for explanation when
            // "or faster" applies])  In most cases, the USBDeviceTasks()
            // function does not take very long to execute (ex: <100
            // instruction cycles) before it returns.
            USBDeviceTasks();
        #endif

        // if( USBGetDeviceState() < CONFIGURED_STATE )
        // {
        //     continue;
        // }

        /* If we are currently suspended, then we need to see if we need to
         * issue a remote wakeup.  In either case, we shouldn't process any
         * keyboard commands since we aren't currently communicating to the host
         * thus just continue back to the start of the while loop. */
        
        // if( USBIsDeviceSuspended()== true )
        // {
            /* Jump back to the top of the while loop. */
        //     continue;
        // }

	// Application-specific tasks.
	// Application related code may be added here, or in the ProcessIO() function.

        ProcessIO();

    }//end while
}//end main

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
// MIDI send parts. always running.
void ProcessIO(void){
    static uint8_t buff;
    static uint8_t buff_b;
    static uint8_t clk_cnt = 0;
 
    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    if (!USBHandleBusy(USBTxHandle)) {
        if (midi_send_num_clk != 0) {
            if (in_measure == 0) {
                USBTxHandle = USBTxOnePacket(MIDI_EP, (uint8_t*) & midiData_clk[midi_send_mes_clk], 4);

                midi_send_mes_clk++;
                if (midi_send_mes_clk == 1024) {
                    midi_send_mes_clk = 0;
                }
                midi_send_num_clk--;
            }
        }

        if (midi_send_num != 0) {
            if (in_measure == 0) {
                USBTxHandle = USBTxOnePacket(MIDI_EP, (uint8_t*) & midiData_b[midi_send_mes], 4);

                midi_send_mes++;
                if (midi_send_mes == 1024) {
                    midi_send_mes = 0;
                }
                midi_send_num--;
            }
        }
    }

    if (!USBHandleBusy(USBRxHandle)) {
        USBRxHandle = USBRxOnePacket(MIDI_EP, (uint8_t*) & ReceivedDataBuffer, 2);

        buff = ReceivedDataBuffer[0];
        buff_b = ReceivedDataBuffer[1];

        if (buff_b == 0xf8 && buff == 0x0f) {
            clk_cnt++;

            if(pic_clk_mode == 1){
                pic_clk_mode = 0;
                clk_cnt = 1;
                sensor_num = 0;
                sensor_count = 0;
                pre_sensor = 14;
            }

            clk_pic_mode_cont = 0;
            sequencer_refresh = 0;
            pic_clk_cont_tap1 = 0;

            if (clk_cnt > 5) {

                switch (sensor_count) {
                    case 0:
                    case 4:
                    case 8:
                    case 12:
                        clk24_on = 1;
                        break;
                    default:
                        break;
                }

                pre_sensor = sensor_num;                // save previous selected sensor
                sensor_num = *(p_seq + sensor_count);   // select new sensor pos

                if(sensor_count == 15){
                    timing_log = 0;
                }

                if (sensor_count == 15) {
                    sensor_count = 0;
                } else {
                    sensor_count++;
                }

                clk_cnt = 0;
                clk_1_4_on = 1;
            }

            if (in_measure == 0) {
                in_measure = 1;         // measurement sequence is in progress
                temp_3 = 0;
                TMR1 = 9599;
                PR1 = 9600;             // Trigger ADC1 every 200 usec // 47.9MHz
                T1CONbits.TON = 1;      //Start Timer 1
            } else {
                timing_error = 1;
                in_measure = 0; // measurement sequence end
            }
        }
    }
}

// running every 1ms. for button detecton/action and internal clock action.
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void) {

    static uint8_t trig = 0;
    static uint8_t trig_btn5 = 0;
    static uint8_t trig_comm = 0;
    static uint8_t btn = 0;
    static uint8_t btn_all = 0;
    static uint8_t btn1,btn2,btn3,btn4,btn5;
    static uint8_t i,k;
    static uint16_t clk_pic = 0;
    static uint16_t long_push = 0;
    static uint16_t long_push_cont = 0;
    static uint8_t Active_Count = 5;
    static uint8_t pre_mode;
 
    // RG0,1  RD5,6,7
    btn = (PORTG & 0x03) | ((PORTD >> 3) & 0x1c);
#if(OLD_PCB == 1)
    btn1 = (btn & 0x04) >> 2;
    btn2 = (btn & 0x10) >> 4;
    btn3 = (btn & 0x08) >> 3;
    btn4 = (btn & 0x01);
    btn5 = (btn & 0x02) >> 1;
    btn_all = (btn1 << 2) | (btn2 << 4) | (btn3 << 3) | btn4 | (btn5 << 1);
    btn = (PORTG & 0x01) | ((PORTD >> 3) & 0x1c);
#else
    btn1 = (btn & 0x01);
    btn2 = (btn & 0x02) >> 1;
    btn3 = (btn & 0x10) >> 4;
    btn4 = (btn & 0x08) >> 3;
    btn5 = (btn & 0x04) >> 2;
    btn_all = (btn1 << 2) | (btn2 << 4) | (btn3 << 3) | btn4 | (btn5 << 1);
    btn = (PORTG & 0x03) | ((PORTD >> 3) & 0x18);
#endif

    // send serial to PC when pushing button ( only one button push)
    if(btn_all != 0 && trig_comm == 0){
        if(btn_all == 0x02){    // only btn5 pushed
        } else {
            while (U1STAbits.TRMT != 1);
            U1TXREG = 0xc0 | btn_all;
            trig_comm = 1;
        }
    } else {
        if((btn_all == 0) || (btn_all == 0x02)){
            trig_comm = 0;
        }
    }

    // octave move
    if(main_mode == MODE_PAD){
        if(btn2 == 1){
            octave_m = 1;
        } else if(btn4 == 1){
            octave_m = 2;
        } else {
            octave_m = 0;
        }
    }

    // internal clock mode
    if (pic_clk_mode == 1) {
        pic_clk_cont++;
        if (pic_clk_set < pic_clk_cont) {
            pic_clk_cont = 0;

            if (serial_clk_cont == 0) {
                if (sequencer_refresh == 1) {
                    clk_pic++;
                    if (clk_pic > Active_Count) {
                        clk_pic = 0;
                        if (sensor_count == 15) {
                            timing_log = 0;
                        }

                        if (sensor_count == 15) {
                            sensor_count = 0;
                        } else {
                            sensor_count++;
                        }

                        pre_sensor = sensor_num; // save previous selected sensor
                        sensor_num = *(p_seq + sensor_count); // select new sensor pos
                        switch (sensor_count) {
                            case 0:
                            case 4:
                            case 8:
                            case 12:
                                clk24_on = 1;
                                break;
                            default:
                                break;
                        }

                        clk_1_4_on = 1;
                    }
                    midi_send(0, MIDI_CIN_MTC, 0xf8, 0, 0);
                }
            }

            if (in_measure == 0) {
                in_measure = 1; // measurement sequence is in progress
                temp_3 = 0;
                TMR1 = 9599;
                PR1 = 9600; // Trigger ADC1 every 200 usec // 47.9MHz
                T1CONbits.TON = 1; //Start Timer 1
             } else {
                timing_error = 1;
                in_measure = 0; // measurement sequence end
            }
        }

        if(pic_clk_cont == 65535){
            pic_clk_cont = 0;
        }
    }

    // tap tempo reset
    if(pic_trig_on == 1){
        pic_clk_cont_tap1++;
        if(pic_clk_cont_tap1 > 3100){
            pic_clk_cont_tap1 = 3100;
            pic_trig_on = 0;
        }
    }

    // sensor num reset
    if((pic_trig_on == 0) && (pic_clk_mode == 0)){
        pic_clk_cont_tap1++;
        if (pic_clk_cont_tap1 == 1000) {
            sensor_num = 0;
            sensor_count = 0;
            pre_sensor = 14;
        }

        if (pic_clk_cont_tap1 > 1100) {
            pic_clk_cont_tap1 = 1100;
        }
    }

    // double click counter
    double_click_cnt++;
    if(double_click_cnt > 1000){
        double_click_cnt = 1000;
        double_click = 0;
    }

    // button action
    if (btn != 0 && trig == 0) {
        // button 5
        if (btn5 == 1) { // shift mode
            if (btn1 == 1) { // tap tempo. to internal clock mode
                if (main_mode == MODE_DEBUG) {
                    button_debug = 5;
                } else {
                    if (pic_clk_mode == 1) {
                        if (pic_trig_on == 0) {
                            pic_clk_cont_tap1 = 0;
                            pic_trig_on = 1;
                        } else if (pic_trig_on == 1) {
                            // pic_clk_set = pic_clk_cont_tap1 >> 2;   // devided by 4
                            // 1 / 24 = 0x04166
                            pic_clk_set = (uint16_t) ((float) (pic_clk_cont_tap1) * (float) (0.0416666)); // divided by 24
                            if (pic_clk_set < 5) {
                                pic_clk_set = 5;
                            }
                            pic_trig_on = 0;
                        }
                    } else {
                        pic_clk_mode = 1;
                        sequencer_refresh = 1;
                        sensor_num = 0;
                        sensor_count = 0;
                        pre_sensor = 14;
                        pic_clk_cont = 60000;
                    }
                }
            } else if (btn2 == 1) { // sequence change
                pad_seq++;
                if (pad_seq == 6) {
                    pad_seq = 0;
                }
                p_seq = sensorseq_def[pad_seq];
                long_push = 2;
            } else if (btn3 == 1) { // loop data clear
                if (main_mode == MODE_SEQUENCE) {
                    for (k = 0; k < 16; k++) {
                        note_flg[k][track] = 0;
                    }
                }

                if (main_mode == MODE_PAD) {
                    for (i = 0; i < 16; i++) {
                        for (k = 0; k < 100; k++) {
                            send_note_log[i][k] = 0;
                        }

                        for (k = 0; k < 16; k++) {
                            note_value_clk[i][k] = 0;
                        }
                    }
                }
            } else if (btn4 == 1) { // loop saved note all clear

                for (i = 0; i < 16; i++) {
                    for (k = 0; k < 100; k++) {
                        send_note_log[i][k] = 0;
                    }

                    for (k = 0; k < 16; k++) {
                       note_value_clk[i][k] = 0;
                    }
                }
            }

        } else if (main_mode != MODE_CC) { // normal mode except CC Message Mode
            if (btn1 == 1) {
                if (main_mode == MODE_DEBUG) {
                    button_debug = 1;
                } else {
                    main_mode++;
                    if (main_mode == MODE_PAD) {
                        for (led_num = 0; led_num < LED_UNIT; led_num++) {
                            on_flag_2[led_num] = 0;
                        }
                    }
                    if (main_mode == MODE_CC) {
                        main_mode = MODE_SEQUENCE;
                    }
                }
            }

            if (btn2 == 1) {
                if (main_mode == MODE_DEBUG) {
                    button_debug = 2;
                } else {
                    if (main_mode == MODE_SEQUENCE) {
                        track++;
                        if (track == 6) {
                            track = 0;
                        }

                        switch (track) {
                            case 0: R_ref = 255;
                                B_ref = 0;
                                G_ref = 0;
                                break;
                            case 1: R_ref = 0;
                                B_ref = 255;
                                G_ref = 0;
                                break;
                            case 2: R_ref = 0;
                                B_ref = 0;
                                G_ref = 255;
                                break;
                            case 3: R_ref = 255;
                                B_ref = 255;
                                G_ref = 0;
                                break;
                            case 4: R_ref = 255;
                                B_ref = 0;
                                G_ref = 255;
                                break;
                            case 5: R_ref = 0;
                                B_ref = 255;
                                G_ref = 255;
                                break;
                            case 6: R_ref = 0;
                                B_ref = 0;
                                G_ref = 0;
                                break;
                            case 7: R_ref = 255;
                                B_ref = 255;
                                G_ref = 255;
                                break;
                            default: break;
                        }
                    }
                }
            }

            if (btn3 == 1) { //looper on (all on)
                if (main_mode == MODE_DEBUG) {
                    button_debug = 3;
                } else {
                    if (looper[0] == 0) {

                        for (i = 0; i < 10; i++) {
                            looper[i] = 1;
                        }

                    } else {

                        for (i = 0; i < 10; i++) {
                            looper[i] = 0;
                            for (k = 0; k < 16; k++) {
                                note_flg[k][i] = 0;
                            }
                        }

                        for (i = 0; i < 16; i++) {
                            for (k = 0; k < 100; k++) {
                                send_note_log[i][k] = 0;
                            }

                            for (k = 0; k < 16; k++) {
                                note_value_clk[i][k] = 0;
                            }
                        }

                    }
                }
            }

            if (btn4 == 1) { // change scale
                if (main_mode == MODE_DEBUG) {
                    button_debug = 4;
                } else {
                    if (main_mode != MODE_PAD) {
                        midi_scale++;
                        if (midi_scale == 17) {
                            midi_scale = 0;
                        }
                        scale_number = scale_num_def[midi_scale];
                        p_scale = scale_address[midi_scale];

                    }
                }
            }
        }

        trig = 1;
    } else {
        if (btn == 0) {     // continuous action except button 5
            trig = 0;
            long_push = 0;
            long_push_cont = 0;

            // CC Message mode
            if(trig_btn5 == 0){
                if (btn5 == 1) {
                    if (main_mode == MODE_CC) {
                        main_mode = pre_mode;
                        double_click = 0;
                    } else if (double_click == 1 && double_click_cnt < 200) {
                        if (main_mode == MODE_DEBUG) {
                            button_debug = 6;
                        } else {
                            pre_mode = main_mode;
                            main_mode = MODE_CC;
                            double_click = 0;
                        }
                    } else if (double_click == 0) {
                        double_click = 1;
                        double_click_cnt = 0;
                    }
                    trig_btn5 = 1;
                }
            } else {
                if (btn5 == 0) {
                    trig_btn5 = 0;
                }
            }
        } else {
        }
    }

    IFS0bits.T2IF = 0;      // Clear Timer 2 interrupt
    //  T2CONbits.TON = 0;  // Stop Timer 2
}

// UART receive interruput
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) {

    static uint8_t buff,buff_2,buff_3,buff_4;
    static uint8_t i,k;

    while (U1STAbits.URXDA == 1) {

        buff = U1RXREG;
        buff_2 = (buff >> 6) & 0x03;
        buff_4 = buff & 0x0f;

        serial_clk_cont = 1;

        if(buff_2 == 2){
            if(buff_4 == 0){
                main_mode = MODE_MULTITOUCH;
                pic_clk_mode = 1;
                sequencer_refresh = 1;
                pic_clk_set = 5;
            } else if(buff_4 == 1){
                if(main_mode != MODE_DEBUG){
                    main_mode = MODE_DEBUG;          // debug mode
                    button_debug = 0;
                }
            } else if(buff_4 == 2){
                main_mode = MODE_PAD;
                pic_clk_mode = 1;
                sequencer_refresh = 1;
                pic_clk_set = 5;
            } else if(buff_4 == 4){
                main_mode = MODE_PRESSURE;
            } else if(buff_4 == 8) {
                main_mode = MODE_SEQUENCE;
            }
        } else if(buff_2 == 0 || buff_2 == 1){
            if ((main_mode == MODE_SEQUENCE) || (main_mode == MODE_PRESSURE)) {
                pic_clk_mode = 0;
                sensor_num = 0;
                sensor_count = 0;
                pre_sensor = 14;
            }
            sensor_num = buff & 0x0f;
            buff_3 = (buff >> 4) & 0x07;
            if (buff_3 != 0) {
                track = buff_3 - 1;
            }

            if(main_mode == MODE_PAD){
                serial_flg = 1;
            }

            if((main_mode == MODE_PRESSURE) || (main_mode == MODE_PAD)){
                mode2_value = sensor_num;
                clk_1_4_on = 1;
            }
        } else if(buff_2 == 3){
            if(buff_4 == 0) {
                looper[7] = 0;
                for (i = 0; i < 16; i++) {
                    for (k = 0; k < 16; k++) {
                        note_value_clk[i][k] = 0;
                    }
                }
            } else if (buff_4 == 1) {
                looper[7] = 1;
            }
        }

        if (buff_3 < 8) {
            R_ref = color_data[buff_3][0];
            B_ref = color_data[buff_3][1];
            G_ref = color_data[buff_3][2];
        }

        if ((main_mode == MODE_SEQUENCE) || (main_mode == MODE_PRESSURE) || (main_mode == MODE_DEBUG)) {
            if (in_measure == 0) {
                in_measure = 1; // measurement sequence is in progress
                temp_3 = 0;
                TMR1 = 9599;
                PR1 = 9600; // Trigger ADC1 every 200 usec // 47.9MHz
                T1CONbits.TON = 1; //Start Timer 1
            } else {
                timing_error = 1;
                in_measure = 0; // measurement sequence end
            }
        }
    }

    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;
    }

    IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
}

// for sensor-led flash sequence
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {

    if(temp_3 == 250){   // 50 ms
    } else {
        temp_3++;
    }

    if(temp_3 == 1){
        meas_count = 0;
        PORTD |= 0x000f;    // led1_1,led_1_2,led1_3,led1_4 ON
    }

    if(temp_3 == 6){
        meas_count = 1;
        PORTD |= 0x0f00;    // led2_1,led_2_2,led2_3,led2_4 ON
    }

    if (temp_3 == 3 || temp_3 == 10) {     // 250 ms
        adc_count = 0;
        AD1CON1bits.SAMP = 1;
        T3CONbits.TON = 1; //Start Timer 3
    }

    if (temp_3 == 10) {
        T1CONbits.TON = 0; //Stop Timer 1
    }

    IFS0bits.T1IF = 0; // Clear Timer 1 interrupt
}

// AD Converter timer
void __attribute__((interrupt, auto_psv)) _T3Interrupt(void) {

    IFS0bits.T3IF = 0; // Clear Timer 3 interrupt

    if (adc_count < 31) {
        adc_count++;
        AD1CON1bits.SAMP = 1;
    } else {
        T3CONbits.TON = 0;  // Stop Timer 3
    }
}

// action after finishing AD converter measurement
void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void) {
    static int i,j,m,k;
    static uint16_t meas_sum;
    static uint8_t timing_flg = 0;
    static uint8_t quantize_value = 0;
    static uint8_t looper_flg = 0;
    static uint8_t tap_flg = 0;
    static uint8_t tap_flg_off = 0;


    T3CONbits.TON = 0; //Stop Timer 3
    ProcessADCSamples(&BufferA[0]);
    IFS0bits.DMA1IF = 0; // Clear the DMA0 Interrupt Flag

    PORTD &= 0xf0f0; // all led off
    
    if (meas_count == 1) {
        meas_count = 2;
        T1CONbits.TON = 0; //Stop Timer 1

        // for CC Message
        if(main_mode == MODE_CC){
            for (i = 0; i < 16; i++) {
                if(meas_value[i] > 10){
                    if(meas_value[i] > 137){
                        meas_value[i] = 137;
                    }
                    comm_send[i] = 1;
                    midi_send(0, MIDI_CIN_CONTROL_CHANGE, 0xB0, 102 + i, meas_value[i] - 10);
                } else if(comm_send[i] == 1){
                    midi_send(0, MIDI_CIN_CONTROL_CHANGE, 0xB0, 102 + i, 0);
                    comm_send[i] = 0;
                }
            }
        }

        tap_flg = 0;
        tap_flg_off = 0;
        for (i = 0; i < 16; i++) {
            // ----- for pad ----
            looper_flg = 0;
            if (main_mode == MODE_PAD) { // for pad
                if(meas_value[i] > pre_meas_value[i]){
                    pre_meas_value[i] = meas_value[i] - pre_meas_value[i];
                } else {
                    pre_meas_value[i] = 0;
                }

              if ((pre_meas_value[i] > 30) && (note_flg[i][7] == 0)) {
                    note_flg[i][7] = 1;
                    send_note_log[i][timing_log] = (pre_meas_value[i] - 20);
                    if (send_note_log[i][timing_log] > 0x7F) {
                        send_note_log[i][timing_log] = 0x7F;
                    }

                    if (looper[7] == 1) {
                        if (serial_clk_cont == 0) {
                            if (timing_log > (sensor_count * 6)) {
                                quantize_value = timing_log - (sensor_count * 6);
                            } else {
                                quantize_value = 0;
                            }

                            if (quantize_value < 3) {
                                note_value_clk[i][sensor_num] = send_note_log[i][timing_log];
                            } else {
                                looper_flg = 1;
                                if (sensor_count == 15) {
                                    // sensor_num = *(p_seq + sensor_count); // select new sensor pos
                                    note_value_clk[i][*(p_seq)] = send_note_log[i][timing_log];
                                } else {
                                    note_value_clk[i][*(p_seq + sensor_count + 1)] = send_note_log[i][timing_log];
                                }
                            }
                        } else {
                            note_value_clk[i][sensor_num] = send_note_log[i][timing_log];
                        }
                    } else {
                         if (serial_clk_cont == 1) {
                            note_value_clk[i][sensor_num] = send_note_log[i][timing_log];
                         }
                    }

                    timing_flg = 1;
                } else if ((meas_value[i] < 30) && (note_flg[i][7] == 1) && (note_state[i][7] == 1)) {
                    note_flg[i][7] = 0;
                    /*
                    if (serial_clk_cont == 1) {
                         if (looper[7] == 1) {
                                note_value_clk[i][sensor_num] = 0;
                         }
                    }
                    */
           //       tap_flg_off = 1;
                }
                pre_meas_value[i] = meas_value[i];

                if (clk_1_4_on == 1) {
                    for (m = 0; m < 8; m++) {
                        on_flag_2[127 - (i * 8 + m)] = 0;
                    }
                }
            }

            if (((note_state[i][7] == 1) && (note_flg[i][7] == 0) && (looper_flg == 0)) || ((looper[7] == 1 && clk_1_4_on == 1 && note_value_clk[i][pre_sensor] != 0))) {
                midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + 7, send_note[i][7], 0);
                note_state[i][7] = 0;
                tap_flg_off = 1;
            }

            if ((note_flg[i][7] == 1 && note_state[i][7] == 0 && looper_flg == 0) || (looper[7] == 1 && clk_1_4_on == 1 && note_value_clk[i][sensor_num] != 0)){

                if(looper[7] == 1 && clk_1_4_on == 1 && note_value_clk[i][sensor_num] != 0){
                    midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + 7, note_value[i][7] + (octave_m * 12), note_value_clk[i][sensor_num]);
                } else {
                    midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + 7, note_value[i][7] + (octave_m * 12), send_note_log[i][timing_log]);
                }

                send_note[i][7] = note_value[i][7] + (octave_m * 12);
                tap_flg = 1;
                note_state[i][7] = 1;

                for (m = 0; m < 8; m++) {
                    on_flag_2[127 - (i * 8 + m)] = 1;
                }
            }
        }

        // for sequencer
        if (clk_1_4_on == 1) {
            if(main_mode == MODE_SEQUENCE){
                if (meas_value[sensor_num] > 10) {
                    note_value[sensor_num][track] = FullScaleCalc(meas_value[sensor_num]);
                    note_flg[sensor_num][track] = 1;
                }
            }

            for (i = 0; i < 6; i++) {
                if (note_state[pre_sensor][i] == 1) {
                    midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + i, send_note[pre_sensor][i], 0);
                    note_state[pre_sensor][i] = 0;
                }
            }

            for (i = 0; i < 6; i++) {
                if (note_flg[sensor_num][i] == 1) {
                    // for debug
                    send_note[sensor_num][i] = ScaleMapper(note_value[sensor_num][i]);
                    midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + i, send_note[sensor_num][i], 0x7F);
                    note_state[sensor_num][i] = 1;
                    if (looper[i] == 0) {
                        note_flg[sensor_num][i] = 0;
                    }
                }
            }
        }

        // ----- for pressure ----
        if(main_mode == MODE_PRESSURE){
            meas_sum = 0;
            for (i = 0; i < 16; i++) {
                meas_sum += meas_value[i];
            }
            meas_sum >>= 5;

            if (clk_1_4_on == 1) {
                if (note_state[0][6] == 1) {
                    midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + 6, send_note[0][6], 0);
                    note_state[0][6] = 0;
                }

                note_value[0][6] = ScaleMapper(meas_sum) + 0x30;

                if (meas_sum > 10) {
                    midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + 6, note_value[0][6], 0x7F);
                    note_state[0][6] = 1;
                    send_note[0][6] = note_value[0][6];
                }
            }
        }

        // ----- for Multitouch ----
        if (main_mode == MODE_MULTITOUCH) {
            for (j = 0; j < 49; j++) {
                add_value[j] = check_16[led_comb_finger[j][0]]
                        + check_16[led_comb_finger[j][1]]
                        + check_16[led_comb_finger[j][2]]
                        + check_16[led_comb_finger[j][3]];

                if ((add_value[j] < 150) && on_state[j] == 1) {
                    on_state[j] = 0;
                    midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + 8, ch_value[j], 0);
                } else if (((add_value[j] > 200) && on_state[j] == 0) || on_state[j] == 1) {
                    if (timing_error == 0) {
                        for (m = 0; m < 8; m++) {
                            if (led_combination[j][m] != -1) {
                                on_flag[led_combination[j][m]] = 1;
                            }
                        }

                        if (on_state[j] == 0) {
                            midi_send(0, MIDI_CIN_NOTE_ON, 0x90 + 8, ch_value[j], 0x7F);
                            while (U1STAbits.TRMT != 1);
                            U1TXREG = j;
                        }
                        
                        on_state[j] = 1;
                    }
                }
            }
        }

        /*
        if (main_mode == MODE_MULTITOUCH) {
            // debug
            for (k = 0; k < 64; k++) {
                send_data[k * 2] = 0x3f & (uint8_t) (check_16[k] >> 6);
                send_data[k * 2 + 1] = 0x40 | (uint8_t) (check_16[k] & 0x003f);

                while (U1STAbits.TRMT != 1);
                U1TXREG = send_data[k * 2];
                while (U1STAbits.TRMT != 1);
                U1TXREG = send_data[k * 2 + 1];
            }

            while (U1STAbits.TRMT != 1);
            U1TXREG = 0x0d;
            while (U1STAbits.TRMT != 1);
            U1TXREG = 0x0a;
            // debug end
        }
        */

        if(main_mode == MODE_SEQUENCE || main_mode == MODE_PRESSURE){
            for (i = 0; i < 32; i++) {
                __delay_us(5);
                while (U1STAbits.TRMT != 1);
                U1TXREG = send_data[i];
            }
        }

        /*
        if (main_mode == MODE_PAD && (tap_flg_off == 1)) {
            for (k = 0; k < 16; k++) {
                send_data[k * 2] = 0;
                send_data[k * 2 + 1] = 0x40;

                __delay_us(5);
                while (U1STAbits.TRMT != 1);
                U1TXREG = send_data[k * 2];
                while (U1STAbits.TRMT != 1);
                U1TXREG = send_data[k * 2 + 1];
            }
        }
*/

        if (main_mode == MODE_PAD && serial_flg == 1) {
            for (k = 0; k < 16; k++) {
                send_data[k * 2] = 0x0f & (uint8_t) (note_value_clk[k][sensor_num] >> 6);
                send_data[k * 2 + 1] = 0x40 | (uint8_t) (note_value_clk[k][sensor_num] & 0x003f);

                __delay_us(5);
                while (U1STAbits.TRMT != 1);
                U1TXREG = send_data[k * 2];
                while (U1STAbits.TRMT != 1);
                U1TXREG = send_data[k * 2 + 1];

                if (looper[7] == 0) {
                    send_note_log[k][timing_log] = 0;
                    note_value_clk[k][sensor_num] = 0;
                }
            }
            serial_flg = 0;
        }

        if(main_mode == MODE_DEBUG){
            for (i = 0; i < 64; i++) {
            //    __delay_us(5);
                while (U1STAbits.TRMT != 1);
                U1TXREG = 0x0f & (uint8_t) (check_16[i] >> 6);
            //    __delay_us(5);
                while (U1STAbits.TRMT != 1);
                U1TXREG = 0x40 | (uint8_t) (check_16[i] & 0x003f);
            }

            while (U1STAbits.TRMT != 1);
            U1TXREG = 0x0d;
            while (U1STAbits.TRMT != 1);
            U1TXREG = 0x0a;
        }

        pattern();
      
        clk24_on = 0;
        clk_1_4_on = 0;

        auto_calib = 1;

        DMA0CONbits.CHEN = 0; // Enable DMASet up Timer2 for Output Compare PWM mode:
        DMA0CONbits.CHEN = 1; // Enable DMASet up Timer2 for Output Compare PWM mode:
        timing_error = 0;

        timing_log++;
        if (timing_log == 100) {
            timing_log--;
        }
        
    } else if (meas_count == 0) {
        // no action (first time measurement)
    }
}

// LED update complete
void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
    DMA0CONbits.CHEN = 0; // disable DMA0 channel

    in_measure = 0; // measurement sequence end
}

// decide led light color
void pattern(void) {
    static int j,k;
    static uint8_t mode2_count = 0;
    static uint8_t white_clr = 18;
    static uint8_t led_on_cnt = 0;
    static uint8_t v_r,v_g,v_b;
    static uint8_t buff;

    k = 0;
    led_on_cnt = 0;
    
    if(PORTDbits.RD13 == 1){
        white_clr = 54;
    } else {
        white_clr = 18;
    }

    if (main_mode == MODE_PRESSURE) {
        if (clk24_on == 1) {
            mode2_count++;
            if (mode2_count == 4) {
                mode2_count = 0;
            }

            switch (mode2_count) {
                case 0: mode2_value = 0;
                    break;
                case 1: mode2_value = 3;
                    break;
                case 2: mode2_value = 15;
                    break;
                case 3: mode2_value = 12;
                    break;
                default: break;
            }
        }
    }

    for (j = 0; j < LED_UNIT; j++) {
        if (serial_clk_cont == 0) {
            if (timing_error == 1) {
                on_flag[j] = 0;
                on_flag_2[j] = 0;
            }
        }

        if (main_mode == MODE_SEQUENCE) {
            if (j == led_combination[sensor_num][k]) {
                if (k < 8) {
                    if (note_state[sensor_num][track] == 1) {
                        set_led(j, 128, 128, 128); // exist data
                    } else {
                        set_led(j, R_ref, G_ref, B_ref);
                    }
                    k++;
                }
            } else {
                led_off(j);
            }
        } else if (main_mode == MODE_PRESSURE) {
            if (j == led_combination[mode2_value][k]) {
                if (k < 8) {
                    set_led(j, white_clr, white_clr, white_clr); // exist data
                    k++;
                }
            } else {
                led_off(j);
            }
        } else if (main_mode == MODE_PAD) {

            if (j == led_on_select_1[led_on_cnt]) {         // led brink
                if ((15 - led_on_cnt) == sensor_num) {
                    v_r = 0;
                    v_g = 0;
                    v_b = 0;
                } else {
                    v_r = white_clr;
                    v_g = white_clr;
                    v_b = white_clr;
                }
                led_on_cnt++;
            } else {
                v_r = 0;
                v_g = 0;
                v_b = 0;
            }

            if (on_flag_2[j] == 1) {
                v_r = white_clr;
                v_g = white_clr;
                v_b = white_clr;
            } 

            set_led(j,v_r,v_g,v_b);

        } else if (main_mode == MODE_MULTITOUCH) {
            if (j == led_on_select_2[led_on_cnt]) {
                v_r = white_clr;
                v_g = white_clr;
                v_b = white_clr;
                led_on_cnt++;
            } else {
                v_r = 0;
                v_g = 0;
                v_b = 0;
            }

            if (on_flag[j] == 1) {
                v_r = white_clr;
                v_g = white_clr;
                v_b = white_clr;

                on_flag[j] = 0;
            }
            set_led(j, v_r, v_g, v_b);

        } else if (main_mode == MODE_CC) {
            if(j > ((led_on_cnt * 8) + 7)){
                led_on_cnt++;
            }
            buff = j - (led_on_cnt * 8);
            if (buff == 0) {
                v_r = dial_led_color[0][0];
                v_g = dial_led_color[0][1];
                v_b = dial_led_color[0][2];

                // v_r = 0; v_g = 0; v_b = 0;
            } else {
                /*
                meas_value[0] = 255;
                meas_value[2] = 255;
                meas_value[5] = 255;
                meas_value[7] = 255;
                meas_value[8] = 255;
                meas_value[10] = 255;
                meas_value[13] = 255;
                meas_value[15] = 255;
                */
                if (meas_value[15 - led_on_cnt] > buff * 18) {
                    v_r = dial_led_color[buff][0];
                    v_g = dial_led_color[buff][1];
                    v_b = dial_led_color[buff][2];
                } else {
                    v_r = 0;
                    v_g = 0;
                    v_b = 0;
                }
            }

            set_led(dial_led[buff] + (led_on_cnt * 8), v_r, v_g, v_b);
        } else if (main_mode == MODE_DEBUG){
            v_r = white_clr; v_g = white_clr; v_b = white_clr;
            switch(button_debug){
                case 0: led_off(j);                 break;
                case 1: set_led(j, v_r, 0, 0);  break;
                case 2: set_led(j, 0, v_g, 0);  break;
                case 3: set_led(j, 0, 0, v_b);  break;
                case 4: set_led(j, 0, v_g, v_b);  break;
                case 5: set_led(j, v_r, v_g, v_b);  break;
                case 6: set_led(j, 0, 0, 0);  break;
                default:    break;
            }
        }

        if (auto_calib == 0) {
            led_off(j);
        }
    }

}

// check the ADC result value
void ProcessADCSamples(__eds__ int * AdcBuffer){
    static int j;
    static uint16_t buff_2;

    // read all measured value (32 values) of AD converter
    for(j = 0; j < 32; j++){
        if(meas_count == 0){
            check_16[j] = *(AdcBuffer + adc_pos[j]);
        } else {
            check_16[j + 32] = *(AdcBuffer + adc_pos[j]);
        }
    }

    if (meas_count == 1) {
        // pot has two leds. this section occur after second time measurement.
        if (auto_calib == 1) {
            for (j = 0; j < 64; j++) {
                if (check_16[j] < ad_offset[j]) {
                    check_float[j] = (float) (ad_offset[j]) - (float) (check_16[j]);    //  offset subtraction
                } else {
                    check_float[j] = 0;
                }
                check_float[j] = check_float[j] * coef[j];
                check_16[j] = (uint16_t) (check_float[j]);
            }
        } else {
            // sets offset value and calculate the coeficients
            for (j = 0; j < 64; j++) {
                ad_offset[j] = check_16[j];
                coef[j] = GetCoef(ad_offset[j]);
            }
        }

        for (j = 0; j < 16; j++) {
            // calculate minimum value and create send data for UART
            buff_2 = 0xffff;
            if (buff_2 > check_16[j * 2])
                buff_2 = check_16[j * 2];

            if (buff_2 > check_16[j * 2 + 1])
                buff_2 = check_16[j * 2 + 1];

            if (buff_2 > check_16[(j + 16) * 2])
                buff_2 = check_16[(j + 16) * 2];

            if (buff_2 > check_16[(j + 16) * 2 + 1])
                buff_2 = check_16[(j + 16) * 2 + 1];

            meas_value[j] = buff_2;
            send_data[j * 2] = 0x0f & (uint8_t) (buff_2 >> 6);
            send_data[j * 2 + 1] = 0x40 | (uint8_t) (buff_2 & 0x003f);
        }
    }
}

// set the message to the send cue
void midi_send(uint8_t val, uint8_t index, uint8_t data_0, uint8_t data_1, uint8_t data_2) {

    static uint8_t buff = 0;
    midiData.Val = 0; //must set all unused values to 0 so go ahead
    midiData.CableNumber = val;
    midiData.CodeIndexNumber = index;
    midiData.DATA_0 = data_0; //Note off
    midiData.DATA_1 = data_1; //pitch
    midiData.DATA_2 = data_2; //velocity 0, note off

    if(index == MIDI_CIN_MTC){
        midiData_clk[midi_send_count_clk] = midiData;

        midi_send_count_clk++;
        if (midi_send_count_clk == 1024) {
            midi_send_count_clk = 0;
            buff = buff;
        }

        midi_send_num_clk++;

        if (midi_send_num_clk == 1024) {
            midi_send_num_clk--;
            buff = buff;
        }
    } else {
        midiData_b[midi_send_count] = midiData;

        midi_send_count++;
        if (midi_send_count == 1024) {
            midi_send_count = 0;
            buff = buff;
        }

        midi_send_num++;

        if (midi_send_num == 1024) {
            midi_send_num--;
            buff = buff;
        }
    }
}

// recalc the sound scale
uint8_t ScaleMapper(uint16_t meas){

    static uint8_t buff;
    static uint8_t buff_2;

    // 1 / 127 = 0.007874
    // 1 / 95 =  0.010526
    buff = (uint8_t)((float)(meas) * scale_number * 0.007874);
    buff_2 = *(p_scale + buff);

    return buff_2;
}

// check the full scale
uint8_t FullScaleCalc(uint16_t meas){
    static uint8_t buff;
    // 127 / 200 = 0.635
    // 95 / 200 = 0.475
    buff = (uint8_t)((float)(meas) * 0.475);

    if(buff > 95)
        buff = 95;

    return buff;
}

// set led color
void set_led(int j,uint8_t R,uint8_t G,uint8_t B) {
    static uint16_t i;
    static uint16_t r,p,q;

    r = j * 24;
    p = j * 24 + 8;
    q = j * 24 + 16;

    for (i = 0; i < 8; i++) {
        freq[r + i] = (G & color_bit[i]) ? HIGH : LOW;
        freq[p + i] = (R & color_bit[i]) ? HIGH : LOW;
        freq[q + i] = (B & color_bit[i]) ? HIGH : LOW;
    }
}

// set led off
void led_off(int j) {
    static uint16_t i;
    static uint16_t r,p,q;

    r = j * 24;
    p = j * 24 + 8;
    q = j * 24 + 16;

    for (i = 0; i < 8; i++) {
        freq[r + i] = LOW;
        freq[p + i] = LOW;
        freq[q + i] = LOW;
    }
}


