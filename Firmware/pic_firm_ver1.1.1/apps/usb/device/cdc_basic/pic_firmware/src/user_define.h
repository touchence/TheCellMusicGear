/* 
 * File:   user_define.h
 * Author: ymorishita
 *
 * Created on 2016/09/21, 12:35
 */

#ifndef USER_DEFINE_H
#define	USER_DEFINE_H

#define  MAX_CHNUM                      32		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		1		// Size of the input buffer per analog inputmain
#define  LED_UNIT                       128

#define CPU_CLOCK 7370000L // [Hz]
#define CPU_PLL 52
#define FCY ( CPU_CLOCK * CPU_PLL / 8)

#define HIGH    38
#define LOW     19

// #define ACT_CHECK

#define OLD_PCB 0

#define  NUM_CHS2SCAN			16		// Number of channels enabled for channel scan

#define RX_BUFFER_ADDRESS_TAG
#define TX_BUFFER_ADDRESS_TAG
#define MIDI_EVENT_ADDRESS_TAG

#define MODE_SEQUENCE   0
#define MODE_PRESSURE   1
#define MODE_PAD        2
#define MODE_MULTITOUCH 3
#define MODE_CC         4
#define MODE_DEBUG      5



#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* USER_DEFINE_H */

