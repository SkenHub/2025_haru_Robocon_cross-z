/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

enum {
	SOLENOID_0=0x320,SOLENOID_1,SOLENOID_2,SOLENOID_3,SOLENOID_4,SOLENOID_5,SOLENOID_6,SOLENOID_7,SOLENOID_8,SOLENOID_9,SOLENOID_A,SOLENOID_B,SOLENOID_C,SOLENOID_D,SOLENOID_E,SOLENOID_F
};


/* Setting */
/* Module ID
 *  SOLENOID_0 ~ SOLENOID_F  */
constexpr uint32_t CAN_STDID = SOLENOID_0;
/* Pin is from the left */

/* ------- */


CanData received_raw;
uint8_t received_data[8];
Gpio solenoid[6];

void main_interrupt(void) {
	for (int i = 0; i < 6; ++i) {
		solenoid[i].write((received_data[i]==0x01 ? HIGH : LOW));
	}
}

int main(void)
{
	sken_system.init();
	solenoid[0].init(B6, OUTPUT);
	solenoid[1].init(B7, OUTPUT);
	solenoid[2].init(B8, OUTPUT);
	solenoid[3].init(B9, OUTPUT);
	solenoid[4].init(C6, OUTPUT);
	solenoid[5].init(C7, OUTPUT);

	sken_system.startCanCommunicate(B13, B12, CAN_2);
	sken_system.addCanRceiveInterruptFunc(CAN_2, &received_raw);
	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);

	while(true) {
		for (int i = 0; i < 8; ++i) {
			if (received_raw.rx_stdid == CAN_STDID) {
				received_data[i] = received_raw.rx_data[i];
			}
		}
	}
}
