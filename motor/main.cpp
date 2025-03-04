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

CanData can_data_r;

Motor mtr[4];
uint8_t cancan;

uint8_t candata_r[8];
uint8_t a,b,c,d,e,f;

void can_rceive(void){
	if(can_data_r.rx_stdid == 0x300){
	  candata_r[0] = can_data_r.rx_data[0];
	  candata_r[1] = can_data_r.rx_data[1];
	  candata_r[2] = can_data_r.rx_data[2];
	  candata_r[3] = can_data_r.rx_data[3];
	  candata_r[4] = can_data_r.rx_data[4];
	  candata_r[5] = can_data_r.rx_data[5];
	  candata_r[6] = can_data_r.rx_data[6];
	  candata_r[7] = can_data_r.rx_data[7];
	}
	else if(can_data_r.rx_stdid == 0x320){
      a = can_data_r.rx_data[0];
	}
	else if(can_data_r.rx_stdid == 0x114){
	  b = can_data_r.rx_data[0];
	}
}
int main(void)
{
	sken_system.init();
	mtr[0].init(Apin,B8,TIMER10,CH1);
	mtr[0].init(Bpin,B9,TIMER11,CH1);
	mtr[1].init(Apin,A6,TIMER13,CH1);
	mtr[1].init(Bpin,A7,TIMER14,CH1);
	mtr[2].init(Apin,A8,TIMER1,CH1);
	mtr[2].init(Bpin,A11,TIMER1,CH4);
	mtr[3].init(Apin,B14,TIMER12,CH1);
	mtr[3].init(Bpin,B15,TIMER12,CH2);
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
	sken_system.addTimerInterruptFunc(can_rceive,3,1);
	while (1) {
		for(int m=0;m<4;m++)
			mtr[m].write(candata_r[m]);
	 cancan = candata_r[1];
	}
}
