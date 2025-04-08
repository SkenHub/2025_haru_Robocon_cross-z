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

Gpio solenoid[6];

uint8_t candata_r[6];
uint8_t a[6],b[6],c[6],d[6],e[6],f[6],g[6];

void can_rceive(void){
	if(can_data_r.rx_stdid == 0x300){
	  /*candata_r[0] = can_data_r.rx_data[0];
	  candata_r[1] = can_data_r.rx_data[1];
	  candata_r[2] = can_data_r.rx_data[2];
	  candata_r[3] = can_data_r.rx_data[3];
	  candata_r[4] = can_data_r.rx_data[4];
	  candata_r[5] = can_data_r.rx_data[5];*/
	  for(int j=0;j<10;j++){
		for(int m=0;m<6;m++){
	  	  candata_r[m] = can_data_r.rx_data[m];
	    }
	  }
	}
	/*if(can_data_r.rx_stdid == 0x320){
      a = can_data_r.rx_data[0];
	}*/
	/*if(can_data_r.rx_stdid == 0x114){
	    for(int m=0;m<6;m++){
		b[m] = can_data_r.rx_data[m];
	    }
	}*/
	/*if(can_data_r.rx_stdid == 0x360){
		for(int j=0;j<6;j++){
			c[j] = can_data_r.rx_data[j];
	    }
	}*/
	/*if(can_data_r.rx_stdid == 0x250){
	  d = can_data_r.rx_data[0];
	}*/
	if(can_data_r.rx_stdid == 0x160){
		for(int l=0;l<6;l++){
			f[l] = can_data_r.rx_data[l];
	    }
	}
    if(can_data_r.rx_stdid == 0x161){
    	g[0] = can_data_r.rx_data[0];
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
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
	sken_system.addTimerInterruptFunc(can_rceive,3,1);
	while (1) {
		for (int i = 0; i < 6; ++i) {
			if(candata_r[i] == 1){
			    solenoid[i].write(HIGH);
			}
			else{
				solenoid[i].write(LOW);
			}
		}
	}
}
