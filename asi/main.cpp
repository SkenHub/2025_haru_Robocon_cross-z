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
const float syarin_e = 186.3904;

Motor mtr[4];

Encoder encoder[4];
Encoder_data e_data[4];
Pid pid_control[4];
double target[4];
double now[4];
double out[4];
uint8_t a,b,c,d,e,f;

uint8_t candata_s[8] = {0,0,0,0,0,0,0,0};
uint8_t candata_r[8];

double asi_rps_on[8] = {out[0],out[1],out[2],out[3]};
float mtr_rps_off[4] = {0,0,0,0};

void func(void)
{
	encoder[0].interrupt(&e_data[0]);
	encoder[1].interrupt(&e_data[1]);
	encoder[2].interrupt(&e_data[2]);
	encoder[3].interrupt(&e_data[3]);
	for(int n=0;n<4;n++){
		now[n] = e_data[n].rps;
	}
	if(candata_r[2] == 0){
		target[0] = candata_r[0];
		target[2] = candata_r[0];
	    target[1] = candata_r[1];
		target[3] = candata_r[1];
	}
	else{
		float senk;
		senk = (2*3.141529*candata_r[3])/360;
		for(int w=0;w<4;w++){
			target[w] = senk;
		}
	}
	for(int c=0;c<4;c++){
		out[c] = pid_control[c].control(target[c],now[c],1);
	}
	mtr[0].write(out[0]);
	mtr[1].write(out[1]);
	mtr[2].write(out[2]);
	mtr[3].write(out[3]);
}

void can_send(void){
	/*x_count = (enc_data_r[0]+enc_data_r[2])/2/1000*syarin_e;
	y_count = (enc_data_r[1]+enc_data_r[3])/2/1000*syarin_e;
	if((enc_data_r[0]<0 && enc_data_r[2]>0)||(enc_data_r[0]>0 && enc_data_r[2]<0)){
	k_count = (enc_data_r[0]+enc_data_r[1]+enc_data_r[2]+enc_data_r[3])/4/1000*syarin_e/144.67;
	}
	x += x_count;
	y += y_count;
	k += k_count;*/
	//sken_system.canTransmit(CAN_2,0x400,candata_s,8,1);//CAN送信
}
void can_rceive(void){
	if(can_data_r.rx_stdid == 0x114){
	    candata_r[0] = can_data_r.rx_data[0];
		candata_r[1] = can_data_r.rx_data[1];
		candata_r[2] = can_data_r.rx_data[2];
	}
	else if(can_data_r.rx_stdid == 0x320){
	    a = can_data_r.rx_data[0];
	}
	else if(can_data_r.rx_stdid == 0x300){
		b = can_data_r.rx_data[0];
	}
}

int main(void)
{
	sken_system.init();
	encoder[0].init(A0,A1,TIMER5);
	encoder[1].init(B3,A5,TIMER2);
	encoder[2].init(B6,B7,TIMER4);
	encoder[3].init(C6,C7,TIMER3);
	mtr[0].init(Apin,B8,TIMER10,CH1);
	mtr[0].init(Bpin,B9,TIMER11,CH1);
	mtr[1].init(Apin,A6,TIMER13,CH1);
	mtr[1].init(Bpin,A7,TIMER14,CH1);
	mtr[2].init(Apin,A8,TIMER1,CH1);
	mtr[2].init(Bpin,A11,TIMER1,CH4);
	mtr[3].init(Apin,B14,TIMER12,CH1);
	mtr[3].init(Bpin,B15,TIMER12,CH2);
	pid_control[0].setGain(30,0,0.01);
	pid_control[1].setGain(25,0,0.01);
	pid_control[2].setGain(27,0,0.01);
	pid_control[3].setGain(30,0,0.01);
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
	sken_system.addTimerInterruptFunc(can_rceive,3,1);
	sken_system.addTimerInterruptFunc(func,1,1);
	//sken_system.addTimerInterruptFunc(can_send,2,1);
	while (1) {
		}
}
