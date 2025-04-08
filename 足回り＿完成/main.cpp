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
#include "math.h"

CanData can_data_r;
const float syarin_e = 186.3904;

Motor mtr[4];

Encoder encoder[4];
Encoder_data e_data[4];
Pid pid_control[4];
double pai = 3.14159265;
double syatai_hk = 184.763;
double target[4];
double now[4];
double out[4];
double VX,VY,deg_s;
uint8_t a[6],b[6],c[6],d[6],e[6],f[6],g[6];

uint8_t candata_r[6];

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
	/*target[0] = (VX + syarin_e * deg_s) / (2 * pai * syatai_hk);
	target[1] = (VY + syarin_e * deg_s) / (2 * pai * syatai_hk);
	target[2] = (VX + syarin_e * deg_s) / (2 * pai * syatai_hk);
	target[3] = (VY + syarin_e * deg_s) / (2 * pai * syatai_hk);*/
	target[0] = (VX + VY + deg_s)/100;
	target[1] = (VX + -VY + deg_s)/100;
	target[2] = (-VX + -VY + deg_s)/100;
	target[3] = (-VX + VY + deg_s)/100;
	for(int c=0;c<4;c++){
		out[c] = pid_control[c].control(target[c],now[c],1);
	}
	mtr[0].write(out[0]);
	mtr[1].write(out[1]);
	mtr[2].write(out[2]);
	mtr[3].write(out[3]);
}

void can_rceive(void){
	if(can_data_r.rx_stdid == 0x160){
	    candata_r[0] = can_data_r.rx_data[0];
		candata_r[1] = can_data_r.rx_data[1];
		candata_r[2] = can_data_r.rx_data[2];
		candata_r[3] = can_data_r.rx_data[3];
		candata_r[4] = can_data_r.rx_data[4];
		candata_r[5] = can_data_r.rx_data[5];
	}
	/*else if(can_data_r.rx_stdid == 0x320){
	    a = can_data_r.rx_data[0];
	}*/
	if(can_data_r.rx_stdid == 0x300){
		for(int m=0;m<6;m++){
		    b[m] = can_data_r.rx_data[m];
	    }
	}
	/*else if(can_data_r.rx_stdid == 0x360){
		for(int j=0;j<6;j++){
		    c[j] = can_data_r.rx_data[j];
		}
	}*/
	/*else if(can_data_r.rx_stdid == 0x250){
	    d = can_data_r.rx_data[0];
	}*/
	/*if(can_data_r.rx_stdid == 0x114){
		for(int l=0;l<6;l++){
		    f[l] = can_data_r.rx_data[l];
	    }
	}*/
    if(can_data_r.rx_stdid == 0x161){
	   g[0] = can_data_r.rx_data[0];
	}
    VX/*speed*/ = (double)(int16_t(candata_r[0] << 8 | candata_r[1]));
    VY/*deg*/ = (double)(int16_t(candata_r[2] << 8 | candata_r[3]));
    deg_s = (double)(int16_t(candata_r[4] << 8 | candata_r[5]));
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
	pid_control[0].setGain(50,0.001,0.0001);
	pid_control[1].setGain(50,0.001,0.0001);
	pid_control[2].setGain(50,0.001,0.0001);
	pid_control[3].setGain(50,0.001,0.0001);
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
	sken_system.addTimerInterruptFunc(can_rceive,3,1);
	sken_system.addTimerInterruptFunc(func,1,1);
	while (1) {
		}
}
