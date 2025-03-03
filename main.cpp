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
#include "sken_module.hpp"

Uart serial;
uint8_t sri_data_r[4];
uint8_t sri_data_s[4];
int sirei_r = 0;
int sirei_s = 0;

//uint8_t send_data_DD[8] = {1,1,1,1,1,1,1,1};
uint8_t send_data_DD[8] = {0,0,0,0,0,0,0,0};
int limit_data[7];
Gpio SW;
Encoder encoder[4];
Encoder_data e_data[4];
CanData can_data_enc;
uint8_t candata_enc;
int16_t x,y,theta;

Pid pid_control;
Pid pid_mtr_koon;
Pid pid_mtr_bool;
double target = 30.0;
double now[4];
uint8_t out_bool[8];

const uint8_t mtr_org[8] = {0,0,0,0,0,0,0,0};
uint8_t mtr_rps_on[8] = {30/*out_bool[0]*/,30/*out_bool[1]*/,5,0,0,0,0,0};
uint8_t mtr_rps_off[8] = {0,0,0,0,0,0,0,0};
uint8_t asi_rps[8] = {sri_data_r[0],sri_data_r[1],sri_data_r[2],0,0,0,0,0};
uint8_t asi_rps_off[8] = {1,0,0,0,0,0,0,0};

double mtr_koon_d;
double mtr_bool_h;
double mtr_bool_l;
double kyori;
Motor mtr;
double out;

void main_interrupt(void){
	module_transmitter[SOLENOID_0].transmit(send_data_DD);
	for(int a=0;a<7;a++){
	    if (module_receiver[SENSOR_0].get_limit(a)) {
            limit_data[a] = 1;
	    }
	    else limit_data[a] = 0;
	}
	module_receiver[SENSOR_0].get_enc(&x,&y,&theta);
	//sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_enc);
	encoder[0].interrupt(&e_data[0]);
	encoder[1].interrupt(&e_data[1]);
	encoder[2].interrupt(&e_data[2]);
	encoder[3].interrupt(&e_data[3]);
	mtr_koon_d = e_data[0].deg;
	mtr_bool_h = e_data[1].rps;
	mtr_bool_l = e_data[2].rps;
	out_bool[0] = pid_control.control(target,mtr_bool_h,1);
	out_bool[1] = pid_control.control(target,mtr_bool_l,1);
	sri_data_s[0] = x;
	sri_data_s[1] = y;
	sri_data_s[2] = theta;
}

void hk_kaiyu(void){
	if(sirei_r == 1){
	    send_data_DD[0] = 1;
	    HAL_Delay(500);
	    send_data_DD[1] = 1;
	    sri_data_s[3] = 1;
	    sirei_s = 1;
	}
	else{
		send_data_DD[0] = 0;
		send_data_DD[1] = 0;
	}
}

void koon_kaisyu(void){
	if(sirei_r == 3){
    double mkhyo = 250;
    kyori = 80*mtr_koon_d/360;
    while(limit_data[0] == 1){
    	mtr.write(30);
    }
    HAL_Delay(500);
    send_data_DD[3] = 1;
    HAL_Delay(500);
    send_data_DD[4] = 1;
    HAL_Delay(1000);
    while(kyori == 250){
        out = pid_mtr_koon.control(mkhyo,kyori,1);
        mtr.write(out);
    }
    sirei_s = 3;
	}
}

void bool_kaisyu(void){
    mtr_rps_on[3] = 30;
}
void bool_hasya(void){
	mtr_rps_on[1] = 30;
	mtr_rps_on[2] = 30;
	mtr_rps_on[3] = 5;
}

int main(void)
{
    sken_system.init();
    sken_module_init();
    sken_system.init();
    serial.init(A9,A10,SERIAL1,9600);
    sken_system.startCanCommunicate(B13,B12,CAN_2);
    encoder[0].init(A0,A1,TIMER2);
    encoder[1].init(B3,A5,TIMER3);
    encoder[2].init(B6,B7,TIMER4);
    encoder[3].init(C6,C7,TIMER5);
    mtr.init(Apin,B8,TIMER1,CH1);
    mtr.init(Bpin,B9,TIMER1,CH1);
    SW.init(C13,INPUT_PULLUP);
    pid_control.setGain(1,0.1,0.01);
    pid_mtr_koon.setGain(1,0.1,0.01);
    pid_mtr_bool.setGain(1,0.1,0.01);
    sken_system.addTimerInterruptFunc(0,1);
    sken_system.addTimerInterruptFunc(main_interrupt, 2, 1);
	while(1){
		sri_data_r[4] = serial.read(1000);
		for(int c=0;c>3;c++){
		asi_rps[c] = sri_data_r[c];
		}
		sirei_r = sri_data_r[3];
		sri_data_s[3] = sirei_s;
		serial.write(sri_data_s,4);
        sken_module_receive();
        sken_system.canTransmit(CAN_2,0x300,mtr_rps_on,8,1);
        sken_system.canTransmit(CAN_2,0x114,asi_rps_off,8,1);

	}
}
