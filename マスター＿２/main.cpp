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

Uart pc_serial;
uint8_t sri_data_r[6];
//uint8_t sri_data_s[6] = {0xA5,0xA5,0,1,2,3};
uint8_t sri_data_s[6] = {0,0,0,0,0,0};
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
uint8_t asi_rps_off[8] = {1,1,1,1,1,1,1,1};

double mtr_koon_d;
double mtr_bool_h;
double mtr_bool_l;
double kyori;
double mkhyo = 0;
Motor mtr[4];
double out;

CanData can_data_r;
//uint8_t candata_enc[8];
uint8_t a,b,c,d,e,f;

void can(void){
	if(can_data_r.rx_stdid == 0x360){
		sri_data_s[0] = can_data_r.rx_data[0];
		sri_data_s[1] = can_data_r.rx_data[1];
		sri_data_s[2] = can_data_r.rx_data[2];
		sri_data_s[3] = can_data_r.rx_data[3];
		sri_data_s[4] = can_data_r.rx_data[4];
		sri_data_s[5] = can_data_r.rx_data[5];
	}
    if(can_data_r.rx_stdid == 0x250){
	  for(int d=0;d<7;d++){
	  limit_data[d] = can_data_r.rx_data[d];
	  }
	}
    if(can_data_r.rx_stdid == 0x320){
      a = can_data_r.rx_data[0];
	}
    if(can_data_r.rx_stdid == 0x114){
	  b = can_data_r.rx_data[0];
	}
	if(can_data_r.rx_stdid == 0x300){
	  c = can_data_r.rx_data[0];
	}
	sken_system.canTransmit(CAN_2,0x300,mtr_rps_on,8,1);
	sken_system.canTransmit(CAN_2,0x114,asi_rps_off,8,1);
}


void main_interrupt(void){
	for (int i = 0; i < 4; ++i) {
		 encoder[i].interrupt(&e_data[i]);
	}
	//module_transmitter[SOLENOID_0].transmit(send_data_DD);
	mtr_koon_d = e_data[0].deg;
	mtr_bool_h = e_data[1].rps;
	mtr_bool_l = e_data[2].rps;
	out_bool[0] = pid_control.control(target,mtr_bool_h,1);
	out_bool[1] = pid_control.control(target,mtr_bool_l,1);
}

void hako_kaisyu(void){
	if(sirei_r == 1){
	    send_data_DD[0] = 1;
	    send_data_DD[1] = 1;
	    sri_data_s[3] = 1;
	    sirei_s = 1;
    }
	else if(sirei_r == 2){
	    send_data_DD[0] = 0;
	    send_data_DD[1] = 0;
	    sirei_s = 2;
	}
}
void koon_kaisyu(void){
	sirei_r = 3;
	if(sirei_r == 3){
		while(kyori > -75){
			for (int i = 0; i < 4; ++i) {
			    encoder[i].interrupt(&e_data[i]);
			}
			mtr_koon_d = e_data[0].deg;
			kyori = 80*mtr_koon_d/360;
		    mtr[0].write(20);
		}
		mtr[0].write(0);
		kyori = 80*mtr_koon_d/360;
		send_data_DD[5] = 0;
		while(limit_data[6] != 0){
		    mtr[0].write(-20);
		    if(can_data_r.rx_stdid == 0x250){
		    	for(int d=0;d<7;d++){
		    	    limit_data[d] = can_data_r.rx_data[d];
		        }
		    }
		}
		send_data_DD[3] = 1;
	 }
	 if(sirei_r == 4){
		 send_data_DD[3] = 0;
	     send_data_DD[4] = 1;
	     while(kyori > -75){
	    	 for (int i = 0; i < 4; ++i) {
	    	 	 encoder[i].interrupt(&e_data[i]);
	    	 }
	    	  mtr_koon_d = e_data[0].deg;
	    	 kyori = 80*mtr_koon_d/360;
	         mtr[0].write(20);
	     }
	     send_data_DD[5] = 1;
	     send_data_DD[3] = 1;
	     while(limit_data[0] == 1){
	         mtr[0].write(-30);
	     }
	     send_data_DD[4] = 0;
	     sirei_s = 4;
	  }
}

int main(void)
{
    sken_system.init();
    sken_module_init();
    pc_serial.init(C10,A11,SERIAL4,9600);
    pc_serial.startDmaRead(sri_data_r,6);
    sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
    sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
    encoder[0].init(A0, A1, TIMER5);
    encoder[1].init(B3, A5, TIMER2);
    encoder[2].init(B6, B7, TIMER4);
    encoder[3].init(C6, C7, TIMER8);
    mtr[0].init(Apin,B8,TIMER10,CH1);
	mtr[0].init(Bpin,B9,TIMER11,CH1);
	mtr[1].init(Apin,A6,TIMER13,CH1);
	mtr[1].init(Bpin,A7,TIMER14,CH1);
	mtr[2].init(Apin,A8,TIMER1,CH1);
	mtr[2].init(Bpin,A11,TIMER1,CH4);
	mtr[3].init(Apin,B14,TIMER12,CH1);
	mtr[3].init(Bpin,B15,TIMER12,CH2);
    SW.init(C13,INPUT_PULLUP);
    pid_control.setGain(1,0.1,0.01);
    pid_mtr_koon.setGain(1,0.1,0.01);
    pid_mtr_bool.setGain(1,0.1,0.01);
    sken_system.addTimerInterruptFunc(main_interrupt, 2, 1);
    sken_system.addTimerInterruptFunc(can,3,10);
    sken_system.addTimerInterruptFunc(koon_kaisyu,4,1000);
    sken_system.addTimerInterruptFunc(hako_kaisyu,5,500);
	while(1){
		sri_data_r[4] = pc_serial.read(1000);
		for(int c=0;c>3;c++){
		asi_rps[c] = sri_data_r[c];
		}
		//sirei_r = sri_data_r[3];
		sri_data_s[6] = sirei_s;
		pc_serial.write(sri_data_s,6);
        sken_module_receive();

     //ボール回収
       if(sirei_r == 5){
           mtr_rps_on[3] = 30;
           sirei_s = 4;
        }
     //ボール発射
       if(sirei_r == 6){
    	   send_data_DD[2] = 0;
       }
       else{
    	   send_data_DD[2] = 1;
       }
       if(sirei_r == 7){
    	   mtr[1].write(30);
    	   mtr[2].write(5);
       	   sirei_s = 5;
       }
       else if(sirei_r == 8){
    	   mtr[1].write(15);
    	   mtr[2].write(5);
    	   sirei_s = 6;
       }
       else if(sirei_r == 9){
    	   mtr_rps_on[3] = -5;
    	   sirei_s = 7;
       }
	}
}
