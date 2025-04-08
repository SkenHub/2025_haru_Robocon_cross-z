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

//Uart pc_serial;
uint8_t sri_data_r[6] = {0,0,0,0,0,0};
//uint8_t sri_data_s[6] = {5,5,5,5,5,5};
uint8_t sri_data_s[6] = {0,0,0,0,0,0};
int sirei_r = 0;
uint8_t sirei_s = 0;

//uint8_t send_data_DD[8] = {1,1,1,1,1,1,1,1};
uint8_t send_data_DD[6] = {0,0,0,0,0,0};
Gpio SW;
Gpio LED;
Encoder encoder[4];
Encoder_data e_data[4];
CanData can_data_enc;
uint8_t candata_enc;
int16_t x,y,theta;
uint8_t kanryo[1] = {0};

//Pid pid_control;
//Pid pid_mtr_koon;
//Pid pid_mtr_bool;
//double target = 30.0;
//double now[4];
//uint8_t out_bool[8];
int aizu[26] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int denzi_k[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int hako_k[8] = {0,0,0,0,0,0,0,0};

//const uint8_t mtr_org[8] = {0,0,0,0,0,0,0,0};
//uint8_t mtr_rps_on[8] = {30/*out_bool[0]*/,30/*out_bool[1]*/,5,0,0,0,0,0};
//uint8_t mtr_rps_off[8] = {0,0,0,0,0,0,0,0};
//uint8_t asi_rps[6] = {0,0,0,0,0,0};
uint8_t asi_rps[6] = {5,5,5,5,5,5};
//uint8_t asi_rps_off[8] = {1,1,1,1,1,1,1,1};

double mtr_koon_d;
double mtr_bool_h;
double mtr_bool_l;
double kyori;
double mkhyo = 0;
Motor mtr[4];
double out;
int limit_data;

CanData can_data_r;
//uint8_t candata_enc[8];
uint8_t a,b,c,d,e,f,g;

void can(void){
	/*if(can_data_r.rx_stdid == 0x360){
		sri_data_s[0] = can_data_r.rx_data[0];
		sri_data_s[1] = can_data_r.rx_data[1];
		sri_data_s[2] = can_data_r.rx_data[2];
		sri_data_s[3] = can_data_r.rx_data[3];
		sri_data_s[4] = can_data_r.rx_data[4];
		sri_data_s[5] = can_data_r.rx_data[5];
	}*/
	if(can_data_r.rx_stdid == 0x161){
		sirei_r = can_data_r.rx_data[0];
	}
    /*if(can_data_r.rx_stdid == 0x250){
	  for(int d=0;d<7;d++){
	  limit_data[d] = can_data_r.rx_data[d];
	  }
      g = can_data_r.rx_data[0];
	}*/
    if(can_data_r.rx_stdid == 0x160){
        for(int m=0;m<6;m++){
          sri_data_r[m] = can_data_r.rx_data[m];
        }
    }
    /*if(can_data_r.rx_stdid == 0x320){
      a = can_data_r.rx_data[0];
	}*/
    /*if(can_data_r.rx_stdid == 0x114){
	  b = can_data_r.rx_data[0];
	}
	if(can_data_r.rx_stdid == 0x300){
	  c = can_data_r.rx_data[0];
	}*/
	/*sken_system.canTransmit(CAN_2,0x300,send_data_DD,6,1);
	//sken_system.canTransmit(CAN_2,0x114,asi_rps,6,1);
	sken_system.canTransmit(CAN_2,0x150,sri_data_s,6,1);
	sken_system.canTransmit(CAN_2,0x151,kanryo,1,1);*/
}


void main_interrupt(void){
	for (int i = 0; i < 4; ++i) {
		 encoder[i].interrupt(&e_data[i]);
	}
	//module_transmitter[SOLENOID_0].transmit(send_data_DD);
	mtr_koon_d = e_data[0].deg;
	kyori = 80*mtr_koon_d/360;
}

void hako_kaisyu(void){
	if(hako_k[0] == 1 && aizu[1] == 0){
		send_data_DD[0] = 1;
		aizu[1] = 1;
	}
	else if(hako_k[0] == 1 && aizu[2] == 0){
		send_data_DD[1] = 1;
		aizu[2] = 1;
	}
	else if(hako_k[0] == 1 && aizu[3] == 0){
		send_data_DD[0] = 0;
		aizu[3] = 1;
	}
	else if(hako_k[0] == 1 && aizu[4] == 0){
		send_data_DD[1] = 0;
		aizu[4] = 1;
		sirei_s = 2;
	}
	if(hako_k[1] == 1 && aizu[12] == 0){
		send_data_DD[1] = 1;
		aizu[12] = 1;
	}
	else if(hako_k[1] == 1 && aizu[13] == 0){
		send_data_DD[0] = 1;
		aizu[13] = 1;
	}
	else if(hako_k[1] == 1 && aizu[14] == 0){
		send_data_DD[1] = 0;
		aizu[14] = 1;
	}
	else if(hako_k[1] == 1 && aizu[15] == 0){
		send_data_DD[0] = 0;
		aizu[15] = 1;
		sirei_s = 7;
	}
}
void koon_kaisyu(void){
	if(denzi_k[0] == 1 && aizu[7] ==  0){
		send_data_DD[3] = 0;
		denzi_k[1] = 1;
		aizu[7] = 1;
	}
	else if(denzi_k[1] == 1 && aizu[8] ==  0){
		send_data_DD[4] = 0;
		denzi_k[2] = 1;
		aizu[8] = 1;
	}
	else if(denzi_k[3] == 1 && aizu[9] ==  0){
		send_data_DD[5] = 0;
		denzi_k[4] = 1;
		aizu[9] = 1;
	}
	else if(denzi_k[4] == 1 && aizu[10] ==  0){
		send_data_DD[3] = 1;
		aizu[6] = 2;
	}
	if(denzi_k[6] == 1 && aizu[19] == 0){
		send_data_DD[4] = 0;
		denzi_k[7] = 1;
		aizu[19] = 1;
	}
	else if(denzi_k[8] == 1 && aizu[16] == 0){
		send_data_DD[3] = 0;
		denzi_k[9] = 1;
		aizu[16] = 1;
	}
	else if(denzi_k[9] == 1 && aizu[25] == 0){
		send_data_DD[5] = 1;
		denzi_k[10] = 1;
		aizu[25] = 1;
	}
	else if(denzi_k[10] == 1 && aizu[22] == 0){
		send_data_DD[4] = 1;
		denzi_k[11] = 1;
		aizu[22] = 1;
	}
	else if(denzi_k[11] == 1 && aizu[24] == 0){
		send_data_DD[3] = 1;
		aizu[24] = 1;
		aizu[23] = 1;
	}

}

int main(void)
{
    sken_system.init();
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
    SW.init(C11,INPUT_PULLUP);
    LED.init(A12,OUTPUT);
    /*pid_control.setGain(1,0.1,0.01);
    pid_mtr_koon.setGain(1,0.1,0.01);
    pid_mtr_bool.setGain(1,0.1,0.01);*/
    sken_system.addTimerInterruptFunc(main_interrupt, 1, 1);
    sken_system.addTimerInterruptFunc(can,3,1);
    sken_system.addTimerInterruptFunc(koon_kaisyu,6,700);
    sken_system.addTimerInterruptFunc(hako_kaisyu,7,700);
	while(1){
		sken_system.canTransmit(CAN_2,0x300,send_data_DD,6,1);
		//sken_system.canTransmit(CAN_2,0x114,asi_rps,6,1);
		//sken_system.canTransmit(CAN_2,0x150,sri_data_s,6,1);
		sken_system.canTransmit(CAN_2,0x151,kanryo,1,1);
		//static int limit_data;
		//(!SW.read())? limit_data=1 : limit_data=0;
		limit_data = SW.read();
		for(int c=0;c>6;c++){
		asi_rps[c] = sri_data_r[c];
		}
		//sirei_r = sri_data_r[2];
		kanryo[0] = sirei_s;
	 //初期展開
		//sirei_r = 1;
		mtr_koon_d = e_data[0].deg;
		kyori = 80*mtr_koon_d/360;
		if(sirei_r == 1 && aizu[0] != 3){
			if(kyori < -77){
				aizu[0] = 1;
			}
		    if(aizu[0] == 1){
				mtr[0].write(0);
				aizu[0] = 2;
			}
		    if(aizu[0] == 2){
				mtr[0].write(-20);
			}
			if(aizu[0] == 0){
				mtr[0].write(20);
			}
			else if(limit_data == 1){
				mtr[0].write(0);
				send_data_DD[3] = 1;
				send_data_DD[4] = 1;
				send_data_DD[5] = 1;
				sirei_s = 1;
				aizu[0] = 3;
			}
		}
		//箱回収
		if(sirei_r == 2 && aizu[1] == 0){
			hako_k[0] = 1;
		}
       //ボール回収
       if(sirei_r == 3){
    	   mtr[1].write(40);
    	   send_data_DD[2] = 1;
           sirei_s = 3;
        }
       else{
    	   mtr[1].write(0);
       }
       //ボール発射
       if(sirei_r == 4){
    	   mtr[2].write(50);
    	   mtr[3].write(27);
       	   sirei_s = 4;
       }
       else if(sirei_r == 5){
    	   send_data_DD[2] = 0;
    	   mtr[2].write(15);
    	   mtr[3].write(27);
    	   sirei_s = 5;
       }
       else{
    	   mtr[2].write(0);
    	   mtr[3].write(0);
       }
       //コーン回収
       	if(sirei_r == 6){
       		if(sirei_r == 6 && aizu[5] == 0){
       			denzi_k[0] = 1;//電磁弁３，４をOFFにする
       			if(kyori < -35 && aizu[14] == 0){
       			    send_data_DD[5] = 1;//距離がー３５以下かつ一回目なら電磁弁５をOFF
       			    aizu[14] = 1;
       			}
       			if(kyori < -75 && aizu[11] == 0){
       			    aizu[6] = 1;
       			    aizu[11] = 1;
       		    }
       			if(aizu[6] == 1){//距離がー７５以下になったら、モーター停止後電磁弁５，３をON
       			    mtr[0].write(0);
       			    denzi_k[3] = 1;
       			}
       			else if(aizu[6] == 2){
       			    mtr[0].write(-25);//電磁弁５，３がONになった後、モーターを負回転
       			}
       			if(aizu[6] == 0 && denzi_k[2] == 1){
       			    mtr[0].write(25);//今回の動作が一回目かつ電磁弁３，４がOFFになったならモーターを正回転
                }
       			if(limit_data == 1 && aizu[6] == 2){//リミットがONかつ電磁弁５，３がONになったら、モーター停止＆電磁弁４をＯＮ
       			    mtr[0].write(0);
       			    send_data_DD[4] = 1;
       			    aizu[6] = 3;
       			}
       			if(sirei_r == 3 && aizu[6] == 3){
       			    aizu[5] = 1;
       			    sirei_s = 6;
       			}
       	    }
        }
       	//箱リリース
        if(sirei_r == 7 && aizu[12] == 0){
        	hako_k[1] = 1;
       	}
        //コーンリリース
        if(sirei_r == 8 && aizu[16] == 0){
        	denzi_k[6] = 1;
        	if(aizu[17] == 0 && denzi_k[7] == 1){
        	    mtr[0].write(25);
            }
        	if(kyori < -75 && aizu[18] == 0){
        	    aizu[17] = 1;
        	    aizu[18] = 1;
        	}
        	if(aizu[17] == 1){
        	    mtr[0].write(0);
        	    denzi_k[8] = 1;
        	}
        	if(sirei_r == 8 && aizu[23] == 1){
        	    aizu[16] = 1;
        	    sirei_s = 8;
        	}
        }
        //LED点灯
        if(sirei_r == 10){
        	LED.write(HIGH);
        	sirei_s = 10;
        }
        else{
        	LED.write(LOW);
        }
	}
}

/*過去の遺物
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
if(sirei_r == 3 || sirei_r == 4 || sirei_r == 5){
				 if((sirei_r == 3 && aizu[6] == 0)||(sirei_r == 4 && aizu[7] == 0)||(sirei_r == 5 && aizu[8] == 0) ){
					 denzi_k[0] = 1;//電磁弁３，４をOFFにする
			         if(kyori > -35 && aizu[9] == 0){
			    	     send_data_DD[5] = 0;//距離がー３５以下かつ一回目なら電磁弁５をOFF
			    	     aizu[9] = 1;
			         }
			         if(kyori > -75){
			     	     aizu[5] = 1;
			         }
			         if(aizu[5] == 1){//距離がー７５以下になったら、モーター停止後電磁弁５，３をON
			     	     mtr[0].write(0);
			     	     denzi_k[3] = 1;
			         }
			         else if(aizu[5] == 2){
			     	     mtr[0].write(-20);//電磁弁５，３がONになった後、モーターを負回転
			         }
			         if(aizu[5] == 0 && denzi_k[2] == 1){
			         	 mtr[0].write(20);//今回の動作が一回目かつ電磁弁３，４がOFFになったならモーターを正回転

			         }
			         if(limit_data[6] == 0){//リミットが押された後、モーター停止＆電磁弁４をＯＮ
			     	     mtr[0].write(0);
			     	     send_data_DD[4] = 1;
			     	     aizu[5] = 3;
			     	     if(sirei_r == 3){
			     	   	     aizu[6] = 1;
			     	   	     sirei_s = 4;
			     	   	 }
			     	   	 else if(sirei_r == 4){
			     	   	     aizu[7] = 1;
			     	   	     sirei_s = 5;
			     	   	 }
			     	   	 else if(sirei_r == 5){
			     	   	     aizu[8] = 1;
			     	   	     sirei_s = 6;
			     	   	 }
			     	     aizu[3] = 0;
			     	     denzi_k[3] = 0;
			     	     denzi_k[4] = 0;

			         }
				 }
			  }
       //ボール回収
       if(sirei_r == 6){
    	   mtr[1].write(30);
           sirei_s = 7;
        }
       //ボール発射
       if(sirei_r == 7){
    	   send_data_DD[2] = 1;
    	   mtr[2].write(50);
    	   mtr[3].write(30);
       	   sirei_s = 5;
       }
       else if(sirei_r == 8){
    	   send_data_DD[2] = 0;
    	   mtr[2].write(25);
    	   mtr[3].write(30);
    	   sirei_s = 6;
       }
       else if(sirei_r == 11){
    	   mtr_rps_on[3] = -5;
    	   sirei_s = 7;
       }
	}
}
 */
