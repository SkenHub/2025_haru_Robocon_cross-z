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

Encoder encoder[4];
Encoder_data e_data[4];
CanData can_data_r;
int16_t x = 0,y = 0;
double theta =  0.0;
uint8_t send_data_enc[8];
uint8_t send_data_limit[8];
Gpio limit[8];
double keisoku_k[4];
double tyok = 596.4;
double pai = 3.14159265;
double k_han = 143.165;
double keisoku[4];
uint8_t a,b,c,d,e,f;
double prev_deg[4] = {0, 0, 0, 0};
double init_deg[4] = {0, 0, 0, 0};
bool init_done = false;

void can(void){
	if(can_data_r.rx_stdid == 0x300){
	  a = can_data_r.rx_data[0];
	}
	else if(can_data_r.rx_stdid == 0x320){
      a = can_data_r.rx_data[0];
	}
	else if(can_data_r.rx_stdid == 0x114){
	  b = can_data_r.rx_data[0];
	}
}

void main_interrupt(void) {
    double wheel_disp[4];  // 各ホイールの移動距離
    double delta_deg[4];   // エンコーダの変化量

    // **エンコーダの値を取得**
    for (int i = 0; i < 4; ++i) {
        encoder[i].interrupt(&e_data[i]);
    }

    // **初回のみエンコーダの初期値を記録**
    if (!init_done) {
        for (int i = 0; i < 4; ++i) {
            init_deg[i] = e_data[i].deg;
        }
        init_done = true;
    }

    // **エンコーダの変化量を取得 (絶対値)**
    for (int i = 0; i < 4; ++i) {
        delta_deg[i] = e_data[i].deg - init_deg[i];  // 角度の変化量
        wheel_disp[i] = (tyok * M_PI * delta_deg[i]) / 360.0;  // 角度 → mm
    }

    // **ホイールのインデックス**
    double F = wheel_disp[0];  // 前ホイール
    double R = wheel_disp[1];  // 右ホイール
    double B = wheel_disp[2];  // 後ホイール
    double L = wheel_disp[3];  // 左ホイール

    // **ロボットの回転角 (theta) の計算**
    double delta_theta = ((R + L) - (F + B)) / (4.0 * k_han) * (180.0 / M_PI);
    theta += delta_theta;

    // **角度を -180° 〜 180° に制限**
    while (theta > 180) theta -= 360;
    while (theta < -180) theta += 360;

    // **並進速度の計算（ホイールの移動距離を考慮）**
    double vx = (F - B) / 2.0;  // 前後方向
    double vy = (R - L) / 2.0;  // 左右方向

    // **角度を考慮した座標更新**
    double rad_theta = theta * M_PI / 180.0;  // 度数法 → ラジアン変換
    x += vx * cos(rad_theta) - vy * sin(rad_theta);
    y += vx * sin(rad_theta) + vy * cos(rad_theta);

    // **送信データの更新**
    send_data_enc[0] = static_cast<int16_t>(x);
    send_data_enc[1] = static_cast<int16_t>(y);
    send_data_enc[2] = static_cast<int16_t>(theta);
}

int main(void)
{
	sken_system.init();
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	encoder[0].init(A0, A1, TIMER5);
	encoder[1].init(B3, A5, TIMER2); //invert
    encoder[2].init(B6, B7, TIMER4);
	encoder[3].init(C6, C7, TIMER8);
	limit[0].init(B15, INPUT_PULLUP);
	limit[1].init(B14, INPUT_PULLUP);
	limit[2].init(A11, INPUT_PULLUP);
	limit[3].init(A8, INPUT_PULLUP);
	limit[4].init(A7, INPUT_PULLUP);
	limit[5].init(A6, INPUT_PULLUP);
	limit[6].init(B9, INPUT_PULLUP);
	limit[7].init(B8, INPUT_PULLUP);
	sken_system.addTimerInterruptFunc(can,1,10);
	sken_system.addTimerInterruptFunc(main_interrupt,2,1);
	while (1) {
		sken_system.canTransmit(CAN_2,0x360,send_data_enc,8,1);
		sken_system.canTransmit(CAN_2,0x250,send_data_limit,8,1);
	}
}

