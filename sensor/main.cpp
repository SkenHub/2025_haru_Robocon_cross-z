#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "math.h"

Encoder encoder[4];
Encoder_data e_data[4];
Gpio limit[8];

CanData can_data_r;

int16_t x = 0,y = 0;//マシン座標(x,y)
double theta =  0.0;//マシン角度(θ)

uint8_t send_data_enc[8];//エンコ送信データ
uint8_t send_data_limit[8];//リミット送信データ

double tyok = 596.4;//ホイール直径
double k_han = 143.165;//マシン半径

double keisoku_k[4];//計測値_k？？？
double keisoku[4];//計測値？？？

uint8_t a,b,c,d,e,f;//CAN受信データ用

double prev_deg[4] = {0, 0, 0, 0};//前回のエンコーダ値
double init_deg[4] = {0, 0, 0, 0};//エンコーダの初期角度

bool init_done = false;//エンコーダの初期値を取得したかのフラグ

//CAN受信処理
void can(void) {
    switch (can_data_r.rx_stdid) {
        case 0x300:
        case 0x320:
            a = can_data_r.rx_data[0];
            break;
        case 0x114:
            b = can_data_r.rx_data[0];
            break;
    }
}



double wheel_disp[4];  // 各ホイールの移動距離
double delta_deg[4];   // エンコーダの変化量
double F,B,L,R;
double oF,oB,oL,oR = 0;
double delta_theta;
double vx;
double vy;
double rad_theta;

double prev_theta;

//メインの割り込み
void main_interrupt(void) {

    //double wheel_disp[4];  // 各ホイールの移動距離
    //double delta_deg[4];   // エンコーダの変化量

    //各エンコーダの最新値を取得
    for (int i = 0; i < 4; ++i) {
        encoder[i].interrupt(&e_data[i]);
    }

    //初回のみエンコーダの初期値を記録（基準角度）
    if (!init_done) {
        for (int i = 0; i < 4; ++i) {
            init_deg[i] = e_data[i].deg;
        }
        init_done = true;//フラグ立て
    }

    //エンコーダの変化量を取得 (絶対値)
    for (int i = 0; i < 4; ++i) {
        delta_deg[i] = e_data[i].deg - init_deg[i];  // 初期角度からの変化量
        wheel_disp[i] = (tyok * M_PI * delta_deg[i]) / 360.0;  //角度から円弧を算出（角度→mm）
    }

    // **ホイールのインデックス**
    //double F = wheel_disp[0];  // 前ホイール	  F
    //double R = wheel_disp[1];  // 右ホイール	L   R
    //double B = wheel_disp[2];  // 後ホイール	  B
    //double L = wheel_disp[3];  // 左ホイール
    F = wheel_disp[0];  // 前ホイール	  F0
    R = wheel_disp[1];  // 右ホイール	L3  R1
    B = wheel_disp[2];  // 後ホイール	  B2
    L = wheel_disp[3];  // 左ホイール

    //ロボットの回転角 (theta) の計算
    //double delta_theta = ((R + L) - (F + B)) / (4.0 * k_han);//回転角の変化量

    delta_theta = (((R-oR) + (L-oL)) - ((F-oF) + (B-oB))) / (4.0 * k_han);//回転角の変化量
    theta += delta_theta;//累積回転角///////////////////////ここだぁ！！！！（おそらく増えつづけるOR減り続けるOR振動の原因）
    vx = (F-oF) - (B-oB) / 2.0;	// 前後方向
    vy = (R-oR) - (L-oL) / 2.0;	// 左右方向
    oF=F;
    oB=B;
    oL=L;
    oR=R;

    // **角度を -180° 〜 180° に制限**
    while (theta > 180) theta -= 360;
    while (theta < -180) theta += 360;

    // **並進距離の計算（ホイールの移動距離を考慮）**
    //double vx = (F - B) / 2.0;	// 前後方向
    //double vy = (R - L) / 2.0;	// 左右方向


    //角度を考慮した座標更新
    //double rad_theta = theta * M_PI / 180.0;	//度数法 → ラジアン変換
    rad_theta = theta * M_PI / 180.0;
    x += vx * cos(rad_theta) - vy * sin(rad_theta);
    y += vx * sin(rad_theta) + vy * cos(rad_theta);

    // **送信データの更新**
    send_data_enc[0] = (x >> 8) & 0xFF;  // x の上位バイト
    send_data_enc[1] = x & 0xFF;         // x の下位バイト
    send_data_enc[2] = (y >> 8) & 0xFF;  // y の上位バイト
    send_data_enc[3] = y & 0xFF;         // y の下位バイト
    int16_t theta_int = static_cast<int16_t>(theta);
    send_data_enc[4] = (theta_int >> 8) & 0xFF;
    send_data_enc[5] = theta_int & 0xFF;

    for (int i = 0; i < 8; i++) {
        send_data_limit[i] = limit[i].read();  // GPIOの状態を送信データに格納
    }
}

int main(void)
{
	sken_system.init();
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	encoder[0].init(A0, A1, TIMER5);
	encoder[1].init(B3, A5, TIMER2);
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
/*
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "math.h"

Encoder encoder[4];
Encoder_data e_data[4];
double deg[4];

int main(void)
{
	sken_system.init();

	encoder[0].init(A0,A1,TIMER5);
	encoder[1].init(B3,A5,TIMER2);
	encoder[2].init(B6,B7,TIMER4);
	encoder[3].init(C6,C7,TIMER3);
	encoder[0].reset();
	encoder[1].reset();
	encoder[2].reset();
	encoder[3].reset();
	while(1)
	{
		encoder[0].interrupt(&e_data[0]);
		encoder[1].interrupt(&e_data[1]);
		encoder[2].interrupt(&e_data[2]);
		encoder[3].interrupt(&e_data[3]);
		deg[0] = e_data[0].deg;
		deg[1] = e_data[1].deg;
		deg[2] = e_data[2].deg;
		deg[3] = e_data[3].deg;
	}
}
*/
