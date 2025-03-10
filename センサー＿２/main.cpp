#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "math.h"

Encoder encoder[4];
Encoder_data e_data[4];
Gpio limit[8];

double tyok = 60.0;    // ホイール直径
double k_han = 143.165; // マシン半径

double init_deg[4] = {0, 0, 0, 0}; // エンコーダの初期角度取得用
bool init_done = false;            // 初期値取得済みフラグ

// グローバルな自己位置推定変数
double x = 0, y = 0;      // ワールド座標での位置
double theta = 0.0;           // ロボットの向き（度）
double rad_theta = 0.0;       // ロボットの向き（ラジアン）

// エンコーダからの累積距離（各ホイール）
double F, R, B, L;           // Front, Right, Back, Left の各ホイール
// 前回の値（差分計算用）
double oF = 0.0, oR = 0.0, oB = 0.0, oL = 0.0;

// ロボットの局所座標系での移動量
double vx, vy;
// 角度変化量
double delta_theta; // 単位: 度

CanData can_data_r;
uint8_t a,b,c,d,e,f;
uint8_t send_data_enc[8];   // エンコーダ関連CAN送信データ
uint8_t send_data_limit[8]; // リミットスイッチのCAN送信データ

// タイマー割り込み関数（オドメトリ更新）
void main_interrupt(void) {
    // 各エンコーダの割り込み処理
    for (int i = 0; i < 4; ++i) {
        encoder[i].interrupt(&e_data[i]);
    }

    // 初回実行時はエンコーダの初期値を保存
    if (!init_done) {
        for (int i = 0; i < 4; ++i) {
            init_deg[i] = e_data[i].deg;
        }
        oF = e_data[0].distance;
        oR = e_data[1].distance;
        oB = e_data[2].distance;
        oL = e_data[3].distance;
        init_done = true;
        return; // 初回は位置更新を行わずに終了
    }

    // 現在の各ホイールの累積距離を取得
    F = e_data[0].distance;
    R = e_data[1].distance;
    B = e_data[2].distance;
    L = e_data[3].distance;

    // 各ホイールの今回の増分を計算
    double delta_F = F - oF;
    double delta_R = R - oR;
    double delta_B = B - oB;
    double delta_L = L - oL;

    // 角度変化量(delta_theta)の計算
    // 右側（R, L）の合計と前面・背面（F, B）の合計の差分から角度変化を求める
    // ※結果はラジアン単位なので、度に変換
    double delta_theta_rad = ((delta_R + delta_L) - (delta_F + delta_B)) / (4.0 * k_han);
    delta_theta = delta_theta_rad * (180.0 / M_PI);
    theta += delta_theta;  // 角度変化を累積

    // thetaを[-180, 180]の範囲に正規化
    while (theta > 180.0) theta -= 360.0;
    while (theta < -180.0) theta += 360.0;

    // ロボット座標系での移動量の計算
    // ここでは、FとBの差がX軸、RとLの差がY軸の移動に相当すると仮定
    vx = (delta_F - delta_B) / 2.0;
    vy = (delta_R - delta_L) / 2.0;

    // ワールド座標への変換：現在の向き(theta)を利用
    rad_theta = theta * M_PI / 180.0;
    x += vx * cos(rad_theta) - vy * sin(rad_theta);
    y += vx * sin(rad_theta) + vy * cos(rad_theta);

    // 次回計算用に現在値を保存
    oF = F;
    oR = R;
    oB = B;
    oL = L;

    // CAN送信用データ例：ここでは角度(theta)を送信（16ビット整数に変換）
    int16_t x_int = static_cast<int16_t>(x);
    send_data_enc[0] = (x_int >> 8) & 0xFF;  // x の上位バイト
    send_data_enc[1] = x_int & 0xFF;         // x の下位バイト
    int16_t y_int = static_cast<int16_t>(y);
    send_data_enc[2] = (y_int >> 8) & 0xFF;  // y の上位バイト
    send_data_enc[3] = y_int & 0xFF;         // y の下位バイト
    int16_t theta_int = static_cast<int16_t>(theta);
    send_data_enc[4] = (theta_int >> 8) & 0xFF;
    send_data_enc[5] = theta_int & 0xFF;

    // 各リミットスイッチの状態を読み取り
    for (int i = 0; i < 8; i++) {
        send_data_limit[i] = limit[i].read();
    }
}

void can(void){
	if(can_data_r.rx_stdid == 0x360){
	  a = can_data_r.rx_data[0];
	}
    if(can_data_r.rx_stdid == 0x250){
      b = can_data_r.rx_data[0];
	}
    if(can_data_r.rx_stdid == 0x320){
      c = can_data_r.rx_data[0];
	}
    if(can_data_r.rx_stdid == 0x114){
	  d = can_data_r.rx_data[0];
	}
	if(can_data_r.rx_stdid == 0x300){
	  e = can_data_r.rx_data[0];
	}
	sken_system.canTransmit(CAN_2, 0x360, send_data_enc, 8, 1);
	sken_system.canTransmit(CAN_2, 0x250, send_data_limit, 8, 1);
}

int main(void) {
    // システム初期化
    sken_system.init();
    sken_system.startCanCommunicate(B13, B12, CAN_2); // CAN通信開始
    sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN通信
    // エンコーダの初期化（各ピンとタイマーを指定）
    encoder[0].init(A0, A1, TIMER5,60.0);
    encoder[1].init(B3, A5, TIMER2,60.0);
    encoder[2].init(B6, B7, TIMER4,60.0);
    encoder[3].init(C6, C7, TIMER8,60.0);

    // リミットスイッチの初期化（各ピン、プルアップ設定）
    limit[0].init(B15, INPUT_PULLUP);
    limit[1].init(B14, INPUT_PULLUP);
    limit[2].init(A11, INPUT_PULLUP);
    limit[3].init(A8, INPUT_PULLUP);
    limit[4].init(A7, INPUT_PULLUP);
    limit[5].init(A6, INPUT_PULLUP);
    limit[6].init(B9, INPUT_PULLUP);
    limit[7].init(B8, INPUT_PULLUP);

    // タイマー割り込みに自己位置更新関数を登録
    // 例：タイマー2、1ms間隔でmain_interrupt()を実行
    sken_system.addTimerInterruptFunc(main_interrupt, 2, 1);
    sken_system.addTimerInterruptFunc(can, 3, 10);

    // メインループ：CAN通信でエンコーダ・リミットスイッチのデータを送信
    while (1) {
    }
}
