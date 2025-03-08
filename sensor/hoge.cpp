#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "math.h"

Encoder encoder[4];
Encoder_data e_data[4];
Gpio limit[8];

double tyok = 59.64;    // ホイール直径
double k_han = 143.165; // マシン半径

double prev_deg[4] = {0, 0, 0, 0}; // 前回のエンコーダ値
double init_deg[4] = {0, 0, 0, 0}; // エンコーダの初期角度

bool init_done = false; // エンコーダの初期値を取得したかのフラグ

double x = 0, y = 0;   // マシン座標(x, y)
double theta = 0.0;    // マシン角度(θ)
double wheel_disp[4];  // 各ホイールの移動距離
double delta_deg[4];   // エンコーダの変化量
double F, B, L, R;
double oF, oB, oL, oR = 0;
double delta_theta;
double vx;
double vy;
double rad_theta;

uint8_t send_data_enc[8];   // エンコ送信データ
uint8_t send_data_limit[8]; // リミット送信データ

// メインの割り込み
void main_interrupt(void) {
    for (int i = 0; i < 4; ++i) {
        encoder[i].interrupt(&e_data[i]);
    }

    if (!init_done) {
        for (int i = 0; i < 4; ++i) {
            init_deg[i] = e_data[i].deg;
        }
        init_done = true;
    }

    for (int i = 0; i < 4; ++i) {
        delta_deg[i] = e_data[i].deg - init_deg[i];
        wheel_disp[i] = e_data[i].distance; // distance を使用
    }

    F = wheel_disp[0];
    R = wheel_disp[1];
    B = wheel_disp[2];
    L = wheel_disp[3];

    delta_theta = (((R - wheel_disp[1]) + (L - wheel_disp[3])) - ((F - wheel_disp[0]) + (B - wheel_disp[2]))) / (4.0 * k_han);
    theta = delta_theta; // +=ではなく、初期状態からの変位で計算
    vx = (F - oF - (B - oB)) / 2.0;
    vy = (R - oR - (L - oL)) / 2.0;
    oF = F;
    oB = B;
    oL = L;
    oR = R;

    while (theta > 180) theta -= 360;
    while (theta < -180) theta += 360;

    rad_theta = theta * M_PI / 180.0;
    x += vx * cos(rad_theta) - vy * sin(rad_theta);
    y += vx * sin(rad_theta) + vy * cos(rad_theta);

    send_data_enc[0] = (x >> 8) & 0xFF;
    send_data_enc[1] = x & 0xFF;
    send_data_enc[2] = (y >> 8) & 0xFF;
    send_data_enc[3] = y & 0xFF;
    int16_t theta_int = static_cast<int16_t>(theta);
    send_data_enc[4] = (theta_int >> 8) & 0xFF;
    send_data_enc[5] = theta_int & 0xFF;

    for (int i = 0; i < 8; i++) {
        send_data_limit[i] = limit[i].read();
    }
}

int main(void) {
    sken_system.init();
    sken_system.startCanCommunicate(B13, B12, CAN_2); // CAN開始
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
    sken_system.addTimerInterruptFunc(can, 1, 10);
    sken_system.addTimerInterruptFunc(main_interrupt, 2, 1);
    while (1) {
        sken_system.canTransmit(CAN_2, 0x360, send_data_enc, 8, 1);
        sken_system.canTransmit(CAN_2, 0x250, send_data_limit, 8, 1);
    }
}
