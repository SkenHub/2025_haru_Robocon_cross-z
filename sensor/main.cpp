#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

Motor motor[3];//0ローラー:1コンベア:2ピッチ角
RcPwm servo;//ヨー角
Gpio limit;//ヨー角原点
Gpio led;//PS4と接続確認
int x,y,z=0;
int i=38;//iサーボ角度
int j,l=0;//jエンコーダー比較（ピッチ角）:lモーター2上下左右停止判定（ピッチ角）
double deg;
Encoder encoder;//ピッチ角度用
Encoder_data e_data;

PS4 ps4;
PS4_data ps4_data;

void time1()
{
	if (ps4_data.Left == 1&&i<140)i++;//サーボ左旋回
	if (ps4_data.Right == 1&&i>-50)i--;//サーボ右旋回
}

void time2()
{
	if (ps4_data.Up == 1)//モーター2上指令
		{
			if (j<=35)
			{
				j+=5;
				l=810;
			}
		}

	if (ps4_data.Down == 1)//モーター2下指令
		{
			if (j>=10)
			{
				j-=5;
				l=810;
			}
		}
}

void move()
{
	(ps4_data.Cross)?led.write(HIGH):led.write(LOW);//接続確認

	if (ps4_data.Triangle == 1)j=0;//モーター2上下原点
	if (ps4_data.Square == 1)i=38;//サーボ左右原点

	if (ps4_data.Circle == 1)//上下左右同時原点
	{
		j=0;
		i=38;
	}

	motor[1].write(ps4_data.L1*-30);//モーター1コンベア

	deg = e_data.deg;

	servo.turn(i);

	if (j==0)//モーター2上下原点指令時
	{
		if (limit.read())motor[2].write(-20);
		if (!limit.read())
			{
				motor[2].write(0);
				encoder.reset();
			}
	}
	else//(j≠0)
	{
		if (l==810)//モーター2上下判定
		{
			if (deg < j)l=1;
			if (deg == j)l=0;
			if (deg > j)l=-1;
		}
		if (l==1)//モーター2上指令時
		{
			motor[2].write(20);
			if (deg >= j)l=0;
		}
		if (l==-1)//モーター2下指令時
		{
			motor[2].write(-20);
			if (deg <= j)l=0;
		}
		if (l==0)motor[2].write(0);//モーター2上下停止指令判定
	}

	if(ps4_data.RyPad <= 0)//モーター0射出
	{
		motor[0].write(0);
	}
	else
	{
		motor[0].write((ps4_data.RyPad)*0.31);
	}

}

int main(void)
{
	sken_system.init();
	led.init(A5,OUTPUT);
	motor[0].init(Apin,B15,TIMER12,CH2);//1ローラー
	motor[0].init(Bpin,B14,TIMER12,CH1);

	motor[1].init(Apin,A11,TIMER1,CH1);//2コンベア
	motor[1].init(Bpin,A8,TIMER1,CH4);

	motor[2].init(Bpin,A6,TIMER13,CH1);//3ピッチ角
	motor[2].init(Apin,A7,TIMER14,CH1);

	limit.init(C1,INPUT_PULLUP);//r1
	servo.init(B3,TIMER2,CH2);//S2
	encoder.init(A0,A1,TIMER5);
	sken_system.addTimerInterruptFunc(time1,0,30);
	sken_system.addTimerInterruptFunc(time2,1,500);
	ps4.StartRecive(C10,C11,SERIAL3);
	while(1)
	{
		ps4.Getdata(&ps4_data);
		encoder.interrupt(&e_data);
		move();
	}
}

/*#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

Motor motor[3];
Gpio led, m1, m2;

PS4 ps4;
PS4_data ps4_data;
double ly, rx, l, r;

int main(void)
{
    sken_system.init();
    led.init(A5, OUTPUT);
    m1.init(B8, OUTPUT);
    m2.init(B9, OUTPUT);
    motor[0].init(Apin, B15, TIMER12, CH2);
    motor[0].init(Bpin, B14, TIMER12, CH1);
    motor[1].init(Bpin, A8, TIMER1, CH1);
    motor[1].init(Apin, A11, TIMER1, CH4);
    motor[2].init(Apin, A6, TIMER3, CH1);
    motor[2].init(Bpin, A7, TIMER3, CH2);
    ps4.StartRecive(C10, C11, SERIAL3);
    while (1)
    {
        ps4.Getdata(&ps4_data);
        (ps4_data.Cross == 1) ? led.write(HIGH) : led.write(LOW);
        if (ps4_data.L1 == 1)
        {
            m1.write(HIGH);
            m2.write(HIGH);
        }
        else
        {
            m1.write(LOW);
            m2.write(LOW);
        }
        (ps4_data.R1 == 1) ? motor[2].write(80) : motor[2].write(0);
        // motor[0].write((ps4_data.LyPad)/1.28);
        // motor[1].write((ps4_data.RyPad)/1.28);
        ly = (ps4_data.LyPad) * 0.703125;
        rx = (ps4_data.RxPad) * 0.703125;
        if (ly >= 0)
        {
            l = ly + rx;
            r = ly - rx;
        }
        else
        {
            l = ly - rx;
            r = ly + rx;
        }
        if (l >= 90)
        {
            l = 90;
        }
        if (r >= 90)
        {
            r = 90;
        }
        if (l <= -90)
        {
            l = -90;
        }
        if (r <= -90)
        {
            r = -90;
        }
        motor[0].write(l);
        motor[1].write(r);
    }
}*/
