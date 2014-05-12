/**
 ******************************************************************************
 ** ファイル名 : sample.c
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/ATK1(OSEK)用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */


/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET      590 /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE	 500 /* 白色の光センサ値 */
#define LIGHT_BLACK	 700 /* 黒色の光センサ値 */
#define KP       0
#define KI       0
#define KD       0

/* sample_c4マクロ */
#define DEVICE_NAME       "ET0"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY          "1234" /* Bluetooth通信用パスキー */
#define CMD_START         '1'    /* リモートスタートコマンド(変更禁止) */

/* 関数プロトタイプ宣言 */
static int remote_start(void);

/* Bluetooth通信用データ受信バッファ */
char rx_buf[BT_MAX_RX_BUF_SIZE];

//*****************************************************************************
// 関数名 : ecrobot_device_initialize
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス初期化処理フック関数
//*****************************************************************************
void ecrobot_device_initialize()
{
	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* 光センサ赤色LEDをON */
	ecrobot_init_bt_slave(PASS_KEY); /* Bluetooth通信初期化 */
}

//*****************************************************************************
// 関数名 : ecrobot_device_terminate
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス終了処理フック関数
//*****************************************************************************
void ecrobot_device_terminate()
{
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* 光センサ赤色LEDをOFF */
	ecrobot_term_bt_connection(); /* Bluetooth通信を終了 */
}

//*****************************************************************************
// 関数名 : user_1ms_isr_type2
// 引数 : なし
// 戻り値 : なし
// 概要 : 1msec周期割り込みフック関数(OSEK ISR type2カテゴリ)
//*****************************************************************************
void user_1ms_isr_type2(void){}

//*****************************************************************************
// タスク名 : TaskMain
// 概要 : メインタスク
//*****************************************************************************
TASK(TaskMain)
{
	signed char forward;      /* 前後進命令 */
	signed char turn;         /* 旋回命令 */
	signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

	signed char dif = 0;
	signed char difInt = 0;
	signed char difPrev = 0;
	signed int  obj = (LIGHT_BLACK + LIGHT_WHITE)/2;
	//signed char now = 0;

	/**
	 * Bluetooth通信用デバイス名の変更は、Bluetooth通信接続が確立されていない場合のみ有効です。
	 * 通信接続確立時にはデバイス名は変更されません。(下記のAPIは何もしません)
	 */
	ecrobot_set_bt_device_name(DEVICE_NAME);

	while(1)
	{

		if (remote_start() == 1)
		{
			break; /* リモートスタート */
		}

		//if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1)
		//{
		//	break; /* タッチセンサが押された */
		//}

		systick_wait_ms(10); /* 10msecウェイト */
	}

	balance_init();			    /* 倒立振子制御初期化 */
	nxt_motor_set_count(NXT_PORT_C, 0); /* 左モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_B, 0); /* 右モータエンコーダリセット */
	while(1)
	{
	  forward = 50; /* 前進命令 */
	  turn = 0;
	  //forward = 50; /* 前進命令 */
	  //	if (ecrobot_get_light_sensor(NXT_PORT_S3) <= (LIGHT_WHITE + LIGHT_BLACK)/2)
	  //	{
	  //		turn = 50;  /* 右旋回命令 */
	  //	}
	  //	else
	  //	{
	  //		turn = -50; /* 左旋回命令 */
	  //	}
	  //

	  dif = (float)( obj -  ecrobot_get_light_sensor(NXT_PORT_S3) ) / 220 * 127 ;
	  //turn が 操作量
	  //now = ecrobot_get_light_sensor(NXT_PORT_S3);   // 現在の光センサの値(制御量)を取得
	  //dif = obj - now;                               // 目標値objと制御量nowの差を得る
	  //difInt = difInt + dif;                         // 前回までの差の蓄積に今回の差を足す

	  turn = dif * KP + difInt * KI + ( difPrev - dif ) * KD; // P、I、Dから実際の操作量を求める。
	  display_goto_xy(0,0);
	  display_string("dif:");
	  display_int(dif,1);
	  display_string("\nturn:");
	  display_int(turn,1);
	  display_update();
	  //turn = 0;
	  //difPrev = dif; // 今回の差を、次回のD制御に渡すためdifPrevに保管しておく


	  	/* 倒立振子制御(forward = 0, turn = 0で静止バランス) */
	  	balance_control(
	  		(float)forward,					 /* 前後進命令(+:前進, -:後進) */
	  		(float)turn,					 /* 旋回命令(+:右旋回, -:左旋回) */
			(float)ecrobot_get_gyro_sensor(NXT_PORT_S1),     /* ジャイロセンサ値 */
			(float)GYRO_OFFSET,			         /* ジャイロセンサオフセット値 */
			(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
			(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
			(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
			&pwm_L,                                          /* 左モータPWM出力値 */
                        &pwm_R);                                         /* 右モータPWM出力値 */
                nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1);               /* 左モータPWM出力セット(-100〜100) */
                nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1);               /* 右モータPWM出力セット(-100〜100) */

		systick_wait_ms(4); /* 4msecウェイト */
	}
}


//*****************************************************************************
// 関数名 : remote_start
// 引数 : 無し
// 返り値 : 1(スタート)/0(待機)
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
static int remote_start(void)
{
	int i;
	unsigned int rx_len;
	unsigned char start = 0;

	for (i=0; i<BT_MAX_RX_BUF_SIZE; i++)
	{
		rx_buf[i] = 0; /* 受信バッファをクリア */
	}

	rx_len = ecrobot_read_bt(rx_buf, 0, BT_MAX_RX_BUF_SIZE);
	if (rx_len > 0)
	{
		/* 受信データあり */
		if (rx_buf[0] == CMD_START)
		{
			start = 1; /* 走行開始 */
		}
		ecrobot_send_bt(rx_buf, 0, rx_len); /* 受信データをエコーバック */
	}

	return start;
}
