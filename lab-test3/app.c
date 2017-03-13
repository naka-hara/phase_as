/**
 * TOPPERS/EV3 RT
 *
 * 簡単なプログラミングによるMindstorms EV3操作
 *
 * 2017/3/8 TCS：S-NAKA
 * 
 */

#include "ev3api.h"
#include "app.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

static const sensor_port_t
    sonar_sensorF	= EV3_PORT_1,
    sonar_sensorB	= EV3_PORT_4;

static const motor_port_t
    left_motor		= EV3_PORT_C,
    right_motor		= EV3_PORT_D,
    front_motor		= EV3_PORT_A;

#define SONAR_ALERT_DISTANCE1 60 /* 超音波センサによる障害物検知距離[cm] レベル1*/
#define SONAR_ALERT_DISTANCE2 50 /* 超音波センサによる障害物検知距離[cm] レベル2*/
#define SONAR_ALERT_DISTANCE3 30 /* 超音波センサによる障害物検知距離[cm] レベル3*/

static int Global_Count;
static int GCount_InitFlg;

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);

//*****************************************************************************
// 概要：メイン処理
//
//
//*****************************************************************************

void main_task(intptr_t unused) {

	int act_flg;	// 走行フラグ
	int Lev1_State;	// 障害物検知レベル1状態 0:なし1:レベル1に移行2:レベル1状態
	int Lev2_State;	// 障害物検知レベル1状態 0:なし1:レベル2に移行2:レベル2状態
	int Lev3_State;	// 障害物検知レベル1状態 0:なし1:レベル3に移行2:レベル3状態
	
	/* センサーポートの設定 */
	ev3_sensor_config( sonar_sensorF,ULTRASONIC_SENSOR );
	ev3_sensor_config( sonar_sensorB,ULTRASONIC_SENSOR );
	
	/* モーター出力ポートの設定 */
	ev3_motor_config( left_motor, LARGE_MOTOR );
	ev3_motor_config( right_motor, LARGE_MOTOR );
	ev3_motor_config( front_motor, LARGE_MOTOR );
	
	/* モーターエンコーダリセット */
	ev3_motor_reset_counts( left_motor );
	ev3_motor_reset_counts( right_motor );
	ev3_motor_reset_counts( front_motor );
	
	/* 周期ハンドラリセット */
	eve3_sta_cyc( COUNT_CYC1 );
	GCounta_InitFlg = 1;
	Global_Count = 0;

	/* モーター走行 */
	act_flg = 1;
	
	while ( act_flg ) {
		
		/* 障害物検知 */
		if ( sonar_alert() == 1 ) { // レベル1
			
			/*-- スピードダウン --*/
		
			// 障害物検知状態の設定
			if ( Lev1_State == 0 ) {
				Lev1_State = 1;
				Lev2_State = 0;
				Lev3_Stete = 0;
			}
			else {
				Lev1_State = 2;
				Lev2_State = 0;
				Lev3_State = 0;
			}
			// 自動走行 前進 POWER 40%
			ev3_motor_set_power( left_motor, -40 ); 
    		ev3_motor_set_power( right_motor, -40 );		
		}
		else if ( sonar_alert() == 2 ) { // レベル2
			
			/*-- スピードダウン & 検索動作 --*/
			// 障害物検知状態の設定
			if ( Lev2_State == 0 ) {
				Lev1_State = 0;
				Lev2_State = 1;
				Lev3_Stete = 0;
			}
			else {
				Lev1_State = 0;
				Lev2_State = 2;
				Lev3_State = 0;
			}
			
			if ( Lev2_State == 1 ) {
				/* 周期ハンドラ値初期化 */
				GCounta_InitFlg = 1;
			}
			else {
				
				
			}
			
		}
		else if ( sonar_alert() == 3 ) { // レベル3
		}
		else {
			
			// 障害物検知状態の初期化
			Lev1_State = 0;
			Lev2_State = 0;
			Lev3_State = 0;
			
			// 自動走行 前進 POWER 70%
			ev3_motor_set_power( left_motor, -70 ); 
    		ev3_motor_set_power( right_motor, -70 );
		}
	}
}

//*****************************************************************************
// 概要：超音波センサによる障害物検知
//
// 戻り値 1:レベル1、2:レベル2、3:レベル3
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if ( ++counter == 40/4 ) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance( sonar_sensorF );
        if ( ( distance <= SONAR_ALERT_DISTANCE1 ) && ( distance > SONAR_ALERT_DISTANCE2 ) )
        {
            alert = 1; /* 障害物を検知 */
        }
        else if ( ( distance <= SONAR_ALERT_DISTANCE2 ) && ( distance > SONAR_ALERT_DISTANCE3 ) )
        {
            alert = 2; /* 障害物を検知 */
        }
        else if ( ( distance <= SONAR_ALERT_DISTANCE3 ) && ( distance >= 0 ) )
        {
            alert = 3; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// 概要：1000msルーチン
//
// 周期：1000(ms)
//*****************************************************************************
void Counter_1000cyc(intptr_t idx) {
	if ( GCount_InitFlg == 1 ) {
		Global_Count = 0;
	}
	else {
		Global_Count++;
	}
}

