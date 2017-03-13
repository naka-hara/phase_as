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

#define SONAR_ALERT_DISTANCE 100 /* 超音波センサによる障害物検知距離[cm] */

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);

//*****************************************************************************
// 概要：メイン処理
//
//
//*****************************************************************************

void main_task(intptr_t unused) {

	int act_flg;	// 走行フラグ
	
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


	/* モーター走行 */
	act_flg = 1;
	
	while ( act_flg ) {
		
		/* 障害物検知 */
		if ( sonar_alert() == 1 ) {
			
			/* 自動停止 */
			ev3_motor_stop(left_motor, true);
			ev3_motor_stop(right_motor, true);
			
			act_flg = 0;
		}
		else {
			/* 自動走行 前進 POWER 30% */
			ev3_motor_set_power( left_motor, -100 ); 
    		ev3_motor_set_power( right_motor, -100 );
		}
	}



}

//*****************************************************************************
// 概要：超音波センサによる障害物検知
//
// 周期：1000ms(app.cfgで設定）
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
        if ( ( distance <= SONAR_ALERT_DISTANCE ) && ( distance >= 0 ) )
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

