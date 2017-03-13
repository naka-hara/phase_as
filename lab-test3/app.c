/**
 * TOPPERS/EV3 RT
 *
 * �ȒP�ȃv���O���~���O�ɂ��Mindstorms EV3����
 *
 * 2017/3/8 TCS�FS-NAKA
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

#define SONAR_ALERT_DISTANCE1 150	/* �����g�Z���T�ɂ���Q�����m����[cm] ���x��1*/
#define SONAR_ALERT_DISTANCE2 100	/* �����g�Z���T�ɂ���Q�����m����[cm] ���x��2*/
#define SONAR_ALERT_DISTANCE3 30	/* �����g�Z���T�ɂ���Q�����m����[cm] ���x��3*/

static int Global_Count;
static int GCount_InitFlg;

/* �֐��v���g�^�C�v�錾 */
static int sonar_alert(void);


//*****************************************************************************
// �T�v�F1000ms���[�`��
//
// �����F1000(ms)
//*****************************************************************************
void Counter_1000cyc(intptr_t idx) {
	if ( GCount_InitFlg == 1 ) {
		Global_Count = 0;
	}
	else {
		Global_Count = Global_Count + 1;
	}
}

//*****************************************************************************
// �T�v�F���C������
//
//
//*****************************************************************************

void main_task(intptr_t unused) {

	int rudder_flg;	// ���ǃt���O�F0�F���ɂ��ǂ� 1�F���ǎ��{
	
	// �ϐ�������
	rudder_flg = 0;

	/* �Z���T�[�|�[�g�̐ݒ� */
	ev3_sensor_config( sonar_sensorF,ULTRASONIC_SENSOR );
	ev3_sensor_config( sonar_sensorB,ULTRASONIC_SENSOR );
	
	/* ���[�^�[�o�̓|�[�g�̐ݒ� */
	ev3_motor_config( left_motor, LARGE_MOTOR );
	ev3_motor_config( right_motor, LARGE_MOTOR );
	ev3_motor_config( front_motor, LARGE_MOTOR );
	
	/* ���[�^�[�G���R�[�_���Z�b�g */
	ev3_motor_reset_counts( left_motor );
	ev3_motor_reset_counts( right_motor );
	ev3_motor_reset_counts( front_motor );

	/* �����n���h�����Z�b�g */
	ev3_sta_cyc( COUNT_CYC1 );
	GCount_InitFlg = 1;

	/* ���C���^�X�N */
	while ( 1 ) {
		
		/* ��Q�����m */
		if ( sonar_alert() == 1 ) { // ���x��1
			
			/*-- �X�s�[�h�_�E�� --*/
			// �������s �O�i POWER 50%
			ev3_motor_set_power( left_motor, -50 );
			ev3_motor_set_power( right_motor, -50 );

			GCount_InitFlg = 0;
			Global_Count = 0;
		}
		else if ( sonar_alert() == 2 ) { // ���x��2

			ev3_motor_set_power( left_motor, -10 );
			ev3_motor_set_power( right_motor, -10 );
			
			if( Global_Count > 3000 ) {
				
				if ( rudder_flg == 0 ) {
					ev3_motor_stop( left_motor, true ); // ��~�u���[�L���[�h
					ev3_motor_stop( right_motor, true );

					//ev3_motor_reset_counts( front_motor );
					//ev3_motor_rotate( front_motor, -10, 50, true );
					
					rudder_flg = 1;

					/* �����n���h�����Z�b�g */
					ev3_stp_cyc( COUNT_CYC1 );
					Global_Count = 0;
					ev3_sta_cyc( COUNT_CYC1 );
				}
				else {
					rudder_flg = 0;

					/* �����n���h�����Z�b�g */
					ev3_stp_cyc( COUNT_CYC1 );
					Global_Count = 0;
					ev3_sta_cyc( COUNT_CYC1 );
				}
			}

			// �������s �O�i POWER 20%
//			ev3_motor_set_power( left_motor, -20 );
//			ev3_motor_set_power( right_motor, -20 );
		}
		else if ( sonar_alert() == 3 ) { // ���x��3
			//-- ��ރ��[�h --
			ev3_motor_stop( left_motor, true ); // ��~�u���[�L���[�h
			ev3_motor_stop( right_motor, true );
		}
		else {			
			
			// �������s �O�i POWER 70%
			ev3_motor_set_power( left_motor, -100 ); 
    		ev3_motor_set_power( right_motor, -100 );
		}
	}
}

//*****************************************************************************
// �T�v�F�����g�Z���T�ɂ���Q�����m
//
// �߂�l 1:���x��1�A2:���x��2�A3:���x��3
//*****************************************************************************
static int sonar_alert(void)
{
    static int alert = 0;
    signed int distance;

    distance = ev3_ultrasonic_sensor_get_distance( sonar_sensorF );

    if ( ( distance <= SONAR_ALERT_DISTANCE1 ) && ( distance > SONAR_ALERT_DISTANCE2 ) ) {
		alert = 1; // ��Q�������m
	}
	else if ( ( distance <= SONAR_ALERT_DISTANCE2 ) && ( distance > SONAR_ALERT_DISTANCE3 ) ) {
		alert = 2; // ��Q�������m
	}
	else if ( ( distance <= SONAR_ALERT_DISTANCE3 ) && ( distance >= 0 ) ) {
		alert = 3; // ��Q�������m
	}
/*
    if ( ( distance <= SONAR_ALERT_DISTANCE1 ) && ( distance >= 0 ) ) {
		alert = 1; // ��Q�������m
	    if ( distance <= SONAR_ALERT_DISTANCE2 ) {
			alert = 2; // ��Q�������m
		}
	    if ( distance <= SONAR_ALERT_DISTANCE3 ) {
			alert = 3; // ��Q�������m
		}
	}
*/
	else{
		alert = 0; // ��Q������
	}
    return alert;
}

