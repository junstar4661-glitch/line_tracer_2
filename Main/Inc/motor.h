/*
 * motor.h
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

/* 현재 속도 (mm/s). 램프가 갱신하며 화면 표시용으로 읽는다 */
extern volatile float vCurL;
extern volatile float vCurR;
extern volatile float vTargetL;
extern volatile float vTargetR;

void Motor_Start(void);
void Motor_Stop(void);

void Motor_Start_L(void);
void Motor_Stop_L(void);
void Motor_Start_R(void);
void Motor_Stop_R(void);

void Motor_Coil_Off(void);
void Motor_Coil_Off_L(void);
void Motor_Coil_Off_R(void);

/* Keep the last energised step while the commanded speed is zero.  This is
 * used only by the final pit-in stop to provide holding torque briefly. */
void Motor_Set_Zero_Speed_Hold(uint8_t enable);

void Motor_Set_Dir_L(int8_t dir);   // +1 정방향, -1 역방향
void Motor_Set_Dir_R(int8_t dir);
int8_t Motor_Get_Dir_L(void);
int8_t Motor_Get_Dir_R(void);

void Motor_Set_ARR_L(uint16_t arr);
void Motor_Set_ARR_R(uint16_t arr);
uint16_t Motor_Get_ARR_L(void);
uint16_t Motor_Get_ARR_R(void);

void Motor_Test(void);
void Motor_Phase_Test(void);

void Ramp_Start(void);
void Ramp_Stop(void);
void Ramp_Reset(void);
/* ── 램프 모드 ────────────────────────────────────────────────
 *  MANUAL : 좌우 목표를 각각 램프한다.  mtr speed / 핏인 정지에서 쓴다
 *  DRIVE  : ★기본속도 하나만 램프하고, 조향은 램프 뒤에 곱한다★
 *           조향이 가속도 제한을 안 받는다 = 코너 응답이 ACC에 안 묶인다
 * ──────────────────────────────────────────────────────────── */
void Ramp_Set_Target(float vL, float vR);   /* MANUAL 모드로 전환 */
void Ramp_Set_Speed(float v);               /* DRIVE 모드로 전환  */
float Ramp_Get_Base(void);                  /* 램프된 현재 기본속도 */

/* DRIVE → MANUAL 로 이음매 없이 넘어간다 (현재 바퀴속도를 목표로 승계) */
void Ramp_Mode_Manual_From_Current(void);

/* 최종 바퀴 속도 출력. 여기 한 곳만 ARR을 건드린다 */
void Motor_Set_Wheels(float vL, float vR);

/* 가속도 = 조향 응답속도. 속도를 올리면 이것도 같이 올려야 한다 */
#define RAMP_ACCEL_MIN     1000
#define RAMP_ACCEL_MAX    40000
#define RAMP_ACCEL_STEP     500
#define RAMP_ACCEL_FAST    2500
void Ramp_Set_Accel(int32_t mm_s2);
int32_t Ramp_Get_Accel(void);

/* Separate braking/deceleration limit.  It is used whenever the requested
 * speed magnitude is reduced, including a direction reversal. */
#define RAMP_DECEL_MIN     1000
#define RAMP_DECEL_MAX    40000
#define RAMP_DECEL_STEP     500
#define RAMP_DECEL_FAST    2500
void Ramp_Set_Decel(int32_t mm_s2);
int32_t Ramp_Get_Decel(void);

void Velocity_To_ARR_L(float v_mm_s);
void Velocity_To_ARR_R(float v_mm_s);

int32_t Distance_Get_L_Mm(void);
int32_t Distance_Get_R_Mm(void);
int32_t Distance_Get_Mm(void);
void Distance_Reset(void);

#endif /* INC_MOTOR_H_ */
