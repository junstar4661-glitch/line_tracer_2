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

/* ══ 타이머 역할 ══════════════════════════════════════════
 *   TIM1 / TIM8   좌 / 우 모터 스텝 펄스.  ARR이 속도를 정한다
 *   TIM7          ★제어 루프 2kHz★  기본속도 램프 + 조향 분배
 *   TIM6          센서 스캔 8kHz.  슬롯당 IR LED 하나 점등
 *   TIM3          ADC 트리거 지연.  LED 켜고 40us 뒤 변환
 *   SysTick       버튼 폴링 1kHz
 * ═══════════════════════════════════════════════════════ */

void Ramp_Start(void);
void Ramp_Stop(void);
void Ramp_Reset(void);

/* ── 두 가지 운전 모드 ────────────────────────────────────
 *  MANUAL : 좌우 속도를 직접 준다. 각 바퀴가 따로 램프를 탄다
 *           → mtr speed 화면 전용 (조향 없음)
 *  DRIVE  : ★기본속도 하나만★ 준다. 램프도 그것만 탄다.
 *           좌우 분배는 램프 ★뒤에서★ TIM7이 곱셈으로 만든다
 *           → 조향이 가속도 제한을 안 받는다. 이게 이번 구조변경의 핵심
 * ──────────────────────────────────────────────────────── */
void Ramp_Set_Manual(float vL, float vR);
void Ramp_Set_Speed(float v);

/* TIM7이 램프를 끝낸 뒤 호출한다. 좌우 최종속도를 실제 ARR로 내보낸다
 * (TRIM 보정이 여기서 걸린다) */
void Motor_Set_Wheels(float vL, float vR);

/* 좌우 기계 불균형 보정. 단위 0.1% (±100 = ±10%)
 *   로봇이 왼쪽으로 휘면 ＋, 오른쪽으로 휘면 －
 *   Motor_Set_Wheels 안에서 걸리므로 ★주행·mtr speed 양쪽에 다 적용된다★ */
#define MOTOR_TRIM_MIN   (-100)
#define MOTOR_TRIM_MAX   ( 100)
#define MOTOR_TRIM_STEP  (   1)
void Motor_Set_Trim(int32_t t);
int32_t Motor_Get_Trim(void);

/* 가속도 = 조향 응답속도. 속도를 올리면 이것도 같이 올려야 한다.
 * ★단위 m/s^2★  (1 = 1000 mm/s^2)   예) 20 = 20000 mm/s^2 */
#define RAMP_ACCEL_MIN        1
#define RAMP_ACCEL_MAX       40
#define RAMP_ACCEL_STEP       1     /* L·R 짧게 */
#define RAMP_ACCEL_FAST       5     /* L·R 길게 (100ms마다 반복) */
void Ramp_Set_Accel(int32_t m_s2);
int32_t Ramp_Get_Accel(void);
void Ramp_Set_Decel(int32_t m_s2);
int32_t Ramp_Get_Decel(void);

void Velocity_To_ARR_L(float v_mm_s);
void Velocity_To_ARR_R(float v_mm_s);

int32_t Distance_Get_L_Mm(void);
int32_t Distance_Get_R_Mm(void);
int32_t Distance_Get_Mm(void);
void Distance_Reset(void);

#endif /* INC_MOTOR_H_ */
