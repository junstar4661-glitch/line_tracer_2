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

void Ramp_Start(void);
void Ramp_Stop(void);
void Ramp_Reset(void);
void Ramp_Set_Target(float vL, float vR);

void Velocity_To_ARR_L(float v_mm_s);
void Velocity_To_ARR_R(float v_mm_s);

int32_t Distance_Get_L_Mm(void);
int32_t Distance_Get_R_Mm(void);
int32_t Distance_Get_Mm(void);
void Distance_Reset(void);

void Motor_Timer_Force_Prescaler(void);

#endif /* INC_MOTOR_H_ */
