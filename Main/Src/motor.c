/*
 * motor.c
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#include "motor.h"

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "custom_lcd.h"
#include "button.h"
#include "menu.h"
#include "dac.h"

#define TREAD_MM         177.0f
#define WHEEL_DIA_MM      65.0f
#define STEP_ANGLE_DEG     1.8f
#define STEPS_PER_REV    ((360.0f / STEP_ANGLE_DEG) * 2.0f)   // 하프스텝 8상 → 회전당 400스텝
#define MM_PER_STEP      ((WHEEL_DIA_MM * 3.14159f) / STEPS_PER_REV)

// b1~b6 = 주행선 센서 (수직/Y축 거리, mm), 왼쪽 -, 오른쪽 +
// b0, b7 = 마커 전용 (line position 계산엔 미사용)
const float sensorPositionMm[8] =
{
    0,     // b0 마커 (미사용)
   -37, -23, -8,   // b1, b2, b3 (왼쪽)
   +8, +23, +37,   // b4, b5, b6 (오른쪽)
    0      // b7 마커 (미사용)
};
#define TIM_MOTOR_L &htim1
#define TIM_MOTOR_R &htim8

#define MOTOR_L_IRQ_Handler HAL_TIM1_IRQ_Handler
#define MOTOR_R_IRQ_Handler HAL_TIM8_IRQ_Handler

#define Check_Bit(num, bitMask)	((num & bitMask) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define MOTOR_ARR_MIN	1000      // 1MHz 기준 약 1000step/s 상한
#define MOTOR_ARR_MAX	60000

#define TIM_CLK_HZ         1000000.0f   // TIM1/TIM8 prescaler 250-1 적용 후 값
#define RAMP_TICK_HZ       2000.0f      // TIM7 = 1MHz / 500 = 2000Hz (0.5ms)
#define RAMP_ACCEL_MM_S2   1000.0f      // 목표 가속도. 안 돌면 조금씩 올려라
#define RAMP_ACCEL         (RAMP_ACCEL_MM_S2 / RAMP_TICK_HZ)

#define MOTOR_DAC_REF      1600         // 약 0.97V (VREF 절대최대 2.0V의 절반 이하)
#define MOTOR_RUN_TIMEOUT_MS  10000     // RUN 자동 차단 (안전장치)


typedef struct{
	GPIO_TypeDef* Port;
	uint16_t Pin;
} Motor_TypeDef;

Motor_TypeDef Motor_L[4] = {
		{.Port = MTR_L1_GPIO_Port, .Pin = MTR_L1_Pin},
		{.Port = MTR_L3_GPIO_Port, .Pin = MTR_L3_Pin},
		{.Port = MTR_L2_GPIO_Port, .Pin = MTR_L2_Pin},
		{.Port = MTR_L4_GPIO_Port, .Pin = MTR_L4_Pin},
};

Motor_TypeDef Motor_R[4] = {
		{.Port = MTR_R1_GPIO_Port, .Pin = MTR_R1_Pin},
		{.Port = MTR_R3_GPIO_Port, .Pin = MTR_R3_Pin},
		{.Port = MTR_R2_GPIO_Port, .Pin = MTR_R2_Pin},
		{.Port = MTR_R4_GPIO_Port, .Pin = MTR_R4_Pin},
};

const uint8_t stepSequence[8] = { 0b0001, 0b0011, 0b0010, 0b0110, 0b0100,
		0b1100, 0b1000, 0b1001 };

volatile int8_t motorDirL = 1;
volatile int8_t motorDirR = 1;

volatile int32_t stepCountL = 0;
volatile int32_t stepCountR = 0;

volatile float vTargetL = 0, vTargetR = 0;   // 목표 속도 (mm/s)
volatile float vCurL = 0, vCurR = 0;         // 현재 속도 (mm/s)
volatile uint32_t ramp_tick_count = 0;
volatile uint32_t vector_hit_count = 0;

void MOTOR_L_IRQ_Handler() {
	static uint8_t index = 0;
	uint8_t outBit = *(stepSequence + index);
	HAL_GPIO_WritePin((Motor_L + 0)->Port, (Motor_L + 0)->Pin, Check_Bit(outBit, 0x1));
	HAL_GPIO_WritePin((Motor_L + 1)->Port, (Motor_L + 1)->Pin, Check_Bit(outBit, 0x2));
	HAL_GPIO_WritePin((Motor_L + 2)->Port, (Motor_L + 2)->Pin, Check_Bit(outBit, 0x4));
	HAL_GPIO_WritePin((Motor_L + 3)->Port, (Motor_L + 3)->Pin, Check_Bit(outBit, 0x8));
	index = (uint8_t) ((index + motorDirL) & 0x7);
	stepCountL += motorDirL;
}

void MOTOR_R_IRQ_Handler() {
	static uint8_t index = 0;
	uint8_t outBit = *(stepSequence + index);
	HAL_GPIO_WritePin((Motor_R + 0)->Port, (Motor_R + 0)->Pin, Check_Bit(outBit, 0x1));
	HAL_GPIO_WritePin((Motor_R + 1)->Port, (Motor_R + 1)->Pin, Check_Bit(outBit, 0x2));
	HAL_GPIO_WritePin((Motor_R + 2)->Port, (Motor_R + 2)->Pin, Check_Bit(outBit, 0x4));
	HAL_GPIO_WritePin((Motor_R + 3)->Port, (Motor_R + 3)->Pin, Check_Bit(outBit, 0x8));
	index = (uint8_t) ((index + motorDirR) & 0x7);
	stepCountR += motorDirR;
}

void DAC_Pin_Force_Analog(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Motor_Coil_Off() {
	for (uint8_t i = 0; i < 4; i++) {
		HAL_GPIO_WritePin((Motor_L + i)->Port, (Motor_L + i)->Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin((Motor_R + i)->Port, (Motor_R + i)->Pin, GPIO_PIN_RESET);
	}
}

void Motor_Coil_Off_L() {
	for (uint8_t i = 0; i < 4; i++)
		HAL_GPIO_WritePin((Motor_L + i)->Port, (Motor_L + i)->Pin, GPIO_PIN_RESET);
}

void Motor_Coil_Off_R() {
	for (uint8_t i = 0; i < 4; i++)
		HAL_GPIO_WritePin((Motor_R + i)->Port, (Motor_R + i)->Pin, GPIO_PIN_RESET);
}

void Motor_Timer_Force_Prescaler(void) {
	__HAL_TIM_SET_PRESCALER(TIM_MOTOR_L, 250 - 1);
	__HAL_TIM_SET_PRESCALER(TIM_MOTOR_R, 250 - 1);
	/* PSC는 프리로드라 업데이트 이벤트가 있어야 반영된다 */
	(TIM_MOTOR_L)->Instance->EGR = TIM_EGR_UG;
	(TIM_MOTOR_R)->Instance->EGR = TIM_EGR_UG;
}
void Motor_Start_L() {
	HAL_TIM_Base_Start_IT(TIM_MOTOR_L);
}

void Motor_Stop_L() {
	HAL_TIM_Base_Stop_IT(TIM_MOTOR_L);
}

void Motor_Start_R() {
	HAL_TIM_Base_Start_IT(TIM_MOTOR_R);
}

void Motor_Stop_R() {
	HAL_TIM_Base_Stop_IT(TIM_MOTOR_R);
}

void Motor_Start() {
	DAC_Pin_Force_Analog();
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, MOTOR_DAC_REF);

	Motor_Start_L();
	Motor_Start_R();
}

void Motor_Stop() {
	Motor_Stop_L();
	Motor_Stop_R();
	Motor_Coil_Off();
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
}

void Motor_Set_Dir_L(int8_t dir) {
	motorDirL = (dir >= 0) ? 1 : -1;
}

void Motor_Set_Dir_R(int8_t dir) {
	motorDirR = (dir >= 0) ? 1 : -1;
}

int8_t Motor_Get_Dir_L() {
	return motorDirL;
}

int8_t Motor_Get_Dir_R() {
	return motorDirR;
}

void Motor_Set_ARR_L(uint16_t arr) {
	__HAL_TIM_SET_AUTORELOAD(TIM_MOTOR_L, arr);
}

void Motor_Set_ARR_R(uint16_t arr) {
	__HAL_TIM_SET_AUTORELOAD(TIM_MOTOR_R, arr);
}

uint16_t Motor_Get_ARR_L() {
	return (uint16_t) __HAL_TIM_GET_AUTORELOAD(TIM_MOTOR_L);
}

uint16_t Motor_Get_ARR_R() {
	return (uint16_t) __HAL_TIM_GET_AUTORELOAD(TIM_MOTOR_R);
}

__STATIC_INLINE uint32_t Motor_Clamp(uint32_t arr) {
	if (arr < MOTOR_ARR_MIN)
		return MOTOR_ARR_MIN;
	if (arr > MOTOR_ARR_MAX)
		return MOTOR_ARR_MAX;
	return arr;
}

void Velocity_To_ARR_L(float v_mm_s) {
	if (fabsf(v_mm_s) < 0.5f) {
		Motor_Stop_L();
		Motor_Coil_Off_L();
		return;
	}
	Motor_Set_Dir_L(v_mm_s >= 0 ? 1 : -1);

	float f_step = fabsf(v_mm_s) / MM_PER_STEP;
	uint32_t arr = (uint32_t)((TIM_CLK_HZ / f_step) - 1);
	if (arr < MOTOR_ARR_MIN) arr = MOTOR_ARR_MIN;
	if (arr > MOTOR_ARR_MAX) arr = MOTOR_ARR_MAX;

	Motor_Set_ARR_L((uint16_t) arr);
	Motor_Start_L();
}

void Velocity_To_ARR_R(float v_mm_s) {
	if (fabsf(v_mm_s) < 0.5f) {
		Motor_Stop_R();
		Motor_Coil_Off_R();
		return;
	}
	Motor_Set_Dir_R(v_mm_s >= 0 ? 1 : -1);

	float f_step = fabsf(v_mm_s) / MM_PER_STEP;
	uint32_t arr = (uint32_t)((TIM_CLK_HZ / f_step) - 1);
	if (arr < MOTOR_ARR_MIN) arr = MOTOR_ARR_MIN;
	if (arr > MOTOR_ARR_MAX) arr = MOTOR_ARR_MAX;

	Motor_Set_ARR_R((uint16_t) arr);
	Motor_Start_R();
}

void Ramp_Set_Target(float vL, float vR) {
	vTargetL = vL;
	vTargetR = vR;
}

volatile HAL_StatusTypeDef ramp_start_result;

void Ramp_Start() {
	__HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
	ramp_start_result = HAL_TIM_Base_Start_IT(&htim7);
}

void Ramp_Stop() {
	HAL_TIM_Base_Stop_IT(&htim7);
}

void HAL_TIM7_IRQ_Handler() {
	ramp_tick_count++;
	if (fabsf(vTargetL - vCurL) < RAMP_ACCEL)
		vCurL = vTargetL;
	else
		vCurL += (vTargetL > vCurL) ? RAMP_ACCEL : -RAMP_ACCEL;

	if (fabsf(vTargetR - vCurR) < RAMP_ACCEL)
		vCurR = vTargetR;
	else
		vCurR += (vTargetR > vCurR) ? RAMP_ACCEL : -RAMP_ACCEL;

	Velocity_To_ARR_L(vCurL);
	Velocity_To_ARR_R(vCurR);
}

void Motor_Test() {
	uint8_t target = 0;
	uint8_t running = 0;
	float spdL = 100.0f;
	float spdR = 100.0f;
	int8_t dirL = 1, dirR = 1;
	uint32_t lastAct = 0;
	uint32_t runStartTime = 0;

	DAC_Pin_Force_Analog();
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, MOTOR_DAC_REF);

	Motor_Timer_Force_Prescaler();

	Ramp_Set_Target(0, 0);
	Ramp_Start();
	Motor_Coil_Off();
	Custom_LCD_Clear();

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		if (btn_input == INPUT_CMD_L_SINGLE) {
			if (target != 2)
				spdL = fmaxf(10.0f, spdL - spdL * 0.1f - 1);
			if (target != 1)
				spdR = fmaxf(10.0f, spdR - spdR * 0.1f - 1);
		} else if (btn_input == INPUT_CMD_R_SINGLE) {
			if (target != 2)
				spdL = fminf(500.0f, spdL + spdL * 0.1f + 1);
			if (target != 1)
				spdR = fminf(500.0f, spdR + spdR * 0.1f + 1);
		}
		else if (btn_input == INPUT_CMD_K_SINGLE) {
			target = (uint8_t) ((target + 1) % 3);
		}
		else if (btn_input == INPUT_CMD_L_HOLD && (now - lastAct) > 600) {
			if (target != 2)
				dirL = (int8_t) -dirL;
			if (target != 1)
				dirR = (int8_t) -dirR;
			lastAct = now;
		}
		else if (btn_input == INPUT_CMD_R_HOLD && (now - lastAct) > 600) {
			running = !running;
			if (running)
				runStartTime = now;
			lastAct = now;
		}

		if (running && (now - runStartTime) > MOTOR_RUN_TIMEOUT_MS)
			running = 0;

		if (running)
			Ramp_Set_Target(dirL * spdL, dirR * spdR);
		else
			Ramp_Set_Target(0, 0);

		Custom_LCD_Printf("/0p%-5lu", (unsigned long) (TIM_MOTOR_L)->Instance->PSC);
		Custom_LCD_Printf("/1a%-8lu", (unsigned long) (TIM_MOTOR_L)->Instance->ARR);
		Custom_LCD_Printf("/2v%-4d s%-4d", (int) vCurL, (int) spdL);
		Custom_LCD_Printf("/3%-4s", running ? "RUN" : "OFF");

		if (btn_input == INPUT_CMD_K_HOLD) {
			break;
		}
	}

	Ramp_Set_Target(0, 0);
	HAL_Delay(500);
	Ramp_Stop();
	Motor_Stop();
	Main_Menu();
}

void Motor_Phase_Test() {
	uint8_t target = 0;
	static uint8_t idxL = 0;
	static uint8_t idxR = 0;
	uint32_t lastInputTime = 0;
	uint8_t coilOn = 0;

	DAC_Pin_Force_Analog();
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, MOTOR_DAC_REF);

	Motor_Coil_Off();
	Custom_LCD_Clear();

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		if (btn_input == INPUT_CMD_R_SINGLE) {
			if (target != 2) idxL = (uint8_t) ((idxL + 1) & 0x7);
			if (target != 1) idxR = (uint8_t) ((idxR + 1) & 0x7);
			lastInputTime = now;
			coilOn = 1;
		}
		else if (btn_input == INPUT_CMD_L_SINGLE) {
			if (target != 2) idxL = (uint8_t) ((idxL - 1) & 0x7);
			if (target != 1) idxR = (uint8_t) ((idxR - 1) & 0x7);
			lastInputTime = now;
			coilOn = 1;
		}
		else if (btn_input == INPUT_CMD_K_SINGLE) {
			target = (uint8_t) ((target + 1) % 3);
		}

		if (coilOn && (now - lastInputTime) >= 100) {
			Motor_Coil_Off();
			coilOn = 0;
		}

		uint8_t outBitL = coilOn ? stepSequence[idxL] : 0;
		uint8_t outBitR = coilOn ? stepSequence[idxR] : 0;

		if (coilOn) {
			if (target != 2) {
				HAL_GPIO_WritePin((Motor_L + 0)->Port, (Motor_L + 0)->Pin, Check_Bit(outBitL, 0x1));
				HAL_GPIO_WritePin((Motor_L + 1)->Port, (Motor_L + 1)->Pin, Check_Bit(outBitL, 0x2));
				HAL_GPIO_WritePin((Motor_L + 2)->Port, (Motor_L + 2)->Pin, Check_Bit(outBitL, 0x4));
				HAL_GPIO_WritePin((Motor_L + 3)->Port, (Motor_L + 3)->Pin, Check_Bit(outBitL, 0x8));
			}
			if (target != 1) {
				HAL_GPIO_WritePin((Motor_R + 0)->Port, (Motor_R + 0)->Pin, Check_Bit(outBitR, 0x1));
				HAL_GPIO_WritePin((Motor_R + 1)->Port, (Motor_R + 1)->Pin, Check_Bit(outBitR, 0x2));
				HAL_GPIO_WritePin((Motor_R + 2)->Port, (Motor_R + 2)->Pin, Check_Bit(outBitR, 0x4));
				HAL_GPIO_WritePin((Motor_R + 3)->Port, (Motor_R + 3)->Pin, Check_Bit(outBitR, 0x8));
			}
		} else {
			Motor_Coil_Off();
		}

		const char *tname = (target == 0) ? "BOTH" : (target == 1) ? "L" : "R";
		Custom_LCD_Printf("/0PH %-5s%-4s", tname, coilOn ? "ON" : "OFF");
		Custom_LCD_Printf("/1L#%d %d%d%d%d", idxL,
				(outBitL >> 3) & 1, (outBitL >> 2) & 1, (outBitL >> 1) & 1, outBitL & 1);
		Custom_LCD_Printf("/2R#%d %d%d%d%d", idxR,
				(outBitR >> 3) & 1, (outBitR >> 2) & 1, (outBitR >> 1) & 1, outBitR & 1);

		if (btn_input == INPUT_CMD_K_HOLD) {
			Motor_Coil_Off();
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
			HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
			Main_Menu();
			break;
		}
	}
}
int32_t Distance_Get_L_Mm() {
	return (int32_t)(stepCountL * MM_PER_STEP);
}

int32_t Distance_Get_R_Mm() {
	return (int32_t)(stepCountR * MM_PER_STEP);
}

int32_t Distance_Get_Mm() {
	return (Distance_Get_L_Mm() + Distance_Get_R_Mm()) / 2;
}

void Distance_Reset() {
	stepCountL = 0;
	stepCountR = 0;
}
