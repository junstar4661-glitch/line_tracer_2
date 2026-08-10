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
#include "button.h"
#include "dac.h"
#include "ui.h"   /* 화면은 전부 ui.c가 그린다 */

#define TREAD_MM         167.0f
#define WHEEL_DIA_MM      52.0648f
#define STEP_ANGLE_DEG     1.8f
#define STEPS_PER_REV    ((360.0f / STEP_ANGLE_DEG) * 2.0f)   // 하프스텝 8상 → 회전당 400스텝
#define MM_PER_STEP      ((WHEEL_DIA_MM * 3.14159f) / STEPS_PER_REV)

/* ★ 전진 방향 보정. 좌우 모터가 마주보게 달려서 한쪽은 뒤집어야 한다.
 *   제자리 회전 → 한쪽만 -1  /  후진 → 양쪽 다 -1  /  정상 → 양쪽 다 +1 */
#define MOTOR_L_DIR_SIGN   (-1)
#define MOTOR_R_DIR_SIGN   (+1)

#define TIM_MOTOR_L &htim1
#define TIM_MOTOR_R &htim8

#define MOTOR_L_IRQ_Handler HAL_TIM1_IRQ_Handler
#define MOTOR_R_IRQ_Handler HAL_TIM8_IRQ_Handler

#define Check_Bit(num, bitMask)	((num & bitMask) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define MOTOR_ARR_MIN	150     // 1MHz 기준 6622step/s (약 2708 mm/s)
#define MOTOR_ARR_MAX	60000

#define TIM_CLK_HZ         1000000.0f   // TIM1/TIM8 prescaler 250-1 적용 후 값
#define RAMP_TICK_HZ       2000.0f      // TIM7 = 1MHz / 500 = 2000Hz (0.5ms)
/* ★ 가속도도 SETUP 화면에서 바꾼다. 속도를 올릴 때마다 같이 올려야 하는 값이라
 *   #define 으로 두면 사다리 한 칸마다 빌드해야 한다.
 *   조향이 이 램프를 통과하므로 = 조향 응답속도이기도 하다 */
#define RAMP_ACCEL_DEFAULT 4000
static volatile float rampStep = (float) RAMP_ACCEL_DEFAULT / RAMP_TICK_HZ;
static volatile int32_t rampAccel = RAMP_ACCEL_DEFAULT;

/* 1200/4095*3.3 = 0.967V. 스텝 놓치면 여기만 100씩 올려라 */
#define MOTOR_DAC_REF      1200
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

void MOTOR_L_IRQ_Handler() {
	static uint8_t index = 0;
	uint8_t outBit = *(stepSequence + index);
	HAL_GPIO_WritePin((Motor_L + 0)->Port, (Motor_L + 0)->Pin, Check_Bit(outBit, 0x1));
	HAL_GPIO_WritePin((Motor_L + 1)->Port, (Motor_L + 1)->Pin, Check_Bit(outBit, 0x2));
	HAL_GPIO_WritePin((Motor_L + 2)->Port, (Motor_L + 2)->Pin, Check_Bit(outBit, 0x4));
	HAL_GPIO_WritePin((Motor_L + 3)->Port, (Motor_L + 3)->Pin, Check_Bit(outBit, 0x8));
	index = (uint8_t) ((index + motorDirL) & 0x7);
	/* 거리는 "전진 = +" 기준으로 쌓는다. 전기적 방향과 분리 */
	stepCountL += (int32_t) (motorDirL * MOTOR_L_DIR_SIGN);
}

void MOTOR_R_IRQ_Handler() {
	static uint8_t index = 0;
	uint8_t outBit = *(stepSequence + index);
	HAL_GPIO_WritePin((Motor_R + 0)->Port, (Motor_R + 0)->Pin, Check_Bit(outBit, 0x1));
	HAL_GPIO_WritePin((Motor_R + 1)->Port, (Motor_R + 1)->Pin, Check_Bit(outBit, 0x2));
	HAL_GPIO_WritePin((Motor_R + 2)->Port, (Motor_R + 2)->Pin, Check_Bit(outBit, 0x4));
	HAL_GPIO_WritePin((Motor_R + 3)->Port, (Motor_R + 3)->Pin, Check_Bit(outBit, 0x8));
	index = (uint8_t) ((index + motorDirR) & 0x7);
	stepCountR += (int32_t) (motorDirR * MOTOR_R_DIR_SIGN);
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

/* ★ 삭제됨: Motor_Timer_Force_Prescaler
 *   MX_TIM1_Init / MX_TIM8_Init 이 이미 Prescaler=250-1 로 초기화하고
 *   HAL_TIM_Base_Init 안에서 UG를 때려 적재까지 끝낸다. 완전 중복이었다.
 *   남는 건 부작용뿐이었다 — 여기서 EGR=UG를 다시 치면
 *     · ARPE=ENABLE 이므로 ARR 섀도우가 초기값 65535로 다시 실린다
 *     · UIF가 셋된 채로 남는다
 *   → Start_IT 순간 스텝 1개가 즉발되고, 그 다음 스텝까지 65.5ms 死구간.
 *     출발 턱걸림의 코드 지분이 이것이었다 */

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

	/* 타이머는 여기서 켜지 않는다. 속도가 생기면 Velocity_To_ARR_*가 켠다 */
	Motor_Stop_L();
	Motor_Stop_R();
	Motor_Coil_Off();
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
	/* 주행 방향(+ = 전진)에 장착 보정 부호를 곱해 전기적 방향으로 변환 */
	Motor_Set_Dir_L((int8_t) ((v_mm_s >= 0 ? 1 : -1) * MOTOR_L_DIR_SIGN));

	float f_step = fabsf(v_mm_s) / MM_PER_STEP;
	uint32_t arr = Motor_Clamp((uint32_t) ((TIM_CLK_HZ / f_step) - 1));

	Motor_Set_ARR_L((uint16_t) arr);
	Motor_Start_L();
}

void Velocity_To_ARR_R(float v_mm_s) {
	if (fabsf(v_mm_s) < 0.5f) {
		Motor_Stop_R();
		Motor_Coil_Off_R();
		return;
	}
	Motor_Set_Dir_R((int8_t) ((v_mm_s >= 0 ? 1 : -1) * MOTOR_R_DIR_SIGN));

	float f_step = fabsf(v_mm_s) / MM_PER_STEP;
	uint32_t arr = Motor_Clamp((uint32_t) ((TIM_CLK_HZ / f_step) - 1));

	Motor_Set_ARR_R((uint16_t) arr);
	Motor_Start_R();
}

void Ramp_Set_Accel(int32_t mm_s2) {
	if (mm_s2 < RAMP_ACCEL_MIN) mm_s2 = RAMP_ACCEL_MIN;
	if (mm_s2 > RAMP_ACCEL_MAX) mm_s2 = RAMP_ACCEL_MAX;
	rampAccel = mm_s2;
	rampStep = (float) mm_s2 / RAMP_TICK_HZ;
}

int32_t Ramp_Get_Accel(void) {
	return rampAccel;
}

void Ramp_Set_Target(float vL, float vR) {
	vTargetL = vL;
	vTargetR = vR;
}

void Ramp_Reset() {
	vTargetL = 0;
	vTargetR = 0;
	vCurL = 0;
	vCurR = 0;
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
	const float RAMP_ACCEL = rampStep;

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
	uint32_t lastDraw = 0;

	DAC_Pin_Force_Analog();
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, MOTOR_DAC_REF);

	Ramp_Reset();
	Ramp_Start();
	Motor_Coil_Off();
	Button_Flush();
	UI_MotorSpd_Frame();

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
				spdL = fminf(2900.0f, spdL + spdL * 0.1f + 1);
			if (target != 1)
				spdR = fminf(2900.0f, spdR + spdR * 0.1f + 1);
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

		if (running) {
			float rL = (target != 2) ? (dirL * spdL) : 0;
			float rR = (target != 1) ? (dirR * spdR) : 0;
			Ramp_Set_Target(rL, rR);
		}
		else {
			Ramp_Set_Target(0, 0);
		}

		if ((now - lastDraw) >= 100) {
			lastDraw = now;
			/* ★ vCurR도 같이 넘긴다. 예전엔 vCurL만 띄워서
			 *   target=R 단독 테스트가 무의미했다 */
			UI_MotorSpd_Update((int16_t) spdL, (int16_t) spdR, dirL, dirR,
					vCurL, vCurR, running, target);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			break;
		}
	}

	Ramp_Set_Target(0, 0);
	HAL_Delay(500);
	Ramp_Stop();
	Motor_Stop();
}

void Motor_Phase_Test() {
	uint8_t target = 0;
	static uint8_t idxL = 0;
	static uint8_t idxR = 0;
	uint32_t lastInputTime = 0;
	uint32_t lastDraw = 0;
	uint8_t coilOn = 0;

	DAC_Pin_Force_Analog();
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, MOTOR_DAC_REF);

	Motor_Coil_Off();
	Button_Flush();
	UI_MotorPhase_Frame();

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

		if ((now - lastDraw) >= 80) {
			lastDraw = now;
			UI_MotorPhase_Update(idxL, idxR, outBitL, outBitR, coilOn, target);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Motor_Coil_Off();
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
			HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
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
