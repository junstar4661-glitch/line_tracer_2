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
/* ★ 가속과 감속을 따로 둔다.
 *   스텝모터는 올릴 때가 어렵고(토크 한계) 내릴 때가 쉽다(마찰이 도와줌).
 *   그리고 코너에서 안쪽 바퀴는 감속, 바깥은 가속이므로
 *   감속을 세게 주면 ★꺾이기 시작하는 게 빨라진다★ */
/* ★단위 m/s^2. 내부 계산은 mm 단위라 1000을 곱해서 쓴다 */
#define RAMP_MM_PER_M      1000.0f
#define RAMP_ACCEL_DEFAULT     5
#define RAMP_DECEL_DEFAULT    10
static volatile float rampAccelStep =
		(float) RAMP_ACCEL_DEFAULT * RAMP_MM_PER_M / RAMP_TICK_HZ;
static volatile float rampDecelStep =
		(float) RAMP_DECEL_DEFAULT * RAMP_MM_PER_M / RAMP_TICK_HZ;
static volatile int32_t rampAccel = RAMP_ACCEL_DEFAULT;
static volatile int32_t rampDecel = RAMP_DECEL_DEFAULT;

/* 코일 전류 기준전압. 값/4095*3.3 = V
 *   2000 → 1.61V (현재)     2200 → 1.77V (실용 상한)
 *   2482 → 2.00V ★절대최대. 이 위는 드라이버 파괴★
 * 스텝 놓치면 100씩만 올리고, 올릴 때마다 10분 돌린 뒤 방열판 온도 확인 */
#define MOTOR_DAC_REF      2000
#define MOTOR_RUN_TIMEOUT_MS  10000     // RUN 자동 차단 (안전장치)

/* mtr speed 화면의 속도 증감 폭 */
#define MOTOR_TEST_SPD_STEP   100.0f
#define MOTOR_TEST_SPD_MAX    2900.0f


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

/* ★ 정지 상태에서 다시 켤 때의 처리가 핵심이다.
 *
 *   Motor_Stop_x() 는 CEN 비트만 끈다. CNT(카운터)와 ARR 섀도우는
 *   지난 주행이 남긴 값 그대로다. 게다가 ARPE=ENABLE 이라 ARR에 새 값을 써도
 *   ★다음 업데이트 이벤트까지 반영이 안 된다.★
 *
 *   그래서 그냥 켜면 첫 스텝이 나올 때까지 1us ~ 65ms 사이 아무 때나 걸리고,
 *   그동안 램프는 계속 올라간다. 첫 스텝이 늦게 나오면 정지 상태에서
 *   갑자기 고속을 명령하는 꼴이라 무조건 탈조한다.
 *   ★"어떤 날은 되고 어떤 날은 안 되는" 불규칙 탈조의 정체가 이것★
 *
 *   해결: 켜기 직전에 UG를 때려서 ARR 섀도우를 즉시 적재하고 CNT를 0으로.
 *         UG가 만든 UIF는 지운다 (안 지우면 스텝 1개가 즉발된다) */
/* ※ 2026-08 롤백: 여기서 EGR=UG로 타이머를 강제 재적재하는 실험을 했다가
 *    상태가 더 나빠져서 원래대로 되돌렸다. 그냥 ARR만 쓰고 켠다. */
__STATIC_INLINE void Motor_Apply_ARR(TIM_HandleTypeDef *htim, uint32_t arr) {
	__HAL_TIM_SET_AUTORELOAD(htim, arr);
	HAL_TIM_Base_Start_IT(htim);
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

	Motor_Apply_ARR(TIM_MOTOR_L, arr);
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

	Motor_Apply_ARR(TIM_MOTOR_R, arr);
}

void Ramp_Set_Accel(int32_t m_s2) {
	if (m_s2 < RAMP_ACCEL_MIN) m_s2 = RAMP_ACCEL_MIN;
	if (m_s2 > RAMP_ACCEL_MAX) m_s2 = RAMP_ACCEL_MAX;
	rampAccel = m_s2;
	rampAccelStep = (float) m_s2 * RAMP_MM_PER_M / RAMP_TICK_HZ;
}

int32_t Ramp_Get_Accel(void) {
	return rampAccel;
}

void Ramp_Set_Decel(int32_t m_s2) {
	if (m_s2 < RAMP_ACCEL_MIN) m_s2 = RAMP_ACCEL_MIN;
	if (m_s2 > RAMP_ACCEL_MAX) m_s2 = RAMP_ACCEL_MAX;
	rampDecel = m_s2;
	rampDecelStep = (float) m_s2 * RAMP_MM_PER_M / RAMP_TICK_HZ;
}

int32_t Ramp_Get_Decel(void) {
	return rampDecel;
}

/* ★ 좌우 기계 불균형 보정 (TRIM).
 *   바퀴 지름 차이 · 베어링 마찰 · 축 휨 때문에 조향을 안 걸어도 한쪽으로 휜다.
 *   K로는 절대 못 잡는다 (K는 센서를 보고 움직이는 값이라 원인이 다르다).
 *
 *   부호 규칙 — ★로봇이 휘는 반대 방향으로 밀어준다★
 *      로봇이 왼쪽으로 휨  → TRIM 을 ＋ 로   (왼쪽 바퀴를 빠르게)
 *      로봇이 오른쪽으로 휨 → TRIM 을 － 로
 *   단위: 0.1%.  TRIM 20 = 왼쪽 +2%, 오른쪽 -2% */
static volatile int32_t motorTrim = 0;

void Motor_Set_Trim(int32_t t) {
	if (t < MOTOR_TRIM_MIN) t = MOTOR_TRIM_MIN;
	if (t > MOTOR_TRIM_MAX) t = MOTOR_TRIM_MAX;
	motorTrim = t;
}

int32_t Motor_Get_Trim(void) {
	return motorTrim;
}

void Ramp_Set_Target(float vL, float vR) {
	float t = (float) motorTrim * 0.001f;

	vTargetL = vL * (1.0f + t);
	vTargetR = vR * (1.0f - t);
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

/* ★★ 좌우가 ★같은 시각에★ 목표에 도착하게 만든다 ★★
 *
 *   예전에는 좌/우를 각각 독립으로 제한했다. 그러면 출발할 때
 *      목표 L1540 / R1460  →  둘 다 0에서 같은 기울기로 올라감
 *      → 0.3초 동안 좌우가 ★똑같다 = 조향이 0★
 *      → 그동안 45cm를 그냥 직진하며 라인에서 벗어남
 *      → 마지막에 갑자기 차동이 생기면서 ★확 튄다★
 *
 *   해결: 더 오래 걸리는 쪽에 시간을 맞추고, 짧은 쪽은 그 시간에 나눠서 간다.
 *         그러면 가속 중에도 좌우 비율(=조향)이 계속 유지된다 */
void HAL_TIM7_IRQ_Handler() {
	const float acc = rampAccelStep;
	const float dec = rampDecelStep;
	float dL, dR, rL, rR, tL, tR, t;

	ramp_tick_count++;

	dL = vTargetL - vCurL;
	dR = vTargetR - vCurR;

	/* 바퀴별로 가속인지 감속인지 (0에서 멀어지면 가속) */
	rL = (fabsf(vTargetL) > fabsf(vCurL)) ? acc : dec;
	rR = (fabsf(vTargetR) > fabsf(vCurR)) ? acc : dec;

	/* 각자 목표까지 몇 틱 걸리나 */
	tL = fabsf(dL) / rL;
	tR = fabsf(dR) / rR;
	t  = (tL > tR) ? tL : tR;      /* 오래 걸리는 쪽에 맞춘다 */

	if (t <= 1.0f) {
		vCurL = vTargetL;          /* 이번 틱에 둘 다 도착 */
		vCurR = vTargetR;
	} else {
		vCurL += dL / t;           /* 남은 거리를 같은 시간에 나눠 간다 */
		vCurR += dR / t;
	}

	Velocity_To_ARR_L(vCurL);
	Velocity_To_ARR_R(vCurR);
}

void Motor_Test() {
	uint8_t target = 0;
	uint8_t running = 0;
	/* 시험 대상이 1000 이상이므로 낮은 데서 시작할 이유가 없다 */
	float spdL = 1500.0f;
	float spdR = 1500.0f;
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

		/* ★ 비율 증감(x1.1) → 고정 1000 단위.
		 *   시험 구간이 1000~2500이라 잔걸음이 의미 없다 */
		/* target 0=BOTH 1=L 2=R ★3=TRIM★
		 *   TRIM 모드에서는 L/R 버튼이 속도가 아니라 좌우 보정값을 움직인다.
		 *   직진 시험을 여기서 하니까 조정도 여기서 하는 게 맞다 */
		if (btn_input == INPUT_CMD_L_SINGLE) {
			if (target == 3) {
				Motor_Set_Trim(Motor_Get_Trim() - MOTOR_TRIM_STEP);
			} else {
				if (target != 2)
					spdL = fmaxf(0.0f, spdL - MOTOR_TEST_SPD_STEP);
				if (target != 1)
					spdR = fmaxf(0.0f, spdR - MOTOR_TEST_SPD_STEP);
			}
		} else if (btn_input == INPUT_CMD_R_SINGLE) {
			if (target == 3) {
				Motor_Set_Trim(Motor_Get_Trim() + MOTOR_TRIM_STEP);
			} else {
				if (target != 2)
					spdL = fminf(MOTOR_TEST_SPD_MAX, spdL + MOTOR_TEST_SPD_STEP);
				if (target != 1)
					spdR = fminf(MOTOR_TEST_SPD_MAX, spdR + MOTOR_TEST_SPD_STEP);
			}
		}
		else if (btn_input == INPUT_CMD_K_SINGLE) {
			target = (uint8_t) ((target + 1) % 4);
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
					vCurL, vCurR, running, target, Motor_Get_Trim());
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
