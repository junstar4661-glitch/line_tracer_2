/*
 * sensor.c
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#include "sensor.h"
#include "st7735_lcd.h"
#include "gpio.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "custom_lcd.h"
#include "menu.h"
#include "button.h"

#define SENSOR_NUM 8
#define TIM_IR 		&htim6
#define TIM_SENSOR	&htim3
#define ADC_SENSOR	&hadc1

#define TIM_IR_IRQ_Handler HAL_TIM6_IRQ_Handler
#define SENSOR_IRQ_Handler HAL_ADC1_IRQ_Handler

typedef struct {
	GPIO_TypeDef *Port;
	uint16_t Pin;
} IR_TypeDef;

typedef struct {
	uint8_t index;
	uint8_t sensorRaw[SENSOR_NUM];
	uint8_t sensorBlackMax[SENSOR_NUM];
	uint8_t sensorWhiteMax[SENSOR_NUM];
	uint8_t sensorNormalized[SENSOR_NUM];
	uint8_t sensorState;
	uint8_t sensorThreshold;
} IR_DATA;

volatile IR_DATA irData;

// 1 = 흰색일 때 raw가 더 크다, 0 = 검정일 때 raw가 더 크다 (캘리브레이션이 자동 판별)
volatile uint8_t whiteIsHigh = 1;

IR_TypeDef IR_Index[SENSOR_NUM] = { { .Port = IR_0_GPIO_Port, .Pin = IR_0_Pin },
		{ .Port = IR_1_GPIO_Port, .Pin = IR_1_Pin }, { .Port = IR_2_GPIO_Port,
				.Pin = IR_2_Pin }, { .Port = IR_3_GPIO_Port, .Pin = IR_3_Pin },
		{ .Port = IR_3_GPIO_Port, .Pin = IR_3_Pin }, { .Port = IR_2_GPIO_Port,
				.Pin = IR_2_Pin }, { .Port = IR_1_GPIO_Port, .Pin = IR_1_Pin },
		{ .Port = IR_0_GPIO_Port, .Pin = IR_0_Pin }, };

volatile uint32_t tim6_cnt = 0;
volatile uint32_t adc_cnt = 0;

// b1~b6 = 주행선 센서 (수직/Y축 거리, mm), 왼쪽 -, 오른쪽 +
// b0, b7 = 마커 전용 (line position 계산엔 미사용)
const float sensorLinePosMm[8] = {
    0,
   -37, -23, -8,
   +8, +23, +37,
    0
};

volatile int32_t linePosition = 0;

// ---- 마크 인식 FSM ----
typedef enum { MARK_IDLE, MARK_ACCUM } MarkFsmState_t;

volatile MarkFsmState_t markFsmState = MARK_IDLE;
volatile uint8_t markAccum = 0;
volatile uint8_t markLastResult = 0;   // 0=없음, 1=판정됨(방금)
volatile MarkType_t markLastType;

__STATIC_INLINE void IR_Enable(uint8_t idx) {
	HAL_GPIO_WritePin((IR_Index + idx)->Port, (IR_Index + idx)->Pin,
			GPIO_PIN_SET);
}

__STATIC_INLINE void IR_Disable(uint8_t idx) {
	HAL_GPIO_WritePin((IR_Index + idx)->Port, (IR_Index + idx)->Pin,
			GPIO_PIN_RESET);
}

void TIM_IR_IRQ_Handler() {
	IR_Enable(irData.index);
	__HAL_TIM_SET_COUNTER(TIM_SENSOR, 0);
	__HAL_TIM_CLEAR_FLAG(TIM_SENSOR, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE(TIM_SENSOR);
	tim6_cnt++;
}

void SENSOR_IRQ_Handler() {
	uint8_t idx = irData.index;
	adc_cnt++;
	uint16_t adc_raw = HAL_ADC_GetValue(ADC_SENSOR);
	IR_Disable(idx);

	uint8_t raw = (uint8_t) (adc_raw >> 4);
	irData.sensorRaw[idx] = raw;

	// normalize: raw가 클수록 100에 가까워짐
	int32_t w = (int32_t) irData.sensorWhiteMax[idx];
	int32_t b = (int32_t) irData.sensorBlackMax[idx];
	int32_t lo = (w < b) ? w : b;
	int32_t hi = (w < b) ? b : w;
	int32_t den = hi - lo;
	uint8_t norm;

	if (den <= 0) {
		norm = 0;
	} else {
		int32_t v = ((int32_t) raw - lo) * 100 / den;
		if (v < 0)
			v = 0;
		else if (v > 100)
			v = 100;
		norm = (uint8_t) v;
	}
	irData.sensorNormalized[idx] = norm;

	// state: 1 = 흰색(선), 0 = 검정(바탕)
	uint8_t isWhite;
	if (whiteIsHigh)
		isWhite = (norm >= irData.sensorThreshold);
	else
		isWhite = (norm < irData.sensorThreshold);

	if (isWhite)
		irData.sensorState |= (uint8_t) (1u << idx);
	else
		irData.sensorState &= (uint8_t) ~(1u << idx);

	irData.index = (irData.index + 1) & 0x07;
}

void Sensor_Start() {
	static uint8_t adc_calibrated = 0;

	if (!adc_calibrated) {
		HAL_ADCEx_Calibration_Start(ADC_SENSOR, ADC_SINGLE_ENDED);
		adc_calibrated = 1;
	}
	if (irData.sensorThreshold == 0)
		irData.sensorThreshold = 50;

	HAL_ADC_Start_IT(ADC_SENSOR);
	HAL_Delay(10);
	irData.index = 0;
	__HAL_TIM_CLEAR_FLAG(TIM_IR, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(TIM_IR);
}

void Sensor_Stop() {
	HAL_TIM_Base_Stop_IT(TIM_IR);
	HAL_ADC_Stop_IT(ADC_SENSOR);
}

void Sensor_Calibration() {
	UserInput_t btn_input;
	uint8_t i, stage = 0;   // 0: BLACK(바탕), 1: WHITE(선), 2: DONE
	uint8_t bMin[SENSOR_NUM], bMax[SENSOR_NUM];
	uint8_t wMin[SENSOR_NUM], wMax[SENSOR_NUM];
	uint16_t bSum = 0, wSum = 0;

	Sensor_Start();
	Custom_LCD_Clear();
	for (i = 0; i < SENSOR_NUM; i++) {
		bMin[i] = 255;
		bMax[i] = 0;
		wMin[i] = 255;
		wMax[i] = 0;
	}
	Custom_LCD_Printf("BLACK - HOLD:next");
	HAL_Delay(1000);

	while (1) {
		btn_input = Button_Get_Input();
		uint8_t *pMin = (stage == 0) ? bMin : wMin;
		uint8_t *pMax = (stage == 0) ? bMax : wMax;

		for (i = 0; i < SENSOR_NUM; i++) {
			uint8_t r = irData.sensorRaw[i];
			if (r < pMin[i])
				pMin[i] = r;
			if (r > pMax[i])
				pMax[i] = r;
		}
		for (i = 0; i < 2; i++) {
			Custom_LCD_Printf("/%d %-4d%-4d%-4d%-4d", i, pMax[4 * i],
					pMax[4 * i + 1], pMax[4 * i + 2], pMax[4 * i + 3]);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			stage++;
			if (stage == 1) {
				Custom_LCD_Clear();
				Custom_LCD_Printf("WHITE - HOLD:next");
				HAL_Delay(1000);
			} else {
				break;
			}
		}
	}

	// 흑/백 중 어느 쪽 raw가 큰지 자동 판별
	for (i = 0; i < SENSOR_NUM; i++) {
		bSum += bMax[i];
		wSum += wMax[i];
	}
	whiteIsHigh = (wSum > bSum) ? 1 : 0;

	// 센서별 양 끝점 저장
	for (i = 0; i < SENSOR_NUM; i++) {
		uint8_t lo = (bMin[i] < wMin[i]) ? bMin[i] : wMin[i];
		uint8_t hi = (bMax[i] > wMax[i]) ? bMax[i] : wMax[i];
		if (whiteIsHigh) {
			irData.sensorWhiteMax[i] = hi;
			irData.sensorBlackMax[i] = lo;
		} else {
			irData.sensorWhiteMax[i] = lo;
			irData.sensorBlackMax[i] = hi;
		}
	}

	irData.sensorThreshold = 50;

	Sensor_Stop();
	Custom_LCD_Clear();
	Custom_LCD_Printf("/0CAL DONE thr%d", irData.sensorThreshold);
	Custom_LCD_Printf("/1whiteHigh %d", whiteIsHigh);
	HAL_Delay(1500);
	Main_Menu();
}

void Sensor_Test_Raw() {
	Sensor_Start();
	Custom_LCD_Clear();
	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		for (int i = 0; i < 2; i++) {
			Custom_LCD_Printf("/%d %-4d%-4d%-4d%-4d", i,
					irData.sensorRaw[4 * i], irData.sensorRaw[4 * i + 1],
					irData.sensorRaw[4 * i + 2], irData.sensorRaw[4 * i + 3]);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			Main_Menu();
			break;
		}
	}
}

void Sensor_Test_Normalized() {
	Sensor_Start();
	Custom_LCD_Clear();
	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		for (int i = 0; i < 2; i++) {
			Custom_LCD_Printf("/%d %-4d%-4d%-4d%-4d", i,
					irData.sensorNormalized[4 * i],
					irData.sensorNormalized[4 * i + 1],
					irData.sensorNormalized[4 * i + 2],
					irData.sensorNormalized[4 * i + 3]);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			Main_Menu();
			break;
		}
	}
}

int32_t Sensor_Get_Position() {
	int32_t sum_pw = 0;
	int32_t sum_w = 0;

	for (uint8_t i = 1; i <= 6; i++) {
		int32_t w = (int32_t) irData.sensorNormalized[i];
		if (w < 15)
			w = 0;
		sum_pw += (int32_t) (sensorLinePosMm[i] * 100) * w;
		sum_w += w;
	}

	if (sum_w > 0)
		linePosition = sum_pw / sum_w;

	return linePosition;
}

void Mark_FSM_Reset() {
	markFsmState = MARK_IDLE;
	markAccum = 0;
	markLastResult = 0;
}

void Mark_FSM_Tick() {
	uint8_t st = irData.sensorState;
	markLastResult = 0;

	if (markFsmState == MARK_IDLE) {
		if (st & 0x81) {
			markFsmState = MARK_ACCUM;
			markAccum = st;
		}
	} else {
		markAccum |= st;
		if (!(st & 0x81)) {
			if (markAccum == 0xFF)
				markLastType = MARKTYPE_CROSS;
			else if ((markAccum & 0x81) == 0x81)
				markLastType = MARKTYPE_END;
			else if ((markAccum & 0x81) == 0x80)
				markLastType = MARKTYPE_LEFT;
			else
				markLastType = MARKTYPE_RIGHT;

			markLastResult = 1;
			markFsmState = MARK_IDLE;
			markAccum = 0;
		}
	}
}

void Sensor_Test_State() {
	Sensor_Start();
	Custom_LCD_Clear();
	while (1) {
		UserInput_t btn_input = Button_Get_Input();

		uint8_t st = irData.sensorState;
		char bits[SENSOR_NUM + 1];
		uint8_t cnt = 0;

		for (int i = 0; i < SENSOR_NUM; i++) {
			if (st & (1u << i)) {
				bits[i] = '1';
				cnt++;
			} else {
				bits[i] = '0';
			}
		}
		bits[SENSOR_NUM] = '\0';

		Custom_LCD_Printf("/0W %-8s", bits);
		Custom_LCD_Printf("/1thr%-4dn%-3d", irData.sensorThreshold, cnt);
		Custom_LCD_Printf("/2p %-8d", (int) Sensor_Get_Position());

		Mark_FSM_Tick();
		if (markLastResult) {
			const char *names[] = {"LEFT", "RIGHT", "END", "CROSS"};
			Custom_LCD_Printf("/3MARK:%-6s", names[markLastType]);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			Main_Menu();
			break;
		}
	}
}

#define PHASE_MIN	5
#define PHASE_MAX	1000
#define PHASE_STEP	5

void Sensor_Test_Phase() {
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(TIM_SENSOR);

	if (arr < PHASE_MIN)
		arr = PHASE_MIN;
	if (arr > PHASE_MAX)
		arr = PHASE_MAX;

	Sensor_Start();
	Custom_LCD_Clear();

	while (1) {
		UserInput_t btn_input = Button_Get_Input();

		if (btn_input == INPUT_CMD_L_SINGLE || btn_input == INPUT_CMD_L_HOLD) {
			if (arr > PHASE_MIN + PHASE_STEP)
				arr -= PHASE_STEP;
			else
				arr = PHASE_MIN;
			__HAL_TIM_SET_AUTORELOAD(TIM_SENSOR, arr);
		} else if (btn_input == INPUT_CMD_R_SINGLE
				|| btn_input == INPUT_CMD_R_HOLD) {
			if (arr < PHASE_MAX - PHASE_STEP)
				arr += PHASE_STEP;
			else
				arr = PHASE_MAX;
			__HAL_TIM_SET_AUTORELOAD(TIM_SENSOR, arr);
		}

		uint8_t rmin = 255, rmax = 0;
		for (int i = 0; i < SENSOR_NUM; i++) {
			uint8_t r = irData.sensorRaw[i];
			if (r < rmin)
				rmin = r;
			if (r > rmax)
				rmax = r;
		}

		Custom_LCD_Printf("/0PH%-5d d%-4d", (int) arr, (int) (rmax - rmin));
		Custom_LCD_Printf("/1 %-4d%-4d%-4d%-4d", irData.sensorRaw[0],
				irData.sensorRaw[1], irData.sensorRaw[2], irData.sensorRaw[3]);
		Custom_LCD_Printf("/2 %-4d%-4d%-4d%-4d", irData.sensorRaw[4],
				irData.sensorRaw[5], irData.sensorRaw[6], irData.sensorRaw[7]);

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			Main_Menu();
			break;
		}
	}
}
