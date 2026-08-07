/*
 * sensor.c  — IR_Index 되돌림 테스트용
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
#include "ui.h"

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

/* ★ 되돌림: 뒷절반을 앞절반 거울복사 (예전 상태) */
IR_TypeDef IR_Index[SENSOR_NUM] = {
		{ .Port = IR_0_GPIO_Port, .Pin = IR_0_Pin },
		{ .Port = IR_1_GPIO_Port, .Pin = IR_1_Pin },
		{ .Port = IR_2_GPIO_Port, .Pin = IR_2_Pin },
		{ .Port = IR_3_GPIO_Port, .Pin = IR_3_Pin },
		{ .Port = IR_3_GPIO_Port, .Pin = IR_3_Pin },
		{ .Port = IR_2_GPIO_Port, .Pin = IR_2_Pin },
		{ .Port = IR_1_GPIO_Port, .Pin = IR_1_Pin },
		{ .Port = IR_0_GPIO_Port, .Pin = IR_0_Pin },
};

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
volatile uint8_t lineFound = 0;

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
	for (uint8_t i = 0; i < SENSOR_NUM; i++)
		IR_Disable(i);
}

uint8_t Sensor_Is_Calibrated() {
	for (uint8_t i = 0; i < SENSOR_NUM; i++) {
		if (irData.sensorWhiteMax[i] != irData.sensorBlackMax[i])
			return 1;
	}
	return 0;
}

uint8_t Sensor_Line_Found() {
	return lineFound;
}

void Sensor_Calibration() {
	UserInput_t btn_input;
	uint8_t i, stage = 0;
	uint8_t bMin[SENSOR_NUM], bMax[SENSOR_NUM];
	uint8_t wMin[SENSOR_NUM], wMax[SENSOR_NUM];
	uint16_t bSum = 0, wSum = 0;
	uint32_t lastDraw = 0;
	uint8_t drawnStage = 0xFF;

	Sensor_Start();
	Custom_LCD_Clear();

	for (i = 0; i < SENSOR_NUM; i++) {
		bMin[i] = 255; bMax[i] = 0; wMin[i] = 255; wMax[i] = 0;
	}

	while (1) {
		btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();
		uint8_t *pMin = (stage == 0) ? bMin : wMin;
		uint8_t *pMax = (stage == 0) ? bMax : wMax;

		for (i = 0; i < SENSOR_NUM; i++) {
			uint8_t r = irData.sensorRaw[i];
			if (r < pMin[i]) pMin[i] = r;
			if (r > pMax[i]) pMax[i] = r;
		}

		if (stage != drawnStage) {
			Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-9s", "CAL");
			if (stage == 0)
				Custom_LCD_Printf(UI_C_LABEL "STEP1 " UI_C_VALUE "%-11s", "BLACK");
			else
				Custom_LCD_Printf(UI_C_LABEL "STEP2 " UI_C_ACCENT "%-11s", "WHITE");
			drawnStage = stage;
		}

		if ((now - lastDraw) >= 80) {
			lastDraw = now;
			for (i = 0; i < 4; i++) {
				Custom_LCD_Printf("/%d" UI_SMALL, i + 1);
				Custom_LCD_Printf(UI_C_LABEL "%d ", i);
				UI_Bar(pMax[i], 255, 5, UI_C_TITLE);
				Custom_LCD_Printf(UI_C_VALUE "%3d ", pMax[i]);
				Custom_LCD_Printf(UI_C_LABEL "%d ", i + 4);
				UI_Bar(pMax[i + 4], 255, 5, UI_C_TITLE);
				Custom_LCD_Printf(UI_C_VALUE "%3d", pMax[i + 4]);
			}
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			stage++;
			if (stage == 1) {
				HAL_Delay(600);
			} else {
				break;
			}
		}
	}

	for (i = 0; i < SENSOR_NUM; i++) {
		bSum += bMax[i];
		wSum += wMax[i];
	}
	whiteIsHigh = (wSum > bSum) ? 1 : 0;

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
	UI_Banner("CAL DONE", NULL, UI_C_OK, 0);
	Custom_LCD_Printf("/2" UI_SMALL UI_C_LABEL "thr       " UI_C_VALUE "%-14d", irData.sensorThreshold);
	Custom_LCD_Printf("/3" UI_SMALL UI_C_LABEL "whiteHigh " UI_C_VALUE "%-14d", whiteIsHigh);
	HAL_Delay(1500);
	Main_Menu();
}


void Sensor_Test_Raw() {
	uint32_t lastDraw = 0;

	Sensor_Start();
	Custom_LCD_Clear();
	Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-26s", "SENSOR RAW");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		if ((now - lastDraw) >= 80) {
			lastDraw = now;
			for (uint8_t i = 0; i < 4; i++) {
				Custom_LCD_Printf("/%d" UI_SMALL, i + 1);
				Custom_LCD_Printf(UI_C_LABEL "%d ", i);
				UI_Bar(irData.sensorRaw[i], 255, 5, UI_C_TITLE);
				Custom_LCD_Printf(UI_C_VALUE "%3d ", irData.sensorRaw[i]);
				Custom_LCD_Printf(UI_C_LABEL "%d ", i + 4);
				UI_Bar(irData.sensorRaw[i + 4], 255, 5, UI_C_TITLE);
				Custom_LCD_Printf(UI_C_VALUE "%3d", irData.sensorRaw[i + 4]);
			}
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			Main_Menu();
			break;
		}
	}
}


void Sensor_Test_Normalized() {
	uint32_t lastDraw = 0;

	Sensor_Start();
	Custom_LCD_Clear();
	Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-26s", "SENSOR NORM");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		if ((now - lastDraw) >= 80) {
			lastDraw = now;
			for (uint8_t i = 0; i < 4; i++) {
				uint8_t a = irData.sensorNormalized[i];
				uint8_t b = irData.sensorNormalized[i + 4];

				Custom_LCD_Printf("/%d" UI_SMALL, i + 1);
				Custom_LCD_Printf(UI_C_LABEL "%d ", i);
				UI_Bar(a, 100, 5, (a >= irData.sensorThreshold) ? UI_C_OK : UI_C_TITLE);
				Custom_LCD_Printf(UI_C_VALUE "%3d ", a);
				Custom_LCD_Printf(UI_C_LABEL "%d ", i + 4);
				UI_Bar(b, 100, 5, (b >= irData.sensorThreshold) ? UI_C_OK : UI_C_TITLE);
				Custom_LCD_Printf(UI_C_VALUE "%3d", b);
			}
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
		int32_t n = (int32_t) irData.sensorNormalized[i];
		/* 가중치는 항상 "흰색(라인)일수록 크다"로 통일한다.
		 * norm은 raw가 클수록 100이므로 whiteIsHigh==0이면 뒤집어야 한다 */
		int32_t w = whiteIsHigh ? n : (100 - n);

		if (w < 15)
			w = 0;
		sum_pw += (int32_t) (sensorLinePosMm[i] * 100) * w;
		sum_w += w;
	}

	if (sum_w > 0) {
		linePosition = sum_pw / sum_w;
		lineFound = 1;
	} else {
		lineFound = 0;   // 직전 linePosition 유지 (마지막 방향으로 계속 꺾음)
	}

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
				markLastType = MARKTYPE_RIGHT;  /* bit7 = 오른쪽 끝 센서 */
			else
				markLastType = MARKTYPE_LEFT;   /* bit0 = 왼쪽 끝 센서 */

			markLastResult = 1;
			markFsmState = MARK_IDLE;
			markAccum = 0;
		}
	}
}

void Sensor_Test_State() {
	uint32_t lastDraw = 0;
	uint32_t markShownAt = 0;

	Sensor_Start();
	Custom_LCD_Clear();

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		Mark_FSM_Tick();
		if (markLastResult) {
			const char *names[] = { "LEFT", "RIGHT", "END", "CROSS" };
			const char *col = (markLastType == MARKTYPE_END) ? UI_C_BAD : UI_C_ACCENT;
			Custom_LCD_Printf("/4" UI_SMALL UI_C_LABEL "MARK ");
			Custom_LCD_Printf("%s%-20s", col, names[markLastType]);
			markShownAt = now;
		} else if (markShownAt && (now - markShownAt) > 1200) {
			Custom_LCD_Printf("/4" UI_SMALL UI_C_LABEL "MARK " UI_C_DIM "%-20s", "-");
			markShownAt = 0;
		}

		if ((now - lastDraw) >= 80) {
			lastDraw = now;

			uint8_t st = irData.sensorState;
			uint8_t cnt = 0;
			for (uint8_t i = 0; i < SENSOR_NUM; i++)
				if (st & (1u << i))
					cnt++;

			int32_t p = Sensor_Get_Position();

			Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-13s", "SEN STATE");
			if (Sensor_Line_Found())
				Custom_LCD_Printf(UI_C_OK "%-13s", "ON TRACK");
			else
				Custom_LCD_Printf(UI_C_BAD "%-13s", "LOST");

			Custom_LCD_Printf("/1" UI_SMALL UI_C_LABEL "W ");
			UI_Bits(st, SENSOR_NUM);
			Custom_LCD_Printf("        ");

			Custom_LCD_Printf("/2" UI_SMALL UI_C_LABEL "p");
			Custom_LCD_Printf(UI_C_VALUE "%+6ld ", (long) p);
			UI_CenterBar(p, 3700, 18);

			Custom_LCD_Printf("/3" UI_SMALL UI_C_LABEL "n ");
			Custom_LCD_Printf(UI_C_VALUE "%d", cnt);
			Custom_LCD_Printf(UI_C_LABEL "   thr ");
			Custom_LCD_Printf(UI_C_VALUE "%-3d      ", irData.sensorThreshold);
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
