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
#include "button.h"
#include "ui.h"   /* 화면은 전부 ui.c가 그린다. 여기엔 좌표도 색도 없다 */

#define SENSOR_NUM 8
#define TIM_IR 		&htim6
#define TIM_SENSOR	&htim3
#define ADC_SENSOR	&hadc1

#define TIM_IR_IRQ_Handler HAL_TIM6_IRQ_Handler
#define SENSOR_IRQ_Handler HAL_ADC1_IRQ_Handler

/* 캘리브레이션에서 흑백 raw 차이가 이 값 미만이면 그 센서는 "죽은 센서"로 본다.
 * 죽은 센서는 norm을 계산하지 않고, 위치계산·마커판정에서 통째로 제외한다.
 * (예전 코드는 den<=0일 때 norm=0으로 뒀는데, whiteIsHigh==0이면
 *  w=100-norm=100 이 되어 "라인이 여기 있다"는 최대 확신으로 뒤집혔다) */
#define SENSOR_MIN_SPAN   12

/* 흰색/검정 판정 문턱. sen state 화면에서 L/R로 조정한다 */
#define SENSOR_THR_MIN    10
#define SENSOR_THR_MAX    90

/* 마커 글리치 가드. 마커 FSM은 ADC ISR에서 1kHz로 도므로 단위가 곧 ms다 */
#define MARK_MIN_MS       8     // 이보다 짧은 발화는 노이즈로 버린다
#define MARK_GAP_MIN_MS   40    // 확정 직후 이 시간 안의 재발화는 채터링으로 무시

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
	uint8_t sensorValid[SENSOR_NUM];      // 1 = 캘리에서 흑백 대비 확보됨
	uint8_t sensorState;
	uint8_t sensorThreshold;
} IR_DATA;

volatile IR_DATA irData;

// 1 = 흰색일 때 raw가 더 크다, 0 = 검정일 때 raw가 더 크다 (캘리브레이션이 자동 판별)
volatile uint8_t whiteIsHigh = 1;

/* ★ 하드웨어 배선 구조상 발광 LED 제어핀은 4개(IR_0~IR_3)를 좌우 대칭으로 공유합니다.
 *   최근 코드에서 이를 1:1(IR_0~IR_7)로 매핑하는 바람에 4~7번 센서의 LED가 켜지지 않아 
 *   값을 읽지 못하는 치명적인 버그가 발생했습니다. 이를 다시 거울복사로 롤백합니다. */
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
   -37.5, -23.5, -7.5,
   +7.5, +23.5, +37.5,
    0
};

/* Sensor_Get_Position이 돌려주는 p의 최대 절댓값.
 * 제일 바깥 라인센서 37.5mm x 100 = 3750 */
#define SENSOR_POS_MAX   3750

volatile int32_t linePosition = 0;
volatile uint8_t lineFound = 0;
/* ★1ms 동안의 위치 변화량. D항이 이걸 쓴다.
 *   제어루프에서 미분하지 않고 ★센서가 갱신되는 시점에★ 차분을 뜬다 */
static volatile int32_t lineDelta = 0;
static volatile int32_t linePrev = 0;

// ---- 마크 인식 FSM (ADC ISR 안에서 1kHz 고정으로 돈다) ----
typedef enum { MARK_IDLE, MARK_ACCUM } MarkFsmState_t;

static volatile MarkFsmState_t markFsmState = MARK_IDLE;
static volatile uint32_t markMs = 0;          // 프레임(=1ms) 카운터
static volatile uint32_t markStartMs = 0;
static volatile uint32_t markLastEndMs = 0;
static volatile uint8_t  markPending = 0;     // 소비 대기중인 확정 마커
static volatile MarkType_t markPendingType = MARKTYPE_LEFT;
static volatile uint8_t  markAccum = 0;

__STATIC_INLINE void IR_Enable(uint8_t idx) {
	HAL_GPIO_WritePin((IR_Index + idx)->Port, (IR_Index + idx)->Pin,
			GPIO_PIN_SET);
}

__STATIC_INLINE void IR_Disable(uint8_t idx) {
	HAL_GPIO_WritePin((IR_Index + idx)->Port, (IR_Index + idx)->Pin,
			GPIO_PIN_RESET);
}

/* 한 프레임(8슬롯) 완성될 때마다 ISR에서 호출된다. 호출주기 = 1ms */
static void Mark_FSM_Step(uint8_t st) {
	/* 양 끝단 센서(0번, 7번) 중 하나라도 감지되었는가? */
	uint8_t edge = (uint8_t) (st & 0x81);

	markMs++;

	if (markFsmState == MARK_IDLE) {
		if (edge) {
			/* 직전 확정으로부터 너무 붙어 있으면 채터링이다 */
			if ((markMs - markLastEndMs) < MARK_GAP_MIN_MS)
				return;
			markFsmState = MARK_ACCUM;
			markStartMs = markMs;
			markAccum = st;
		}
		return;
	}

	/* 1. 마커센서가 켜지면 거기부터 누적한다. */
	markAccum |= st;

	if (!edge) {
		uint32_t width = markMs - markStartMs;

		markFsmState = MARK_IDLE;
		markLastEndMs = markMs;

		if (width < MARK_MIN_MS)
			return;                       // 폭 미달 → 글리치, 버린다

		/* 3. 마커 두개가 다 켜졌으면 */
		if ((markAccum & 0x81) == 0x81) {
			/* ★ 전체 개수가 아니라 ★가운데 라인센서(bit1~6)★ 개수로 가른다.
			 *   CROSS = 라인을 가로지르는 선 → 가운데가 전부 흰색이 된다
			 *   END   = 양옆에만 마커        → 가운데는 라인 위 1~2개뿐
			 *   전체(0~7)로 세면 END에서도 마커2 + 라인2 + 흔들림2 = 6이 나와서
			 *   구분이 안 됐다. 0x7E = bit1~bit6 만 남기는 마스크 */
			if (__builtin_popcount(markAccum & 0x7E) >= 5) {
				markPendingType = MARKTYPE_CROSS;
			} else {
				markPendingType = MARKTYPE_END;
			}
		}
		/* 2. 마커가 끝났을때 왼쪽만 켜졌으면 left 반대는 right다. */
		else if (markAccum & 0x80) {
			markPendingType = MARKTYPE_RIGHT;  /* bit7 = 오른쪽 끝 센서 */
		} else {
			markPendingType = MARKTYPE_LEFT;   /* bit0 = 왼쪽 끝 센서 */
		}

		markPending = 1;
	}
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

	/* 3.3V 시스템 전환 등으로 센서 신호가 매우 약해진 경우를 위해 소프트웨어 증폭.
	 * 기존 >> 4 (1/16) 대신 시프트를 아예 없애서(>> 0) 16배 증폭하고 255로 꽉 채워 제한합니다. */
	uint16_t val = adc_raw;
	if (val > 255) val = 255;
	uint8_t raw = (uint8_t) val;
	irData.sensorRaw[idx] = raw;

	if (!irData.sensorValid[idx]) {
		/* 캘리 실패 센서 = 판단 근거 없음. 항상 "라인 아님"으로 고정한다 */
		irData.sensorNormalized[idx] = 0;
		irData.sensorState &= (uint8_t) ~(1u << idx);
	} else {
		// normalize: raw가 클수록 100에 가까워짐
		int32_t w = (int32_t) irData.sensorWhiteMax[idx];
		int32_t b = (int32_t) irData.sensorBlackMax[idx];
		int32_t lo = (w < b) ? w : b;
		int32_t hi = (w < b) ? b : w;
		int32_t den = hi - lo;
		uint8_t norm;

		int32_t v = ((int32_t) raw - lo) * 100 / den;
		if (v < 0)
			v = 0;
		else if (v > 100)
			v = 100;
		norm = (uint8_t) v;

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
	}

	/* 8슬롯 한 바퀴 = 1ms. 여기서 마커 FSM을 돌리면
	 * 주행루프의 LCD 블로킹과 무관하게 판정주기가 1kHz로 고정된다 */
	if (idx == (SENSOR_NUM - 1))
	{
		Sensor_Update_Position();     /* ★위치·변화량을 여기서 확정한다★ */
		Mark_FSM_Step(irData.sensorState);
	}

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

uint8_t Sensor_Valid_Mask() {
	uint8_t m = 0;
	for (uint8_t i = 0; i < SENSOR_NUM; i++)
		if (irData.sensorValid[i])
			m |= (uint8_t) (1u << i);
	return m;
}

/* ★ 예전엔 "하나라도 다르면 OK"였다. 7개가 죽어도 CAL OK가 떴다.
 *   8개 전부 흑백 대비를 확보해야 캘리 완료로 인정한다 */
uint8_t Sensor_Is_Calibrated() {
	for (uint8_t i = 0; i < SENSOR_NUM; i++) {
		if (!irData.sensorValid[i])
			return 0;
	}
	return 1;
}

uint8_t Sensor_Line_Found() {
	return lineFound;
}

/* ★ 주행 시작 전에 반드시 부른다.
 *   라인을 놓치면 linePosition을 "직전 값 그대로" 유지하는 구조라,
 *   초기화를 안 하면 지난 주행에서 이탈한 방향이 그대로 남아 있다가
 *   출발하자마자 그쪽으로 확 꺾어버린다 */
void Sensor_Reset_Line(void) {
	linePosition = 0;
	lineFound = 0;
	lineDelta = 0;
	linePrev = 0;
}

void Sensor_Calibration() {
	UserInput_t btn_input;
	uint8_t i, stage = 0;
	uint8_t bMin[SENSOR_NUM], bMax[SENSOR_NUM];
	uint8_t wMin[SENSOR_NUM], wMax[SENSOR_NUM];
	uint16_t bSum = 0, wSum = 0;
	uint32_t lastDraw = 0;
	uint8_t drawnStage = 0xFF;
	uint8_t deadCount = 0;

	Sensor_Start();
	Button_Flush();
	UI_Sensor_Frame("CALIBRATE");

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
			UI_Cal_Stage(stage);
			drawnStage = stage;
		}

		if ((now - lastDraw) >= 100) {
			lastDraw = now;
			/* 캘리 중엔 유효성이 아직 없으니 전부 유효로 두고 값 밝기로만 칠한다 */
			UI_Sensor_Cells(pMax, 255, 0, 0xFF, 0);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			stage++;
			if (stage == 1) {
				HAL_Delay(600);
				Button_Flush();
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

		/* 유효성은 캘리 결과로 여기서 확정한다. ISR은 읽기만 한다 */
		if ((uint16_t) (hi - lo) >= SENSOR_MIN_SPAN) {
			irData.sensorValid[i] = 1;
		} else {
			irData.sensorValid[i] = 0;
			deadCount++;
		}

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

	UI_Cal_Result(whiteIsHigh, irData.sensorThreshold, Sensor_Valid_Mask(), deadCount);
	HAL_Delay(2500);
	Button_Flush();
}


/* 8칸을 한 번에 넘기기 위한 스냅샷 (irData는 volatile이라 직접 못 넘긴다) */
static void Sensor_Snapshot(uint8_t *dst, const volatile uint8_t *src) {
	for (uint8_t i = 0; i < SENSOR_NUM; i++)
		dst[i] = src[i];
}

void Sensor_Test_Raw() {
	uint32_t lastDraw = 0;
	uint8_t buf[SENSOR_NUM];

	Sensor_Start();
	Button_Flush();
	UI_Sensor_Frame("SENSOR RAW");
	UI_Sensor_Info(1, UI_C_LABEL, "K-hold: back");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		if ((now - lastDraw) >= 100) {
			lastDraw = now;
			uint8_t lo = 255, hi = 0;

			Sensor_Snapshot(buf, irData.sensorRaw);
			for (uint8_t i = 0; i < SENSOR_NUM; i++) {
				if (buf[i] < lo) lo = buf[i];
				if (buf[i] > hi) hi = buf[i];
			}

			/* raw는 캘리와 무관하게 날값이므로 전부 유효로 그린다 */
			UI_Sensor_Cells(buf, 255, 0, 0xFF, 0);
			UI_Sensor_Info(0, UI_C_LABEL, "min %-4d max %-4d d %-4d", lo, hi, hi - lo);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			break;
		}
	}
}


void Sensor_Test_Normalized() {
	uint32_t lastDraw = 0;
	uint8_t buf[SENSOR_NUM];

	Sensor_Start();
	Button_Flush();
	UI_Sensor_Frame("SENSOR NORM");
	UI_Sensor_Info(1, UI_C_LABEL, "K-hold: back");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		if ((now - lastDraw) >= 100) {
			lastDraw = now;
			Sensor_Snapshot(buf, irData.sensorNormalized);
			/* 박스색은 state 비트를 따른다. 캘리 실패 센서는 빨강으로 뜬다 */
			UI_Sensor_Cells(buf, 100, irData.sensorState, Sensor_Valid_Mask(), 1);
			UI_Sensor_Info(0, UI_C_LABEL, "thr %-4d whiteHigh %-4d",
					irData.sensorThreshold, whiteIsHigh);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			break;
		}
	}
}


/* ★ADC ISR 안에서 8슬롯 한 바퀴 끝날 때마다 호출된다 (1kHz 고정).
 *   예전엔 주행루프가 불규칙하게 불렀다 — 그래서 D를 못 썼다 */
static void Sensor_Update_Position(void) {
	int32_t sum_pw = 0;
	int32_t sum_w = 0;

	for (uint8_t i = 1; i <= 6; i++) {
		/* 죽은 센서는 가중치 계산에 아예 넣지 않는다 */
		if (!irData.sensorValid[i])
			continue;

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

	/* 1ms 동안의 변화량. 라인을 놓친 동안은 0으로 둔다
	 * (직전값을 유지하니 차분이 0이 되는 게 맞다) */
	lineDelta = linePosition - linePrev;
	linePrev = linePosition;
}

int32_t Sensor_Get_Position(void) {
	return linePosition;
}

int32_t Sensor_Get_Delta(void) {
	return lineDelta;
}

void Mark_FSM_Reset() {
	__disable_irq();
	markFsmState = MARK_IDLE;
	markMs = 0;
	markStartMs = 0;
	markLastEndMs = 0;
	markAccum = 0;
	markPending = 0;
	__enable_irq();
}

uint8_t Mark_Consume(MarkType_t *outType) {
	uint8_t got = 0;

	__disable_irq();
	if (markPending) {
		if (outType)
			*outType = markPendingType;
		markPending = 0;
		got = 1;
	}
	__enable_irq();

	return got;
}

void Sensor_Test_State() {
	static const char *const MK[] = { "LEFT", "RIGHT", "END", "CROSS" };
	uint32_t lastDraw = 0;
	uint32_t markShownAt = 0;
	const char *markName = "-";
	uint8_t buf[SENSOR_NUM];
	MarkType_t mt;

	Sensor_Start();
	Mark_FSM_Reset();
	Button_Flush();
	UI_Sensor_Frame("SEN STATE");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		/* ★ L / R 로 흰색/검정 판정 문턱(threshold)을 그 자리에서 바꾼다.
		 *   8칸 박스가 즉시 반응하므로 눈으로 보면서 확정할 수 있다 */
		if (btn_input == INPUT_CMD_L_SINGLE || btn_input == INPUT_CMD_L_HOLD) {
			if (irData.sensorThreshold > SENSOR_THR_MIN)
				irData.sensorThreshold--;
			lastDraw = 0;
		} else if (btn_input == INPUT_CMD_R_SINGLE || btn_input == INPUT_CMD_R_HOLD) {
			if (irData.sensorThreshold < SENSOR_THR_MAX)
				irData.sensorThreshold++;
			lastDraw = 0;
		}

		if (Mark_Consume(&mt)) {
			markName = MK[mt];
			markShownAt = now;
		} else if (markShownAt && (now - markShownAt) > 1500) {
			markName = "-";
			markShownAt = 0;
		}

		if ((now - lastDraw) >= 100) {
			lastDraw = now;

			int32_t p = Sensor_Get_Position();

			Sensor_Snapshot(buf, irData.sensorNormalized);
			UI_Sensor_Cells(buf, 100, irData.sensorState, Sensor_Valid_Mask(), 1);
			UI_Sensor_Pos(p, SENSOR_POS_MAX);
			UI_Sensor_Status(Sensor_Line_Found(), markName);
			UI_Badge_Int("thr", irData.sensorThreshold, UI_C_ACCENT);
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			break;
		}
	}
}

#define PHASE_MIN	5
#define PHASE_MAX	1000
#define PHASE_STEP	5

void Sensor_Test_Phase() {
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(TIM_SENSOR);
	uint32_t lastDraw = 0;
	uint8_t buf[SENSOR_NUM];

	if (arr < PHASE_MIN)
		arr = PHASE_MIN;
	if (arr > PHASE_MAX)
		arr = PHASE_MAX;

	Sensor_Start();
	Button_Flush();
	UI_Sensor_Frame("IR PHASE");
	UI_Sensor_Info(1, UI_C_LABEL, "L-  R+   K-hold: back");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

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

		if ((now - lastDraw) >= 100) {
			lastDraw = now;
			uint8_t lo = 255, hi = 0;

			Sensor_Snapshot(buf, irData.sensorRaw);
			for (uint8_t i = 0; i < SENSOR_NUM; i++) {
				if (buf[i] < lo) lo = buf[i];
				if (buf[i] > hi) hi = buf[i];
			}

			UI_Sensor_Cells(buf, 255, 0, 0xFF, 0);
			UI_Sensor_Info(0, UI_C_LABEL, "phase %-5d  d %-4d",
					(int) arr, (int) (hi - lo));
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			Sensor_Stop();
			break;
		}
	}
}
