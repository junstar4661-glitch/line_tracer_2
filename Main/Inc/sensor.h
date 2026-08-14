/*
 * sensor.h
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"

typedef enum {
	MARKTYPE_LEFT = 0,
	MARKTYPE_RIGHT,
	MARKTYPE_END,
	MARKTYPE_CROSS
} MarkType_t;

extern volatile uint8_t whiteIsHigh;
extern volatile int32_t linePosition;

void Sensor_Start(void);
void Sensor_Stop(void);

void Sensor_Calibration(void);
void Sensor_Test_Raw(void);
void Sensor_Test_Normalized(void);
void Sensor_Test_State(void);
void Sensor_Test_Phase(void);

int32_t Sensor_Get_Position(void);
uint8_t Sensor_Line_Found(void);
uint8_t Sensor_Is_Calibrated(void);

/* 1 while a marker or wide cross pattern is physically under the array.
 * This is a tracking-validity signal, not a newly latched marker. */
uint8_t Sensor_Marker_Active(void);

/* ★8슬롯 스캔 한 바퀴가 끝났으면 1 (읽으면 지워진다)★
 *   TIM7 제어기가 이걸 보고 PD를 갱신한다. 시계로 2ms를 세는 것보다
 *   센서 갱신 시점에 정확히 맞아서 미분값이 안 튄다 */
uint8_t Sensor_Take_Frame(void);

/* 캘리브레이션에서 흑백 대비를 확보한 센서 = 1. bit0~bit7 */
uint8_t Sensor_Valid_Mask(void);

/* 마커 판정은 ADC ISR에서 8개 센서 스캔 완료마다 돈다(현재 500Hz).
 * 주행 루프는 Mark_Consume으로 "래치된 결과 1건"을 꺼내 쓴다.
 * 반환 1 = 새 마커 있음(outType에 종류). 꺼내면 래치는 지워진다 */
void Mark_FSM_Reset(void);
uint8_t Mark_Consume(MarkType_t *outType);
/* 마커가 ★시작된★ 지점의 주행거리(mm). Mark_Consume 직후에만 유효 */
int32_t Mark_Get_Dist_Mm(void);
/* 병합창을 거리기준으로 유지하기 위해 현재 주행속도를 알려준다 (mm/s) */
void Mark_Set_Speed(int32_t v_mm_s);

#endif /* INC_SENSOR_H_ */
