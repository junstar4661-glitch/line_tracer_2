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
void Sensor_Reset_Line(void);
uint8_t Sensor_Is_Calibrated(void);

/* 캘리브레이션에서 흑백 대비를 확보한 센서 = 1. bit0~bit7 */
uint8_t Sensor_Valid_Mask(void);

/* 마커 판정은 ADC ISR 안에서 1kHz 고정으로 돈다.
 * 주행 루프는 Mark_Consume으로 "래치된 결과 1건"을 꺼내 쓴다.
 * 반환 1 = 새 마커 있음(outType에 종류). 꺼내면 래치는 지워진다 */
void Mark_FSM_Reset(void);
uint8_t Mark_Consume(MarkType_t *outType);

#endif /* INC_SENSOR_H_ */
