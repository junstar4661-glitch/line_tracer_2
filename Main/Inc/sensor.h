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

extern volatile uint8_t markLastResult;
extern volatile MarkType_t markLastType;
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

void Mark_FSM_Reset(void);
void Mark_FSM_Tick(void);

#endif /* INC_SENSOR_H_ */
