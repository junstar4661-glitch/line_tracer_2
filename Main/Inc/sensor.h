
#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include <stdint.h>
typedef enum { MARKTYPE_LEFT, MARKTYPE_RIGHT, MARKTYPE_END, MARKTYPE_CROSS } MarkType_t;

void Sensor_Start(void);
void Sensor_Stop(void);

void Sensor_Calibration(void);
void Sensor_Test_Raw(void);
void Sensor_Test_Normalized(void);
void Sensor_Test_State(void);
void Sensor_Test_Phase(void);

int32_t Sensor_Get_Position(void);

void Mark_FSM_Reset(void);
void Mark_FSM_Tick(void);

extern volatile uint8_t markLastResult;
extern volatile MarkType_t markLastType;

#endif /* INC_SENSOR_H_ */
