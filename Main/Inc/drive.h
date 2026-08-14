/*
 * drive.h
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "main.h"

void Drive_First(void);
void Drive_Second(void);

/* ★TIM7 인터럽트(2kHz)가 부른다. 폴링 아님★
 *   vBase = 램프를 통과한 기본속도.  여기서 PD 조향을 곱해 바퀴에 낸다.
 *   조향이 램프 ★뒤★에 있으므로 ACC/DEC 제한을 받지 않는다 */
void Drive_Control_Tick(float vBase);
void Drive_Control_Enable(uint8_t enable);

/* Apply classic Ziegler-Nichols PD tuning: Kp=0.8Ku, Kd=Kp*Tu/8.
 * Ku uses x1e-3 1/m units; Tu is the sustained-oscillation period in ms. */
void Drive_Apply_ZN_PD(uint32_t ku_x1e3, uint32_t tu_ms);

#endif /* INC_DRIVE_H_ */
