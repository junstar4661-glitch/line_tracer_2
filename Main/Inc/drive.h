/*
 * drive.h
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "main.h"

void Drive_First(void);
void Drive_Second(void);

/* ★TIM7(2kHz)이 램프를 끝낸 직후 매 틱 호출한다.
 *   PID → 곡률 감속 → 좌우 분배 → Motor_Set_Wheels 까지 여기서 끝낸다.
 *   주행 루프(폴링)는 조향에 일절 관여하지 않는다 */
void Drive_Control_Tick(float vBase);
void Drive_Control_Enable(uint8_t on);

/* ★진단용★ 화면에 그대로 띄운다
 *   g_pNow   = 지금 라인 오차 (OFS 보정 후, 단위 0.01mm)
 *   g_pStart = 출발 직후 첫 오차 — 출발 튐의 원인을 여기서 본다 */
extern volatile int32_t g_pNow;
extern volatile int32_t g_pStart;

#endif /* INC_DRIVE_H_ */
