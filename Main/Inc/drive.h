/*
 * drive.h
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "main.h"

void Drive_First(void);
void Drive_Second(void);

/* ★진단용★ 화면에 그대로 띄운다
 *   g_pNow   = 지금 라인 오차 (OFS 보정 후, 단위 0.01mm)
 *   g_pStart = 출발 직후 첫 오차 — 출발 튐의 원인을 여기서 본다 */
extern volatile int32_t g_pNow;
extern volatile int32_t g_pStart;

#endif /* INC_DRIVE_H_ */
