/*
 * ui.h — 화면 전담. 로직 파일(sensor/motor/drive/menu)은 여기 함수만 부른다.
 *
 * ── 패널 실측 규격 ────────────────────────────────
 *   ST7735 / LANDSCAPE_ROT180 → 160 x 80 px
 *   폰트 small = 6x12 (가로 26자)
 *   폰트 big   = 8x16 (가로 20자)
 *
 *   세로 배치 예산
 *     y  0..12   헤더 바
 *     y 13       구분선 1px
 *     y 14..79   본문 66px
 * ─────────────────────────────────────────────────
 */

#ifndef INC_UI_H_
#define INC_UI_H_

#include "main.h"
#include <stddef.h>

#define UI_W            160
#define UI_H             80
#define UI_HEAD_H        13
#define UI_BODY_Y        14

#define UI_FW             6     // small 폰트 폭
#define UI_FH            12     // small 폰트 높이
#define UI_FW_BIG         8
#define UI_FH_BIG        16

/* 센서 8칸 그리드 */
#define UI_CELL_W        20     // 160 / 8
#define UI_BOX_X          1     // 셀 안에서의 박스 좌측 여백
#define UI_BOX_W         18     // 정사각형
#define UI_BOX_Y         30
#define UI_IDX_Y         16

/* ── 팔레트 (RGB565) ───────────────────────────── */
#define UI_C_BG      0x0000     /* 검정 배경        */
#define UI_C_PANEL   0x0883     /* 패널 바닥        */
#define UI_C_HEAD    0x1148     /* 헤더 바          */
#define UI_C_RULE    0x2A4B     /* 구분선           */
#define UI_C_TITLE   0x071F     /* 제목 (시안)      */
#define UI_C_LABEL   0x8CB4     /* 라벨             */
#define UI_C_VALUE   0xFFFF     /* 값 (흰색)        */
#define UI_C_ACCENT  0xFE40     /* 선택/강조 (노랑) */
#define UI_C_OK      0x070E     /* 정상 (초록)      */
#define UI_C_WARN    0xFCC0     /* 주의 (주황)      */
#define UI_C_BAD     0xF988     /* 이상 (빨강)      */
#define UI_C_DIM     0x424A     /* 꺼짐             */
#define UI_C_BOX     0x1105     /* 박스 기본 바닥   */
#define UI_C_BOXHI   0x0341     /* 박스 활성 바닥   */
#define UI_C_BOXBAD  0x5000     /* 박스 무효 바닥   */
#define UI_C_BLACK   0x0000

/* ── 저수준 ──────────────────────────────────── */
void UI_Init(void);
void UI_Clear(void);
void UI_Fill(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
void UI_Box(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);   // 테두리만
void UI_Text(int16_t x, int16_t y, uint16_t fg, uint16_t bg, const char *fmt, ...);
void UI_TextBig(int16_t x, int16_t y, uint16_t fg, uint16_t bg, const char *fmt, ...);

/* ── 공통 부품 ───────────────────────────────── */
void UI_Header(const char *title, const char *badge, uint16_t badgeCol);
void UI_Badge(const char *badge, uint16_t badgeCol);      // 헤더 우측만 갱신
void UI_Badge_Int(const char *name, int32_t v, uint16_t col);
/* mm/s 값을 "  1.50" 형태 m/s 문자열로. 내부 계산 단위는 계속 mm/s다 */
void UI_MS(char *dst, size_t n, int32_t mm_s);
void UI_Hint(const char *hint);                           // 최하단 안내줄
void UI_Gauge(int16_t x, int16_t y, int16_t w, int16_t h,
		int32_t v, int32_t max, uint16_t c);
void UI_CenterGauge(int16_t x, int16_t y, int16_t w, int16_t h,
		int32_t v, int32_t range, uint16_t c);
void UI_Banner(const char *l1, const char *l2, uint16_t col, uint32_t ms);

/* ── 센서 8칸 그리드 (sensor raw / calibrate / sen norm / sen state) ──
 *   센서 0~7을 가로로 배열하고, 각 번호 아래 정사각형 박스 안에 값을 넣는다.
 *   colorByState = 1 이면 박스 바닥을 state 비트로 칠하고,
 *                  0 이면 값 크기에 비례한 밝기로 칠한다 */
void UI_Sensor_Frame(const char *title);
void UI_Sensor_Cells(const uint8_t *val, uint8_t vmax,
		uint8_t stateBits, uint8_t validBits, uint8_t colorByState);
void UI_Sensor_Info(uint8_t row, uint16_t fg, const char *fmt, ...);
/* 아래 정보줄 0 = 라인 위치(값 + 중앙 게이지), 줄 1 = 상태/마커 */
void UI_Sensor_Pos(int32_t p, int32_t range);
void UI_Sensor_Status(uint8_t lineOk, const char *mark);

/* ── 화면별 ──────────────────────────────────── */
void UI_Menu_Draw(const char *const *names, uint8_t count, uint8_t sel, uint8_t calOk);

void UI_Cal_Stage(uint8_t stage);
void UI_Cal_Result(uint8_t whiteHigh, uint8_t thr, uint8_t validBits, uint8_t dead);

void UI_MotorSpd_Frame(void);
/* target 0=BOTH 1=L 2=R */
void UI_MotorSpd_Update(int16_t spdL, int16_t spdR, int8_t dirL, int8_t dirR,
		float vL, float vR, uint8_t running, uint8_t target);

void UI_MotorPhase_Frame(void);
void UI_MotorPhase_Update(uint8_t idxL, uint8_t idxR, uint8_t bitsL, uint8_t bitsR,
		uint8_t coilOn, uint8_t target);

/* 주행 전 설정 항목. 화면엔 5줄만 보이고 선택이 내려가면 스크롤된다.
 * 게인은 SI 단위다 — Kp [1/m], Kd [s/m] */
typedef enum {
	UI_SET_KP = 0,
	UI_SET_KD,
	UI_SET_SPD,
	UI_SET_ACC,
	UI_SET_DEC,
	UI_SET_FIT,
	UI_SET_COUNT
} UI_SetupItem_t;

#define UI_SETUP_ROWS     UI_SET_COUNT
#define UI_SETUP_VISIBLE  5

void UI_Setup_Frame(const char *title);
void UI_Setup_Update(int32_t kp, int32_t kd, int32_t spd, int32_t acc,
		int32_t dec, int32_t fitInDist, uint8_t sel);
void UI_Countdown(const char *title, int8_t sec);

void UI_Drive_Frame(const char *title);
void UI_Drive_Row(uint8_t row, int32_t dist, uint8_t mkIdx, uint8_t mkTotal,
		const char *mkName, float vL, float vR, uint8_t lineOk);
/* 주행이 왜 끝났는지. 화면에 큰 글씨로 띄운다 */
typedef enum {
	UI_END_COMPLETE = 0,   /* END 마커 2개 — 정상 완주      */
	UI_END_MAP_DONE,       /* 지도 다 재생 — 2차 정상 완주  */
	UI_END_LINE_LOST,      /* 라인 놓침                     */
	UI_END_USER,           /* K 길게 — 사용자 취소          */
	UI_END_MISMATCH,       /* 2차: 지도와 마커 종류가 다름  */
	UI_END_LOG_FULL,       /* 마커 기록칸 가득 참           */
} UI_EndReason_t;

void UI_Drive_Result(UI_EndReason_t reason, uint8_t marks, int32_t dist,
		const char *hint);

/* ── 로그 표 ────────────────────────────────────────────
 *   한 화면에 5줄. 선택이 창 밖으로 나가면 스크롤된다.
 *   drive.c 가 슬롯 5개를 직접 돌려서 그린다 (ui.c는 마커 구조를 모른다) */
#define UI_LOG_VISIBLE  5
/* title = "LOG"(마커 보기) 또는 "SEG"(구간 보기). R 꾹으로 전환한다 */
void UI_Log_Frame(const char *title);
void UI_Log_Head(uint8_t sel, uint8_t total);
void UI_Log_Row(uint8_t slot, int16_t idx, const char *name, int32_t gap,
		uint8_t selected);

#endif /* INC_UI_H_ */
