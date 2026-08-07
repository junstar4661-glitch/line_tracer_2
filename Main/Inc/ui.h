/*
 * ui.h - LCD 공통 화면 부품
 */

#ifndef INC_UI_H_
#define INC_UI_H_

#include "main.h"

/* ── 화면 규격 ──────────────────────────────
 *  size 12 : 6x12  → 26칸 x 8줄  (기본)
 *  size 16 : 8x16  → 20칸 x 8줄  (/A)
 *  줄 좌표 /0 ~ /7                            */
#define UI_COLS      26
#define UI_COLS_BIG  20

/* ── 색 팔레트 (의미 고정) ───────────────── */
#define UI_C_TITLE   "/#00E0FF"   /* 제목       */
#define UI_C_RULE    "/#2E4A5A"   /* 구분선     */
#define UI_C_LABEL   "/#8894A0"   /* 라벨/안내  */
#define UI_C_VALUE   "/w"         /* 값         */
#define UI_C_ACCENT  "/#FFC800"   /* 선택/강조  */
#define UI_C_OK      "/#00E070"   /* 정상       */
#define UI_C_WARN    "/#FF9800"   /* 주의       */
#define UI_C_BAD     "/#FF3040"   /* 이상       */
#define UI_C_DIM     "/#404A54"   /* 꺼짐/빈칸  */

#define UI_BIG       "/A"
#define UI_SMALL     "/a"

void UI_Screen(const char *title, const char *hint);
void UI_Hint(const char *hint);
void UI_Bar(uint8_t value, uint8_t max, uint8_t width, const char *fillColor);
void UI_CenterBar(int32_t value, int32_t range, uint8_t width);
void UI_Bits(uint8_t bits, uint8_t count);
void UI_Banner(const char *l1, const char *l2, const char *color, uint32_t ms);

#endif /* INC_UI_H_ */
