/*
 * ui.c — 화면 전담 구현.
 *
 *  로직 파일(sensor/motor/drive/menu)에는 Custom_LCD_Printf를 남기지 않는다.
 *  좌표·색·배치는 전부 여기서만 결정한다.
 *
 *  저수준은 ST7735 BSP를 직접 쓴다.
 *    ST7735_FillRect / DrawHLine  : RGB565를 그대로 받는다 (드라이버가 MSB부터 보냄)
 *    ST7735_WRAPPER_ShowChar      : POINT_COLOR / BACK_COLOR 전역을 보고 그린다.
 *                                   mode 0(배경 덮어쓰기)만 쓴다.
 *                                   mode 1(투명)은 드라이버 쪽 버퍼가 초기화되지 않아
 *                                   쓰레기 픽셀이 찍힌다 — 쓰지 마라.
 */

#include "ui.h"
#include "custom_lcd.h"
#include "st7735_lcd.h"
#include "st7735.h"
#include "drive.h"      /* g_pNow / g_pStart — 진단 숫자를 화면에 띄운다 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define UI_TXT_MAX   40

/* ────────────────────────────────────────────────
 *  저수준
 * ──────────────────────────────────────────────── */

void UI_Fill(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) {
	if (w <= 0 || h <= 0)
		return;
	if (x < 0) { w += x; x = 0; }
	if (y < 0) { h += y; y = 0; }
	if (x >= UI_W || y >= UI_H)
		return;
	if (x + w > UI_W) w = (int16_t) (UI_W - x);
	if (y + h > UI_H) h = (int16_t) (UI_H - y);
	if (w <= 0 || h <= 0)
		return;

	ST7735_FillRect(&st7735_pObj, (uint32_t) x, (uint32_t) y,
			(uint32_t) w, (uint32_t) h, (uint32_t) c);
}

void UI_Box(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) {
	UI_Fill(x, y, w, 1, c);
	UI_Fill(x, (int16_t) (y + h - 1), w, 1, c);
	UI_Fill(x, y, 1, h, c);
	UI_Fill((int16_t) (x + w - 1), y, 1, h, c);
}

void UI_Clear(void) {
	UI_Fill(0, 0, UI_W, UI_H, UI_C_BG);
}

static void ui_puts(int16_t x, int16_t y, uint16_t fg, uint16_t bg,
		uint8_t size, const char *s) {
	uint16_t oldP = ST7735_WRAPPER_POINT_COLOR;
	uint16_t oldB = ST7735_WRAPPER_BACK_COLOR;
	int16_t step = (size == 12) ? UI_FW : UI_FW_BIG;

	ST7735_WRAPPER_POINT_COLOR = fg;
	ST7735_WRAPPER_BACK_COLOR = bg;

	while (*s) {
		if (x >= UI_W)
			break;
		ST7735_WRAPPER_ShowChar((uint16_t) x, (uint16_t) y, (uint8_t) *s, size, 0);
		x = (int16_t) (x + step);
		s++;
	}

	ST7735_WRAPPER_POINT_COLOR = oldP;
	ST7735_WRAPPER_BACK_COLOR = oldB;
}

void UI_Text(int16_t x, int16_t y, uint16_t fg, uint16_t bg, const char *fmt, ...) {
	char buf[UI_TXT_MAX];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	ui_puts(x, y, fg, bg, 12, buf);
}

void UI_TextBig(int16_t x, int16_t y, uint16_t fg, uint16_t bg, const char *fmt, ...) {
	char buf[UI_TXT_MAX];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	ui_puts(x, y, fg, bg, 16, buf);
}

/* 지정 칸수까지 공백으로 채워서 잔상을 지운다 (FillRect보다 싸다) */
static void ui_row(int16_t x, int16_t y, uint16_t fg, uint16_t bg,
		uint8_t cols, const char *buf) {
	char pad[32];
	uint8_t n = (uint8_t) strlen(buf);

	if (cols > 30) cols = 30;
	if (n > cols) n = cols;

	memcpy(pad, buf, n);
	while (n < cols)
		pad[n++] = ' ';
	pad[n] = '\0';

	ui_puts(x, y, fg, bg, 12, pad);
}

void UI_Init(void) {
	Custom_LCD_Init(LCD_TYPE_ST7735);
	UI_Clear();
}

/* ────────────────────────────────────────────────
 *  공통 부품
 * ──────────────────────────────────────────────── */

void UI_Header(const char *title, const char *badge, uint16_t badgeCol) {
	UI_Fill(0, 0, UI_W, UI_HEAD_H, UI_C_HEAD);
	UI_Fill(0, UI_HEAD_H, UI_W, 1, UI_C_RULE);
	UI_Text(3, 0, UI_C_TITLE, UI_C_HEAD, "%s", title);
	if (badge)
		UI_Badge(badge, badgeCol);
}

/* 배지 영역은 x=96부터. 제목은 최대 15자(90px)까지 안 잘린다 */
#define UI_BADGE_X   96

void UI_Badge(const char *badge, uint16_t badgeCol) {
	int16_t n = (int16_t) strlen(badge);
	int16_t x = (int16_t) (UI_W - 3 - n * UI_FW);

	if (x < UI_BADGE_X) x = UI_BADGE_X;
	/* 배지 영역만 헤더색으로 지우고 다시 쓴다 (제목은 건드리지 않는다) */
	UI_Fill(UI_BADGE_X, 0, (int16_t) (UI_W - UI_BADGE_X), UI_FH, UI_C_HEAD);
	ui_puts(x, 0, badgeCol, UI_C_HEAD, 12, badge);
}

/* ★ 속도는 화면에 전부 m/s 로 띄운다. 내부 계산은 mm/s 그대로.
 *   1500 mm/s → " 1.50"  (5칸 고정폭이라 자리가 안 흔들린다) */
void UI_MS(char *dst, size_t n, int32_t mm_s) {
	int32_t a = (mm_s < 0) ? -mm_s : mm_s;
	snprintf(dst, n, "%s%ld.%02ld", (mm_s < 0) ? "-" : " ",
			(long) (a / 1000), (long) ((a % 1000) / 10));
}

void UI_Badge_Int(const char *name, int32_t v, uint16_t col) {
	char b[16];
	snprintf(b, sizeof(b), "%s %ld", name, (long) v);
	UI_Badge(b, col);
}

void UI_Hint(const char *hint) {
	char buf[32];
	snprintf(buf, sizeof(buf), "%s", hint);
	ui_row(2, UI_H - UI_FH, UI_C_LABEL, UI_C_BG, 26, buf);
}

void UI_Gauge(int16_t x, int16_t y, int16_t w, int16_t h,
		int32_t v, int32_t max, uint16_t c) {
	int32_t n;

	if (max <= 0) max = 1;
	if (v < 0) v = 0;
	if (v > max) v = max;

	n = ((int32_t) (w - 2) * v) / max;

	UI_Box(x, y, w, h, UI_C_RULE);
	UI_Fill((int16_t) (x + 1), (int16_t) (y + 1), (int16_t) n, (int16_t) (h - 2), c);
	UI_Fill((int16_t) (x + 1 + n), (int16_t) (y + 1),
			(int16_t) (w - 2 - n), (int16_t) (h - 2), UI_C_PANEL);
}

void UI_CenterGauge(int16_t x, int16_t y, int16_t w, int16_t h,
		int32_t v, int32_t range, uint16_t c) {
	int16_t mid = (int16_t) (x + w / 2);
	int32_t half = (w - 2) / 2;
	int32_t n;

	if (range <= 0) range = 1;
	if (v > range) v = range;
	if (v < -range) v = -range;

	n = (half * v) / range;

	UI_Box(x, y, w, h, UI_C_RULE);
	UI_Fill((int16_t) (x + 1), (int16_t) (y + 1), (int16_t) (w - 2),
			(int16_t) (h - 2), UI_C_PANEL);

	if (n >= 0)
		UI_Fill(mid, (int16_t) (y + 1), (int16_t) (n + 1), (int16_t) (h - 2), c);
	else
		UI_Fill((int16_t) (mid + n), (int16_t) (y + 1), (int16_t) (-n + 1),
				(int16_t) (h - 2), c);

	/* 중앙 기준선 */
	UI_Fill(mid, y, 1, h, UI_C_LABEL);
}

void UI_Banner(const char *l1, const char *l2, uint16_t col, uint32_t ms) {
	int16_t n = (int16_t) strlen(l1);
	int16_t x = (int16_t) ((UI_W - n * UI_FW_BIG) / 2);

	if (x < 0) x = 0;

	UI_Clear();
	UI_Fill(0, 20, UI_W, 3, col);
	UI_TextBig(x, 28, col, UI_C_BG, "%s", l1);
	UI_Fill(0, 47, UI_W, 3, col);

	if (l2) {
		int16_t n2 = (int16_t) strlen(l2);
		int16_t x2 = (int16_t) ((UI_W - n2 * UI_FW) / 2);
		if (x2 < 0) x2 = 0;
		UI_Text(x2, 55, UI_C_LABEL, UI_C_BG, "%s", l2);
	}

	if (ms)
		HAL_Delay(ms);
}

/* ────────────────────────────────────────────────
 *  센서 8칸 그리드
 *
 *   0    1    2    3    4    5    6    7      ← 인덱스 (마커센서 0·7은 노랑)
 *  ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐    ← 18x18 정사각형
 *  │123│ │ 45│ │  0│ ...                      ← 박스 안에 값
 *  └──┘ └──┘ └──┘ ...
 *  info row 0
 *  info row 1
 * ──────────────────────────────────────────────── */

#define UI_INFO0_Y   50
#define UI_INFO1_Y   64

void UI_Sensor_Frame(const char *title) {
	UI_Clear();
	UI_Header(title, NULL, UI_C_LABEL);

	for (uint8_t i = 0; i < 8; i++) {
		int16_t cx = (int16_t) (i * UI_CELL_W);
		/* 마커 전용 센서(0·7)는 번호를 다른 색으로 구분한다 */
		uint16_t c = (i == 0 || i == 7) ? UI_C_ACCENT : UI_C_LABEL;
		UI_Text((int16_t) (cx + 7), UI_IDX_Y, c, UI_C_BG, "%d", i);
	}
}

/* 값 크기 → 남색에서 시안으로 가는 램프 */
static uint16_t ui_ramp(uint8_t v, uint8_t vmax) {
	uint32_t t;
	uint16_t r, g, b;

	if (vmax == 0) vmax = 1;
	t = ((uint32_t) v * 31u) / vmax;
	if (t > 31) t = 31;

	r = (uint16_t) (t / 5);
	g = (uint16_t) (t * 2);
	b = (uint16_t) (t / 2 + 4);

	return (uint16_t) ((r << 11) | (g << 5) | b);
}

void UI_Sensor_Cells(const uint8_t *val, uint8_t vmax,
		uint8_t stateBits, uint8_t validBits, uint8_t colorByState) {
	for (uint8_t i = 0; i < 8; i++) {
		int16_t bx = (int16_t) (i * UI_CELL_W + UI_BOX_X);
		uint16_t bg, fg;
		char buf[8];

		if (!(validBits & (1u << i))) {
			bg = UI_C_BOXBAD;                  // 캘리 실패 = 못 믿을 센서
			fg = UI_C_BAD;
		} else if (colorByState) {
			if (stateBits & (1u << i)) {
				bg = UI_C_BOXHI;               // 흰색(라인) 판정
				fg = UI_C_OK;
			} else {
				bg = UI_C_BOX;                 // 검정(바탕) 판정
				fg = UI_C_LABEL;
			}
		} else {
			bg = ui_ramp(val[i], vmax);
			fg = (val[i] > (uint8_t) (vmax / 2)) ? UI_C_BLACK : UI_C_VALUE;
		}

		UI_Fill(bx, UI_BOX_Y, UI_BOX_W, UI_BOX_W, bg);
		snprintf(buf, sizeof(buf), "%3d", val[i]);
		/* 3자 x 6px = 18px = 박스 폭. 세로는 (18-12)/2 = 3 만큼 내린다 */
		ui_puts(bx, (int16_t) (UI_BOX_Y + 3), fg, bg, 12, buf);
	}
}

void UI_Sensor_Info(uint8_t row, uint16_t fg, const char *fmt, ...) {
	char buf[UI_TXT_MAX];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	ui_row(2, (int16_t) (row ? UI_INFO1_Y : UI_INFO0_Y), fg, UI_C_BG, 26, buf);
}

void UI_Sensor_Pos(int32_t p, int32_t range) {
	UI_Text(2, UI_INFO0_Y, UI_C_LABEL, UI_C_BG, "p");
	UI_Text(10, UI_INFO0_Y, UI_C_VALUE, UI_C_BG, "%+6ld", (long) p);
	UI_CenterGauge(52, (int16_t) (UI_INFO0_Y + 1), 104, 10, p, range, UI_C_ACCENT);
}

void UI_Sensor_Status(uint8_t lineOk, const char *mark) {
	UI_Fill(2, UI_INFO1_Y, 10, 10, lineOk ? UI_C_OK : UI_C_BAD);
	ui_row(16, UI_INFO1_Y, lineOk ? UI_C_OK : UI_C_BAD, UI_C_BG, 9,
			lineOk ? "ON TRACK" : "LOST");
	UI_Text(74, UI_INFO1_Y, UI_C_LABEL, UI_C_BG, "mk");
	ui_row(92, UI_INFO1_Y, UI_C_ACCENT, UI_C_BG, 11, mark ? mark : "-");
}

/* ────────────────────────────────────────────────
 *  메인 메뉴 — 2열 x 4행
 * ──────────────────────────────────────────────── */

#define MENU_COL_W   80
#define MENU_ROW_H   16
#define MENU_ROW_Y   15

void UI_Menu_Draw(const char *const *names, uint8_t count, uint8_t sel, uint8_t calOk) {
	UI_Header("LINE TRACER", calOk ? "CAL OK" : "NO CAL",
			calOk ? UI_C_OK : UI_C_BAD);

	for (uint8_t i = 0; i < count && i < 8; i++) {
		int16_t cx = (int16_t) ((i / 4) * MENU_COL_W);
		int16_t cy = (int16_t) (MENU_ROW_Y + (i % 4) * MENU_ROW_H);
		uint8_t on = (i == sel);
		uint16_t bg = on ? UI_C_ACCENT : UI_C_BG;
		uint16_t fg = on ? UI_C_BLACK : UI_C_LABEL;
		char buf[20];

		UI_Fill(cx, cy, MENU_COL_W, (int16_t) (MENU_ROW_H - 1), bg);
		snprintf(buf, sizeof(buf), "%d %s", i + 1, names[i]);
		ui_puts((int16_t) (cx + 3), (int16_t) (cy + 1), fg, bg, 12, buf);
	}
}

/* ────────────────────────────────────────────────
 *  캘리브레이션
 * ──────────────────────────────────────────────── */

void UI_Cal_Stage(uint8_t stage) {
	if (stage == 0) {
		UI_Header("CALIBRATE", "STEP 1/2", UI_C_LABEL);
		UI_Sensor_Info(0, UI_C_VALUE, "put BLACK under all 8");
	} else {
		UI_Header("CALIBRATE", "STEP 2/2", UI_C_ACCENT);
		UI_Sensor_Info(0, UI_C_ACCENT, "put WHITE under all 8");
	}
	UI_Sensor_Info(1, UI_C_LABEL, "K-hold: next");
}

void UI_Cal_Result(uint8_t whiteHigh, uint8_t thr, uint8_t validBits, uint8_t dead) {
	UI_Clear();

	if (dead) {
		UI_Header("CAL FAIL", "CHECK", UI_C_BAD);
		UI_Text(3, 18, UI_C_BAD, UI_C_BG, "%d dead sensor(s)", dead);
	} else {
		UI_Header("CAL DONE", "OK", UI_C_OK);
		UI_Text(3, 18, UI_C_OK, UI_C_BG, "all 8 sensors alive");
	}

	/* 어느 센서가 살았는지 8칸 그대로 보여준다 */
	for (uint8_t i = 0; i < 8; i++) {
		int16_t bx = (int16_t) (i * UI_CELL_W + 3);
		uint8_t ok = (validBits >> i) & 1u;
		UI_Fill(bx, 36, 14, 14, ok ? UI_C_OK : UI_C_BAD);
		UI_Text((int16_t) (bx + 4), 37, UI_C_BLACK, ok ? UI_C_OK : UI_C_BAD, "%d", i);
	}

	UI_Text(3, 56, UI_C_LABEL, UI_C_BG, "thr ");
	UI_Text(27, 56, UI_C_VALUE, UI_C_BG, "%-4d", thr);
	UI_Text(63, 56, UI_C_LABEL, UI_C_BG, "whiteHigh ");
	UI_Text(123, 56, UI_C_VALUE, UI_C_BG, "%d", whiteHigh);
}

/* ────────────────────────────────────────────────
 *  MTR SPD
 * ──────────────────────────────────────────────── */

/* MOTOR_ARR_MIN 150 이면 둘레 52.0648mm 기준 6622step/s x 0.40892 = 2708 mm/s */
#define UI_SPD_SCALE    2900
#define UI_SPD_LIMIT    2708
#define UI_SPD_STEER    1425    /* = UI_SPD_LIMIT / 1.9, 조향 무포화 상한 */
#define UI_SPD_TICK_X   ((int16_t)(14 + 1 + (140 * UI_SPD_LIMIT) / UI_SPD_SCALE))
#define UI_SPD_STEER_X  ((int16_t)(14 + 1 + (140 * UI_SPD_STEER) / UI_SPD_SCALE))

void UI_MotorSpd_Frame(void) {
	UI_Clear();
	UI_Header("MTR SPD", "OFF", UI_C_DIM);
	UI_Text(3, 16, UI_C_LABEL, UI_C_BG, "set");
	UI_Text(3, 29, UI_C_LABEL, UI_C_BG, "dir");
	UI_Text(3, 42, UI_C_LABEL, UI_C_BG, "now");
	UI_Text(3, 56, UI_C_LABEL, UI_C_BG, "L");
	UI_Text(3, 67, UI_C_LABEL, UI_C_BG, "R");
}

void UI_MotorSpd_Update(int16_t spdL, int16_t spdR, int8_t dirL, int8_t dirR,
		float vL, float vR, uint8_t running, uint8_t target, int32_t trim) {
	const char *tn = (target == 0) ? "BOTH" : (target == 1) ? "L"
				   : (target == 2) ? "R"    : "TRIM";
	char buf[28], mL[8], mR[8];
	float aL = (vL < 0) ? -vL : vL;
	float aR = (vR < 0) ? -vR : vR;

	snprintf(buf, sizeof(buf), "%s %s", running ? "RUN" : "OFF", tn);
	UI_Badge(buf, running ? UI_C_OK : UI_C_DIM);

	UI_MS(mL, sizeof(mL), spdL);
	UI_MS(mR, sizeof(mR), spdR);
	snprintf(buf, sizeof(buf), "L%s R%s m/s", mL, mR);
	ui_row(27, 16, UI_C_VALUE, UI_C_BG, 22, buf);

	UI_Text(27, 29, (dirL > 0) ? UI_C_OK : UI_C_WARN, UI_C_BG,
			"L%-4s", (dirL > 0) ? "FWD" : "REV");
	UI_Text(75, 29, (dirR > 0) ? UI_C_OK : UI_C_WARN, UI_C_BG,
			"R%-4s", (dirR > 0) ? "FWD" : "REV");

	/* 좌우 보정값. TRIM 모드일 때 노랗게 강조된다 */
	UI_Text(112, 29, (target == 3) ? UI_C_ACCENT : UI_C_DIM, UI_C_BG,
			"T%+4ld", (long) trim);

	UI_MS(mL, sizeof(mL), (int32_t) vL);
	UI_MS(mR, sizeof(mR), (int32_t) vR);
	snprintf(buf, sizeof(buf), "L%s R%s m/s", mL, mR);
	ui_row(27, 42, UI_C_VALUE, UI_C_BG, 22, buf);

	/* 게이지 스케일은 mm/s 그대로 쓴다 (0 ~ UI_SPD_SCALE).
	 * 빨강 = 물리 상한 2708, 주황 = 조향 무포화 상한 1425.
	 * 숫자만 m/s로 표시하고 내부 계산 단위는 안 바꿨다 */
	UI_Gauge(14, 55, 142, 10, (int32_t) aL, UI_SPD_SCALE, running ? UI_C_OK : UI_C_DIM);
	UI_Gauge(14, 66, 142, 10, (int32_t) aR, UI_SPD_SCALE, running ? UI_C_OK : UI_C_DIM);
	UI_Fill(UI_SPD_TICK_X, 55, 1, 10, UI_C_BAD);
	UI_Fill(UI_SPD_TICK_X, 66, 1, 10, UI_C_BAD);
	/* 주황 눈금 = 조향 무포화 상한 2708/1.9 = 1425.
	 * 주행 속도를 이 위로 올리면 코너에서 바깥바퀴가 clamp된다 */
	UI_Fill(UI_SPD_STEER_X, 55, 1, 10, UI_C_WARN);
	UI_Fill(UI_SPD_STEER_X, 66, 1, 10, UI_C_WARN);
}

/* ────────────────────────────────────────────────
 *  MTR PHASE
 * ──────────────────────────────────────────────── */

static void ui_phase_bits(int16_t y, uint8_t bits) {
	for (uint8_t i = 0; i < 4; i++) {
		int16_t x = (int16_t) (66 + i * 22);
		uint8_t on = (bits >> i) & 1u;
		UI_Fill(x, y, 18, 16, on ? UI_C_OK : UI_C_BOX);
		UI_Text((int16_t) (x + 6), (int16_t) (y + 2),
				on ? UI_C_BLACK : UI_C_DIM, on ? UI_C_OK : UI_C_BOX,
				"%d", i + 1);
	}
}

void UI_MotorPhase_Frame(void) {
	UI_Clear();
	UI_Header("MTR PHASE", "OFF", UI_C_DIM);
	UI_Hint("L R:step   K:target");
}

void UI_MotorPhase_Update(uint8_t idxL, uint8_t idxR, uint8_t bitsL, uint8_t bitsR,
		uint8_t coilOn, uint8_t target) {
	const char *tn = (target == 0) ? "BOTH" : (target == 1) ? "L" : "R";
	char buf[24];

	snprintf(buf, sizeof(buf), "%s %s", coilOn ? "ON" : "--", tn);
	UI_Badge(buf, coilOn ? UI_C_OK : UI_C_DIM);

	UI_Text(3, 20, (target != 2) ? UI_C_VALUE : UI_C_DIM, UI_C_BG, "L #%d", idxL);
	ui_phase_bits(18, (uint8_t) ((target != 2) ? bitsL : 0));

	UI_Text(3, 44, (target != 1) ? UI_C_VALUE : UI_C_DIM, UI_C_BG, "R #%d", idxR);
	ui_phase_bits(42, (uint8_t) ((target != 1) ? bitsR : 0));
}

/* ────────────────────────────────────────────────
 *  TUNE STEER_K / 카운트다운
 * ──────────────────────────────────────────────── */

/* 5줄: 14 + 13x5 = 79. 값 글씨는 6x12 (16px 큰글씨는 5줄에 안 들어간다).
 * 대신 선택된 줄을 배경색 + 노란 막대로 확실히 구분한다 */
#define SETUP_Y0     14
#define SETUP_ROW_H  13

/* ★ K는 정수 그대로 띄운다. 예전 "0.000180" 표기는 자릿수도 깨졌고
 *   현장에서 눈으로 비교하기도 나빴다. 내부적으로만 x1e-6 이다 */
static void ui_setup_row(uint8_t idx, uint8_t on, const char *name,
		const char *val, const char *unit) {
	int16_t y = (int16_t) (SETUP_Y0 + idx * SETUP_ROW_H);
	uint16_t bg = on ? UI_C_HEAD : UI_C_BG;
	uint16_t nc = on ? UI_C_ACCENT : UI_C_DIM;
	uint16_t vc = on ? UI_C_VALUE : UI_C_LABEL;

	UI_Fill(0, y, UI_W, SETUP_ROW_H, bg);
	if (on)
		UI_Fill(0, y, 3, SETUP_ROW_H, UI_C_ACCENT);

	ui_puts(7,  (int16_t) (y + 1), nc, bg, 12, name);
	ui_puts(42, (int16_t) (y + 1), vc, bg, 12, val);
	ui_puts(90, (int16_t) (y + 1), nc, bg, 12, unit);
}

void UI_Setup_Frame(const char *title) {
	UI_Clear();
	UI_Header(title, "SETUP", UI_C_ACCENT);
}

static const char *const SETUP_NAME[UI_SET_COUNT] = {
	"P  ", "I  ", "D  ", "SPD", "ACC", "DEC", "OFS", "CRV"
};
static const char *const SETUP_UNIT[UI_SET_COUNT] = {
	"gain", "gain", "gain", "m/s", "m/ss", "m/ss", "p zero", "curve"
};

void UI_Setup_Update(const int32_t *vals, uint8_t sel) {
	uint8_t top;
	char b[10], badge[8];

	/* 선택이 보이는 창 아래로 내려가면 창을 끌어내린다 */
	top = (sel < UI_SETUP_VISIBLE) ? 0
			: (uint8_t) (sel - (UI_SETUP_VISIBLE - 1));

	snprintf(badge, sizeof(badge), "%d/%d", sel + 1, UI_SET_COUNT);
	UI_Badge(badge, UI_C_ACCENT);

	for (uint8_t r = 0; r < UI_SETUP_VISIBLE; r++) {
		uint8_t i = (uint8_t) (top + r);

		if (i == UI_SET_SPD)
			UI_MS(b, sizeof(b), vals[i]);
		else if (i == UI_SET_OFS)
			snprintf(b, sizeof(b), "%+5ld", (long) vals[i]);
		else
			snprintf(b, sizeof(b), "%5ld", (long) vals[i]);

		ui_setup_row(r, (uint8_t) (i == sel), SETUP_NAME[i], b, SETUP_UNIT[i]);
	}
}

void UI_Countdown(const char *title, int8_t sec) {
	UI_Clear();
	UI_Header(title, "READY", UI_C_ACCENT);
	UI_TextBig(20, 26, UI_C_ACCENT, UI_C_BG, "START IN %d", sec);
	UI_Text(3, 56, UI_C_LABEL, UI_C_BG, "put it on the start line");
}

/* ────────────────────────────────────────────────
 *  DRIVE — 주행 중에는 한 번에 한 줄만 갱신한다
 * ──────────────────────────────────────────────── */

#define DRV_DIST_Y   16
#define DRV_MARK_Y   36
#define DRV_SPD_Y    50
#define DRV_LINE_Y   64

void UI_Drive_Frame(const char *title) {
	UI_Clear();
	UI_Header(title, "RUN", UI_C_OK);
	/* big "%5ld" 는 x=4에서 40px → 44에서 끝난다 */
	UI_Text(48, DRV_DIST_Y + 4, UI_C_LABEL, UI_C_BG, "mm");
}

void UI_Drive_Row(uint8_t row, int32_t dist, uint8_t mkIdx, uint8_t mkTotal,
		const char *mkName, float vL, float vR, uint8_t lineOk) {
	char buf[28];

	switch (row) {
	case 0:
		UI_TextBig(4, DRV_DIST_Y, UI_C_VALUE, UI_C_BG, "%5ld", (long) dist);
		break;

	case 1:
		/* ★진단★ 마커 정보 + 지금 오차 p. 출발 튐을 눈으로 보려고 넣었다 */
		snprintf(buf, sizeof(buf), "mk%-2d %-6s p%+5ld",
				mkIdx, mkName ? mkName : "-", (long) g_pNow);
		ui_row(4, DRV_MARK_Y, UI_C_ACCENT, UI_C_BG, 25, buf);
		break;

	case 2:
		{
			char mL[8], mR[8];
			UI_MS(mL, sizeof(mL), (int32_t) vL);
			UI_MS(mR, sizeof(mR), (int32_t) vR);
			snprintf(buf, sizeof(buf), "spd L%s R%s m/s", mL, mR);
		}
		ui_row(4, DRV_SPD_Y, UI_C_VALUE, UI_C_BG, 25, buf);
		break;

	default:
		UI_Fill(4, DRV_LINE_Y, 10, 10, lineOk ? UI_C_OK : UI_C_BAD);
		ui_row(20, DRV_LINE_Y, lineOk ? UI_C_OK : UI_C_BAD, UI_C_BG, 22,
				lineOk ? "ON TRACK" : "LINE LOST");
		break;
	}
}

/* ── 결과 화면 ────────────────────────────────────────
 *  y 15..34   ★중지 이유★  색 배경 + 8x16 큰 글씨 (제일 눈에 띄게)
 *  y 37..48   왜 그랬는지 한 줄 설명
 *  y 51..66   마커 수 / 거리 — 8x16 큰 글씨
 *  y 68..79   조작 안내
 * ──────────────────────────────────────────────────── */
#define RES_BAND_Y   15
#define RES_BAND_H   20
#define RES_WHY_Y    37
#define RES_NUM_Y    51

void UI_Drive_Result(UI_EndReason_t reason, uint8_t marks, int32_t dist,
		const char *hint) {
	const char *big;      /* 밴드에 들어갈 큰 글씨 */
	const char *why;      /* 아래 한 줄 설명       */
	uint16_t col;         /* 밴드 배경색           */
	uint8_t ok;
	int16_t bx;
	char buf[24];

	switch (reason) {
	case UI_END_COMPLETE:
		big = "COMPLETE";  why = "END mark seen twice";  col = UI_C_OK;   ok = 1; break;
	case UI_END_MAP_DONE:
		big = "MAP DONE";  why = "all marks replayed";   col = UI_C_OK;   ok = 1; break;
	case UI_END_LINE_LOST:
		big = "LINE LOST"; why = "no line for 50ms";     col = UI_C_BAD;  ok = 0; break;
	case UI_END_USER:
		big = "USER STOP"; why = "K-hold pressed";       col = UI_C_WARN; ok = 0; break;
	default:
		big = "LOG FULL";  why = "mark buffer overflow"; col = UI_C_WARN; ok = 0; break;
	}

	UI_Clear();
	UI_Header("RESULT", ok ? "OK" : "FAIL", ok ? UI_C_OK : UI_C_BAD);

	/* 중지 이유 — 색 밴드 위에 큰 글씨, 가운데 정렬 */
	UI_Fill(0, RES_BAND_Y, UI_W, RES_BAND_H, col);
	bx = (int16_t) ((UI_W - (int16_t) strlen(big) * UI_FW_BIG) / 2);
	if (bx < 0) bx = 0;
	ui_puts(bx, (int16_t) (RES_BAND_Y + 2), UI_C_BLACK, col, 16, big);

	ui_row(3, RES_WHY_Y, ok ? UI_C_LABEL : col, UI_C_BG, 26, why);

	/* 숫자도 큰 글씨로. mk = 마커 개수, 그 옆이 총 주행거리 */
	snprintf(buf, sizeof(buf), "mk%-3d %5ldmm", marks, (long) dist);
	ui_puts(3, RES_NUM_Y, UI_C_VALUE, UI_C_BG, 16, buf);

	/* ★출발 순간의 오차★ — 출발 튐 진단의 핵심 숫자 */
	snprintf(buf, sizeof(buf), "p at start %+5ld", (long) g_pStart);
	ui_row(3, RES_WHY_Y + 12, UI_C_ACCENT, UI_C_BG, 26, buf);

	if (hint)
		UI_Hint(hint);
}

/* ────────────────────────────────────────────────
 *  LOG REVIEW
 * ──────────────────────────────────────────────── */

void UI_Log_Frame(void) {
	UI_Clear();
	UI_Header("LOG REVIEW", NULL, UI_C_LABEL);
	UI_Hint("L/R: move   K-hold: exit");
}

void UI_Log_Update(uint8_t i, uint8_t count, const char *name, int32_t gap) {
	char buf[24];

	snprintf(buf, sizeof(buf), "%d/%d", i + 1, count);
	UI_Badge(buf, UI_C_VALUE);

	/* %-10s 로 폭을 고정해서 이전 항목 글자가 남지 않게 한다 */
	UI_TextBig(4, 20, UI_C_ACCENT, UI_C_BG, "%-10s", name);

	UI_Text(4, 44, UI_C_LABEL, UI_C_BG, "gap");
	UI_Text(30, 44, UI_C_VALUE, UI_C_BG, "%-8ld mm", (long) gap);
}
