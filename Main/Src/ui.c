/*
 * ui.c - LCD 공통 화면 부품
 */

#include "ui.h"
#include "custom_lcd.h"

void UI_Screen(const char *title, const char *hint) {
	Custom_LCD_Clear();
	Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-13s", title);
	if (hint)
		Custom_LCD_Printf(UI_C_LABEL "%-13s", hint);
}


void UI_Hint(const char *hint) {
	Custom_LCD_Printf("/7" UI_SMALL UI_C_LABEL "%-26s", hint);
}

void UI_Bar(uint8_t value, uint8_t max, uint8_t width, const char *fillColor) {
	char buf[32];
	uint8_t i, n;

	if (width > 30) width = 30;
	if (max == 0) max = 1;
	if (value > max) value = max;

	n = (uint8_t) (((uint32_t) value * width) / max);

	for (i = 0; i < n; i++) buf[i] = '#';
	buf[n] = '\0';
	Custom_LCD_Printf("%s%s", fillColor, buf);

	for (i = 0; i < (uint8_t) (width - n); i++) buf[i] = '.';
	buf[width - n] = '\0';
	Custom_LCD_Printf(UI_C_DIM "%s", buf);
}

void UI_CenterBar(int32_t value, int32_t range, uint8_t width) {
	char left[32], right[32];
	uint8_t i, pos;
	int32_t v = value;

	if (width < 5) width = 5;
	if (width > 29) width = 29;
	if (range <= 0) range = 1;
	if (v > range) v = range;
	if (v < -range) v = -range;

	pos = (uint8_t) (((v + range) * (width - 1)) / (2 * range));

	for (i = 0; i < pos; i++)
		left[i] = (i == width / 2) ? '|' : '-';
	left[pos] = '\0';

	for (i = (uint8_t) (pos + 1); i < width; i++)
		right[i - pos - 1] = (i == width / 2) ? '|' : '-';
	right[width - pos - 1] = '\0';

	Custom_LCD_Printf(UI_C_DIM "[%s", left);
	Custom_LCD_Printf(UI_C_ACCENT "O");
	Custom_LCD_Printf(UI_C_DIM "%s]", right);
}

void UI_Bits(uint8_t bits, uint8_t count) {
	for (uint8_t i = 0; i < count; i++) {
		if (bits & (1u << i))
			Custom_LCD_Printf(UI_C_OK "O");
		else
			Custom_LCD_Printf(UI_C_DIM "-");
		if (i + 1 < count)
			Custom_LCD_Printf(" ");
	}
}

void UI_Banner(const char *l1, const char *l2, const char *color, uint32_t ms) {
	Custom_LCD_Clear();
	Custom_LCD_Printf("/0" UI_BIG "%s%-18s", color, l1);
	if (l2)
		Custom_LCD_Printf("/1" UI_SMALL UI_C_VALUE "%-26s", l2);
	if (ms)
		HAL_Delay(ms);
}
