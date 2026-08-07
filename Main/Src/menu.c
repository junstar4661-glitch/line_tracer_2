/*
 * menu.c
 *
 *  Created on: 2026. 6. 24.
 *      Author: kth59
 */

#include "menu.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "custom_lcd.h"
#include "button.h"
#include "ui.h"

#define MENU_ROWS	4

typedef struct {
	char name[12];
	void (*func)(void);
} Menu_TypeDef;

// @formatter:off
Menu_TypeDef MENU[] = {
		{ .name = "sensor raw" },
		{ .name = "calibrate"  },
		{ .name = "sen norm"   },
		{ .name = "sen state"  },
		{ .name = "mtr phase"  },
		{ .name = "mtr speed"  },
		{ .name = "DRIVE 1st"  },
		{ .name = "DRIVE 2nd"  },
};
// @formatter:on

static void Menu_Cell(uint8_t i, int8_t sel) {
	if (i == (uint8_t) sel)
		Custom_LCD_Printf(UI_C_ACCENT ">%d %-10s", i + 1, MENU[i].name);
	else
		Custom_LCD_Printf(UI_C_LABEL " %d %-10s", i + 1, MENU[i].name);
}

void Main_Menu() {
	static uint8_t init_done = 0;
	static int8_t idx = 0;
	static int8_t drawnIdx = -1;
	static uint8_t drawnCal = 0xFF;
	static uint8_t frameDrawn = 0;
	static uint32_t lastMoveTime = 0;

	if (!init_done) {
		Custom_LCD_Init(LCD_TYPE_ST7735);
		UI_Banner("LINE TRACER", "ZETIN system up", UI_C_TITLE, 800);
		init_done = 1;
		frameDrawn = 0;
	}

	uint8_t max_menu_size = sizeof(MENU) / sizeof(MENU[0]);
	UserInput_t btn_input = Button_Get_Input();

	if (!frameDrawn) {
		Custom_LCD_Clear();
		frameDrawn = 1;
		drawnIdx = -1;
		drawnCal = 0xFF;
	}

	uint8_t cal = Sensor_Is_Calibrated();
	if (idx != drawnIdx || cal != drawnCal) {
		Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-13s", "LINE TRACER");
		if (cal)
			Custom_LCD_Printf(UI_C_OK "%-13s", "CAL OK");
		else
			Custom_LCD_Printf(UI_C_BAD "%-13s", "NO CAL");

		for (uint8_t i = 0; i < MENU_ROWS; i++) {
			uint8_t left = i;
			uint8_t right = (uint8_t) (i + MENU_ROWS);

			Custom_LCD_Printf("/%d" UI_SMALL, i + 1);
			Menu_Cell(left, idx);
			if (right < max_menu_size)
				Menu_Cell(right, idx);
		}
		drawnIdx = idx;
		drawnCal = cal;
	}

	if (btn_input != INPUT_CMD_NONE) {
		switch (btn_input) {
		case INPUT_CMD_L_SINGLE: {
			uint32_t now = HAL_GetTick();
			if (now - lastMoveTime < 150) break;
			lastMoveTime = now;
			idx -= 1;
			if (idx < 0)
				idx = (int8_t) (max_menu_size - 1);
			break;
		}

		case INPUT_CMD_R_SINGLE: {
			uint32_t now = HAL_GetTick();
			if (now - lastMoveTime < 150) break;
			lastMoveTime = now;
			idx += 1;
			if (idx >= (int8_t) max_menu_size)
				idx = 0;
			break;
		}

		case INPUT_CMD_K_SINGLE:
			frameDrawn = 0;
			if (idx == 0)
				Sensor_Test_Raw();
			else if (idx == 1)
				Sensor_Calibration();
			else if (idx == 2)
				Sensor_Test_Normalized();
			else if (idx == 3)
				Sensor_Test_State();
			else if (idx == 4)
				Motor_Phase_Test();
			else if (idx == 5)
				Motor_Test();
			else if (idx == 6)
				Drive_First();
			else if (idx == 7)
				Drive_Second();
			break;

		default:
			break;
		}
	}
}
