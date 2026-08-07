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

#define MENU_ROWS	4

typedef struct {
	char name[16];
	void (*func)(void);
} Menu_TypeDef;

// @formatter:off
Menu_TypeDef MENU[] = {
		{ .name = "sensor raw" },
		{ .name = "calibration" },
		{ .name = "sensor norm" },
		{ .name = "sensor state" },
		{ .name = "motor phase" },
		{ .name = "motor speed" },
		{ .name = "first drive" },
		{ .name = "second drive" },
};
// @formatter:on

void Main_Menu() {
	static uint8_t init_done = 0;
	static int8_t idx = 0;

	if (!init_done) {
		Custom_LCD_Init(LCD_TYPE_ST7735);
		Custom_LCD_Printf("/0/r/AHello world");
		Custom_LCD_Printf("/1Hello world");

		HAL_Delay(500);
		Custom_LCD_Clear();

		init_done = 1;
	}

	uint8_t max_menu_size = sizeof(MENU) / sizeof(MENU[0]);
	UserInput_t btn_input = Button_Get_Input();

	// 왼쪽 칸 = 0~3, 오른쪽 칸 = 4~6
	for (uint8_t i = 0; i < MENU_ROWS; i++) {
		uint8_t left = i;
		uint8_t right = i + MENU_ROWS;

		if (left == idx)
			Custom_LCD_Printf("/r");
		else
			Custom_LCD_Printf("/w");
		Custom_LCD_Printf("/%d%-13s", i, MENU[left].name);

		if (right < max_menu_size) {
			if (right == idx)
				Custom_LCD_Printf("/r");
			else
				Custom_LCD_Printf("/w");
			Custom_LCD_Printf("%-13s", MENU[right].name);
		}
	}

	if (btn_input != INPUT_CMD_NONE) {
		Custom_LCD_Printf("/5%2d", btn_input);

		switch (btn_input) {
		case INPUT_CMD_L_SINGLE:
			idx -= 1;
			if (idx < 0)
				idx = max_menu_size - 1;
			break;

		case INPUT_CMD_R_SINGLE:
			idx += 1;
			if (idx == max_menu_size)
				idx = 0;
			break;

		case INPUT_CMD_K_SINGLE:
			if (idx == 0){
			Sensor_Test_Raw();
			}
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
