#include "drive.h"
#include "sensor.h"
#include "motor.h"
#include "custom_lcd.h"
#include "button.h"
#include "menu.h"

#define DRIVE1_SPEED    150.0f   // 1차 주행 고정 저속 (mm/s)
#define STEER_K         0.00008f // 튜닝 상수, 작으면 둔감 클수록 예민

#define MARK_LOG_MAX    50
typedef struct {
	MarkType_t type;
	int32_t distFromPrev;
} MarkEntry_t;

MarkEntry_t markLog[MARK_LOG_MAX];
uint8_t markLogCount = 0;

void Drive_First() {
	markLogCount = 0;
	int32_t lastMarkDist = 0;
	uint8_t endCount = 0;

	Mark_FSM_Reset();
	Distance_Reset();
	Sensor_Start();
	Motor_Start();
	Ramp_Start();
	Custom_LCD_Clear();

	while (1) {
		UserInput_t btn_input = Button_Get_Input();

		Mark_FSM_Tick();

		int32_t p = Sensor_Get_Position();
		float vL = DRIVE1_SPEED * (1.0f + (float) p * STEER_K);
		float vR = DRIVE1_SPEED * (1.0f - (float) p * STEER_K);
		Ramp_Set_Target(vL, vR);

		if (markLastResult) {
			int32_t nowDist = Distance_Get_Mm();
			if (markLogCount < MARK_LOG_MAX) {
				markLog[markLogCount].type = markLastType;
				markLog[markLogCount].distFromPrev = nowDist - lastMarkDist;
				markLogCount++;
			}
			lastMarkDist = nowDist;

			if (markLastType == MARKTYPE_END) {
				endCount++;
			}

			const char *names[] = { "LEFT", "RIGHT", "END", "CROSS" };
			Custom_LCD_Printf("/0#%-3d%-6s", markLogCount, names[markLastType]);
			Custom_LCD_Printf("/1d=%-6ld", (long) Distance_Get_Mm());
		}

		if (endCount >= 2) {
			break;
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			break;
		}
	}

	Ramp_Set_Target(0, 0);
	HAL_Delay(500);
	Ramp_Stop();
	Motor_Stop();
	Sensor_Stop();

	Custom_LCD_Clear();
	Custom_LCD_Printf("/0DRIVE1 DONE");
	Custom_LCD_Printf("/1marks=%-4d", markLogCount);
	HAL_Delay(1500);
	Main_Menu();
}

#define DRIVE2_V_MAX      300.0f  // 직선 최대 속도 (mm/s)
#define DRIVE2_V_TURN     150.0f  // 코너 통과 속도 (mm/s)
#define DECEL_DIST_MM     150     // 코너 진입 몇 mm 전부터 감속 시작할지

void Drive_Second() {
	if (markLogCount == 0) {
		Custom_LCD_Clear();
		Custom_LCD_Printf("/0NO LOG! run");
		Custom_LCD_Printf("/1drive1 first");
		HAL_Delay(1500);
		Main_Menu();
		return;
	}

	uint8_t logIdx = 0;
	int32_t distSinceMark = 0;
	int32_t lastDist = 0;

	Mark_FSM_Reset();
	Distance_Reset();
	Sensor_Start();
	Motor_Start();
	Ramp_Start();
	Custom_LCD_Clear();

	while (1) {
		UserInput_t btn_input = Button_Get_Input();

		Mark_FSM_Tick();

		int32_t nowDist = Distance_Get_Mm();
		distSinceMark += (nowDist - lastDist);
		lastDist = nowDist;

		int32_t distToNext = markLog[logIdx].distFromPrev - distSinceMark;
		float v_target;

		if (markLog[logIdx].type == MARKTYPE_CROSS
				|| distToNext > DECEL_DIST_MM) {
			v_target = DRIVE2_V_MAX;
		} else if (distToNext > 0) {
			// 코너 접근 중: 선형으로 감속
			v_target = DRIVE2_V_TURN
					+ (DRIVE2_V_MAX - DRIVE2_V_TURN)
							* ((float) distToNext / DECEL_DIST_MM);
		} else {
			v_target = DRIVE2_V_TURN;
		}

		int32_t p = Sensor_Get_Position();
		float vL = v_target * (1.0f + (float) p * STEER_K);
		float vR = v_target * (1.0f - (float) p * STEER_K);
		Ramp_Set_Target(vL, vR);

		if (markLastResult) {
			distSinceMark = 0;
			logIdx++;

			Custom_LCD_Printf("/0#%-3d/%d", logIdx, markLogCount);
			Custom_LCD_Printf("/1d=%-6ld", (long) Distance_Get_Mm());

			if (logIdx >= markLogCount) {
				break;
			}
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			break;
		}
	}

	Ramp_Set_Target(0, 0);
	HAL_Delay(500);
	Ramp_Stop();
	Motor_Stop();
	Sensor_Stop();

	Custom_LCD_Clear();
	Custom_LCD_Printf("/0DRIVE2 DONE");
	Custom_LCD_Printf("/1d=%-6ld", (long) Distance_Get_Mm());
	HAL_Delay(1500);
	Main_Menu();
}
