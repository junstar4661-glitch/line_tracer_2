#include "drive.h"
#include "sensor.h"
#include "motor.h"
#include "custom_lcd.h"
#include "button.h"
#include "menu.h"
#include "ui.h"

#define DRIVE1_SPEED        150.0f
#define STEER_MAX           0.9f
#define LINE_LOST_STOP_MS   300
#define DRIVE_DRAW_MS        120

volatile int32_t g_steerK_x1e6 = 180;
#define STEER_K_STEP   5
#define STEER_K_MIN    0
#define STEER_K_MAX    2000

#define MARK_LOG_MAX    50
typedef struct {
	MarkType_t type;
	int32_t distFromPrev;
} MarkEntry_t;

MarkEntry_t markLog[MARK_LOG_MAX];
uint8_t markLogCount = 0;

static const char *MARK_NAME[] = { "LEFT", "RIGHT", "END", "CROSS" };

/* ★ 안전 정지: 라인이탈/사용자취소는 램프 없이 즉시 전류 차단.
 *   정상 완주(트랙 끝)만 부드럽게 감속 — 이미 트랙 안이라 안전함 */
static void Drive_Stop_Immediate(void) {
	Ramp_Stop();
	Motor_Stop();
}

static void Drive_Stop_Graceful(void) {
	Ramp_Set_Target(0, 0);
	HAL_Delay(500);
	Ramp_Stop();
	Motor_Stop();
}

static void Drive_Tune_SteerK(void) {
	uint32_t lastAct = 0;

	Custom_LCD_Clear();
	Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-26s", "TUNE STEER_K");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		if (btn_input == INPUT_CMD_L_SINGLE && (now - lastAct) > 80) {
			g_steerK_x1e6 -= STEER_K_STEP;
			if (g_steerK_x1e6 < STEER_K_MIN) g_steerK_x1e6 = STEER_K_MIN;
			lastAct = now;
		} else if (btn_input == INPUT_CMD_R_SINGLE && (now - lastAct) > 80) {
			g_steerK_x1e6 += STEER_K_STEP;
			if (g_steerK_x1e6 > STEER_K_MAX) g_steerK_x1e6 = STEER_K_MAX;
			lastAct = now;
		}

		Custom_LCD_Printf("/1" UI_BIG UI_C_VALUE "0.000%03ld", (long) g_steerK_x1e6);
		Custom_LCD_Printf("/2" UI_SMALL UI_C_LABEL "%-26s",
				(g_steerK_x1e6 < 100) ? "low = gentle" :
				(g_steerK_x1e6 > 300) ? "high = twitchy" : "mid = balanced");
		Custom_LCD_Printf("/3" UI_SMALL UI_C_LABEL "%-26s", "L- R+  K-hold:OK");

		if (btn_input == INPUT_CMD_K_HOLD)
			break;
	}
}

static uint8_t Drive_Precheck(const char *title) {
	if (!Sensor_Is_Calibrated()) {
		UI_Banner("NO CAL", "run 2 calibrate first", UI_C_BAD, 1800);
		return 0;
	}

	Drive_Tune_SteerK();

	for (int8_t s = 3; s > 0; s--) {
		Custom_LCD_Clear();
		Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-26s", title);
		Custom_LCD_Printf("/1" UI_BIG UI_C_ACCENT "  START IN  %d", s);
		Custom_LCD_Printf("/2" UI_SMALL UI_C_LABEL "%-26s", "on the start line");
		HAL_Delay(1000);
	}
	Custom_LCD_Clear();
	return 1;
}

static void Drive_Frame(const char *title) {
	UI_Screen(title, "K-hold:STOP");
}

static void Drive_Draw(int32_t dist, float v, uint8_t markIdx, uint8_t markTotal,
		const char *markName, uint8_t lineOk) {
	Custom_LCD_Printf("/1" UI_BIG UI_C_VALUE "%6ld", (long) dist);
	Custom_LCD_Printf(UI_C_LABEL "mm  ");

	Custom_LCD_Printf("/2" UI_SMALL UI_C_LABEL "mk ");
	Custom_LCD_Printf(UI_C_VALUE "%d", markIdx);
	if (markTotal)
		Custom_LCD_Printf(UI_C_LABEL "/%d", markTotal);
	Custom_LCD_Printf(UI_C_ACCENT " %-8s", markName ? markName : "-");

	Custom_LCD_Printf("/3" UI_SMALL UI_C_LABEL "spd ");
	Custom_LCD_Printf(UI_C_VALUE "%-4d", (int) v);
	Custom_LCD_Printf(UI_C_LABEL "mm/s   ");

	Custom_LCD_Printf("/4" UI_SMALL UI_C_LABEL "line ");
	if (lineOk)
		Custom_LCD_Printf(UI_C_OK "%-15s", "ON TRACK");
	else
		Custom_LCD_Printf(UI_C_BAD "%-15s", "LOST -> STOP");
}

static void Drive_Steer(float v_base) {
	int32_t p = Sensor_Get_Position();
	float steerK = (float) g_steerK_x1e6 / 1000000.0f;
	float s = (float) p * steerK;

	if (s > STEER_MAX)
		s = STEER_MAX;
	else if (s < -STEER_MAX)
		s = -STEER_MAX;

	Ramp_Set_Target(v_base * (1.0f + s), v_base * (1.0f - s));
}

static void Drive_Log_Review(void) {
	int8_t i = 0;

	if (markLogCount == 0)
		return;

	while (1) {
		UserInput_t btn_input = Button_Get_Input();

		Custom_LCD_Clear();
		Custom_LCD_Printf("/0" UI_SMALL UI_C_TITLE "%-26s", "LOG REVIEW");
		Custom_LCD_Printf("/1" UI_SMALL UI_C_LABEL "entry " UI_C_VALUE "%d" UI_C_LABEL "/%d",
				i + 1, markLogCount);
		Custom_LCD_Printf("/2" UI_BIG UI_C_ACCENT "%-10s", MARK_NAME[markLog[i].type]);
		Custom_LCD_Printf("/3" UI_SMALL UI_C_LABEL "gap " UI_C_VALUE "%-6ld mm",
				(long) markLog[i].distFromPrev);
		Custom_LCD_Printf("/4" UI_SMALL UI_C_LABEL "%-26s", "L/R:move  K-hold:exit");

		if (btn_input == INPUT_CMD_L_SINGLE) {
			if (i > 0) i--;
		} else if (btn_input == INPUT_CMD_R_SINGLE) {
			if (i < (int8_t) markLogCount - 1) i++;
		} else if (btn_input == INPUT_CMD_K_HOLD) {
			break;
		}
	}
}

void Drive_First() {
	markLogCount = 0;
	int32_t lastMarkDist = 0;
	uint8_t endCount = 0;
	uint32_t lostSince = 0;
	uint32_t lastDraw = 0;
	uint8_t stoppedByLineLoss = 0;
	const char *lastMarkName = "-";

	if (!Drive_Precheck("DRIVE 1st")) {
		Main_Menu();
		return;
	}

	Mark_FSM_Reset();
	Distance_Reset();
	Ramp_Reset();
	Sensor_Start();
	Motor_Start();
	Ramp_Start();
	Drive_Frame("DRIVE 1st");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		Mark_FSM_Tick();

		Drive_Steer(DRIVE1_SPEED);

		if (Sensor_Line_Found()) {
			lostSince = 0;
		} else {
			if (lostSince == 0)
				lostSince = now;
			else if ((now - lostSince) > LINE_LOST_STOP_MS) {
				stoppedByLineLoss = 1;
				break;
			}
		}

		if (markLastResult) {
			int32_t nowDist = Distance_Get_Mm();
			if (markLogCount < MARK_LOG_MAX) {
				markLog[markLogCount].type = markLastType;
				markLog[markLogCount].distFromPrev = nowDist - lastMarkDist;
				markLogCount++;
			}
			lastMarkDist = nowDist;
			lastMarkName = MARK_NAME[markLastType];

			if (markLastType == MARKTYPE_END)
				endCount++;
		}

		if ((now - lastDraw) >= DRIVE_DRAW_MS) {
			lastDraw = now;
			Drive_Draw(Distance_Get_Mm(), vCurL, markLogCount, 0, lastMarkName,
					Sensor_Line_Found());
		}

		if (endCount >= 2)
			break;

		if (btn_input == INPUT_CMD_K_HOLD) {
			stoppedByLineLoss = 1;   /* 사용자 취소도 즉시정지 */
			break;
		}
	}

	if (stoppedByLineLoss)
		Drive_Stop_Immediate();
	else
		Drive_Stop_Graceful();
	Sensor_Stop();

	if (stoppedByLineLoss)
		UI_Banner("STOPPED", "line lost / cancelled", UI_C_WARN, 0);
	else
		UI_Banner("DRIVE1 DONE", NULL, UI_C_OK, 0);
	Custom_LCD_Printf("/2" UI_SMALL UI_C_LABEL "marks " UI_C_VALUE "%-16d", markLogCount);
	Custom_LCD_Printf("/3" UI_SMALL UI_C_LABEL "dist  " UI_C_VALUE "%-12ld mm", (long) Distance_Get_Mm());
	Custom_LCD_Printf("/4" UI_SMALL UI_C_LABEL "%-26s", "R-hold:view log");

	{
		uint32_t t0 = HAL_GetTick();
		while (HAL_GetTick() - t0 < 4000) {
			UserInput_t b = Button_Get_Input();
			if (b == INPUT_CMD_R_HOLD) {
				Drive_Log_Review();
				break;
			}
			if (b != INPUT_CMD_NONE)
				break;
		}
	}
	Main_Menu();
}

#define DRIVE2_V_MAX      300.0f
#define DRIVE2_V_TURN     150.0f
#define DECEL_DIST_MM     150

void Drive_Second() {
	if (markLogCount == 0) {
		UI_Banner("NO LOG", "run DRIVE 1st first", UI_C_BAD, 1800);
		Main_Menu();
		return;
	}

	uint8_t logIdx = 0;
	int32_t distSinceMark = 0;
	int32_t lastDist = 0;
	uint32_t lostSince = 0;
	uint32_t lastDraw = 0;
	uint8_t stoppedByLineLoss = 0;

	if (!Drive_Precheck("DRIVE 2nd")) {
		Main_Menu();
		return;
	}

	Mark_FSM_Reset();
	Distance_Reset();
	Ramp_Reset();
	Sensor_Start();
	Motor_Start();
	Ramp_Start();
	Drive_Frame("DRIVE 2nd");

	while (1) {
		UserInput_t btn_input = Button_Get_Input();
		uint32_t now = HAL_GetTick();

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
			v_target = DRIVE2_V_TURN
					+ (DRIVE2_V_MAX - DRIVE2_V_TURN)
							* ((float) distToNext / DECEL_DIST_MM);
		} else {
			v_target = DRIVE2_V_TURN;
		}

		Drive_Steer(v_target);

		if (Sensor_Line_Found()) {
			lostSince = 0;
		} else {
			if (lostSince == 0)
				lostSince = now;
			else if ((now - lostSince) > LINE_LOST_STOP_MS) {
				stoppedByLineLoss = 1;
				break;
			}
		}

		if (markLastResult) {
			distSinceMark = 0;
			logIdx++;
			if (logIdx >= markLogCount)
				break;
		}

		if ((now - lastDraw) >= DRIVE_DRAW_MS) {
			lastDraw = now;
			Drive_Draw(nowDist, vCurL, logIdx, markLogCount,
					MARK_NAME[markLog[logIdx].type], Sensor_Line_Found());
		}

		if (btn_input == INPUT_CMD_K_HOLD) {
			stoppedByLineLoss = 1;
			break;
		}
	}

	if (stoppedByLineLoss)
		Drive_Stop_Immediate();
	else
		Drive_Stop_Graceful();
	Sensor_Stop();

	if (stoppedByLineLoss)
		UI_Banner("STOPPED", "line lost / cancelled", UI_C_WARN, 0);
	else
		UI_Banner("DRIVE2 DONE", NULL, UI_C_OK, 0);
	Custom_LCD_Printf("/2" UI_SMALL UI_C_LABEL "dist " UI_C_VALUE "%-13ld mm", (long) Distance_Get_Mm());
	HAL_Delay(2000);
	Main_Menu();
}
