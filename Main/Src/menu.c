#include "menu.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "button.h"
#include "ui.h"

typedef struct {
	const char *name;
	void (*func)(void);
} Menu_TypeDef;

// @formatter:off
static const Menu_TypeDef MENU[] = {
		{ "sensor raw", Sensor_Test_Raw        },
		{ "calibrate",  Sensor_Calibration     },
		{ "sen norm",   Sensor_Test_Normalized },
		{ "sen state",  Sensor_Test_State      },
		{ "mtr phase",  Motor_Phase_Test       },
		{ "mtr speed",  Motor_Test             },
		{ "DRIVE 1st",  Drive_First            },
		{ "DRIVE 2nd",  Drive_Second           },
};
// @formatter:on

#define MENU_COUNT   ((uint8_t)(sizeof(MENU) / sizeof(MENU[0])))

void Main_Menu(void) {
	static uint8_t init_done = 0;
	static int8_t idx = 0;
	static int8_t drawnIdx = -1;
	static uint8_t drawnCal = 0xFF;
	static uint8_t frameDrawn = 0;
	static uint32_t lastMoveTime = 0;

	if (!init_done) {
		UI_Init();
		
		/* --- Custom Boot Screen ---
		 * ★ 패널은 160x80 이다. y 좌표가 80을 넘으면 아무것도 안 보인다.
		 *   (예전 배치는 트랙선 y=100, 바퀴 y=95 라 로봇이 통째로 화면 밖이었다)
		 *   아래는 전부 y < 76 안에 들어간다 */
		UI_Clear();

		// 1. Creators (English since font may not support Hangul)
		UI_Text(6,  1, UI_C_LABEL,  UI_C_BG, "Made by:");
		UI_Text(6, 15, UI_C_ACCENT, UI_C_BG, "JUNYOUNG KIM");
		UI_Text(6, 27, UI_C_ACCENT, UI_C_BG, "YONGJUN LEE");

		// 2. Line tracer on the track (측면도, 오른쪽이 진행방향)
		UI_Fill(0, 70, 160, 2, UI_C_VALUE);   // 트랙 노면
		UI_Fill(0, 72, 160, 1, UI_C_DIM);     // 노면 그림자

		UI_Fill(52, 50, 52, 14, UI_C_TITLE);  // 차체
		UI_Fill(58, 44,  8,  6, UI_C_RULE);   // 차체 위 LCD

		UI_Fill(58, 64, 12,  6, UI_C_VALUE);  // 뒷바퀴
		UI_Fill(86, 64, 12,  6, UI_C_VALUE);  // 앞바퀴

		UI_Fill(104, 55, 18,  4, UI_C_LABEL); // 센서바 지지대
		UI_Fill(120, 52, 6,  10, UI_C_BAD);   // 센서바

		HAL_Delay(2000); // Wait 2 seconds so they can see the masterpiece
		/* -------------------------- */

		init_done = 1;
		frameDrawn = 0;
	}

	UserInput_t btn_input = Button_Get_Input();

	if (!frameDrawn) {
		UI_Clear();
		frameDrawn = 1;
		drawnIdx = -1;
		drawnCal = 0xFF;
	}

	uint8_t cal = Sensor_Is_Calibrated();
	if (idx != drawnIdx || cal != drawnCal) {
		const char *names[MENU_COUNT];
		for (uint8_t i = 0; i < MENU_COUNT; i++)
			names[i] = MENU[i].name;

		UI_Menu_Draw(names, MENU_COUNT, (uint8_t) idx, cal);
		drawnIdx = idx;
		drawnCal = cal;
	}

	switch (btn_input) {
	case INPUT_CMD_L_SINGLE: {
		uint32_t now = HAL_GetTick();
		if (now - lastMoveTime < 150) break;
		lastMoveTime = now;
		idx--;
		if (idx < 0)
			idx = (int8_t) (MENU_COUNT - 1);
		break;
	}

	case INPUT_CMD_R_SINGLE: {
		uint32_t now = HAL_GetTick();
		if (now - lastMoveTime < 150) break;
		lastMoveTime = now;
		idx++;
		if (idx >= (int8_t) MENU_COUNT)
			idx = 0;
		break;
	}

	case INPUT_CMD_K_SINGLE:
		frameDrawn = 0;
		if (MENU[idx].func)
			MENU[idx].func();
		Button_Flush();     /* ?�면?�서 빠져?�올 ???�려?�던 K가 메뉴�??��? ?�게 */
		break;

	default:
		break;
	}
}
