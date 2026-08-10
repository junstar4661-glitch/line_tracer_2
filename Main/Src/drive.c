/*
 * drive.c — 주행 로직만. 화면은 ui.c, 입력은 button.c(SysTick)가 담당한다.
 *
 *  주행 while 안에는 블로킹 함수도 GPIO 읽기도 없다.
 *    · 마커 판정  → ADC ISR (1kHz 고정)
 *    · 버튼 폴링  → SysTick ISR (1kHz), 루프는 플래그 하나만 읽는다
 *    · 화면 갱신  → 40ms마다 한 줄씩만
 */

#include "drive.h"
#include "sensor.h"
#include "motor.h"
#include "button.h"
#include "ui.h"

#define STEER_MAX           0.9f
#define LINE_LOST_STOP_MS   300
#define DRIVE_ROW_MS         40     // 화면은 한 번에 한 줄만

/* ── 속도 천장 (현재 코드 기준, 2026-08 실측 반영본) ──────────────
 *   MM_PER_STEP   = 52.0648 x pi / 400        = 0.4089 mm
 *   MOTOR_ARR_MIN = 150 → 1e6/151             = 6622 step/s
 *   물리 상한                                  = 2708 mm/s
 *   조향이 v*(1±STEER_MAX) 이므로
 *   ★바깥바퀴가 clamp에 안 걸리는 상한 = 2708 / 1.9 = 1425 mm/s★
 *   DRIVE_SPD_MAX 를 그 아래로 잡아둔다. 넘기면 명령 차동비 != 실제 차동비가
 *   되어 조향력이 소리없이 샌다 */
#define DRIVE_SPD_MIN         50
#define DRIVE_SPD_MAX       1400
#define DRIVE_SPD_STEP        10
#define DRIVE_SPD_STEP_FAST   50

/* ★ 주행 전 SETUP 화면에서 바꾼다. 빌드 없이 사다리를 올릴 수 있다 */
volatile int32_t g_drive1Speed = 700;    // Drive_First 기본 속도
volatile int32_t g_drive2Speed = 700;    // Drive_Second 최고 속도 (코너는 이것의 절반)

volatile int32_t g_steerK_x1e6 = 180;
#define STEER_K_STEP        5
#define STEER_K_STEP_FAST  25
#define STEER_K_MIN         0
#define STEER_K_MAX      2000

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

/* 주행 전 설정. K와 속도 둘 다 여기서 바꾼다.
 *   L / R      : 값 -, +      (길게 누르면 큰 폭으로)
 *   K 짧게     : 편집 대상 전환 (K <-> SPD)
 *   K 길게     : 확정하고 출발 */
static void Drive_Setup(const char *title, volatile int32_t *spd) {
	uint8_t sel = 0;
	int32_t drawnK = -1, drawnS = -1, drawnA = -1;
	uint8_t drawnSel = 0xFF;

	Button_Flush();
	UI_Setup_Frame(title);

	while (1) {
		UserInput_t b = Button_Get_Input();
		int32_t dir = 0;
		uint8_t fast = 0;

		switch (b) {
		case INPUT_CMD_L_SINGLE: dir = -1;            break;
		case INPUT_CMD_R_SINGLE: dir = +1;            break;
		case INPUT_CMD_L_HOLD:   dir = -1; fast = 1;  break;
		case INPUT_CMD_R_HOLD:   dir = +1; fast = 1;  break;
		case INPUT_CMD_K_SINGLE:
			sel = (uint8_t) ((sel + 1u) % UI_SETUP_ROWS);
			break;
		case INPUT_CMD_K_HOLD:   return;
		default: break;
		}

		if (dir) {
			if (sel == 0) {
				g_steerK_x1e6 += dir * (fast ? STEER_K_STEP_FAST : STEER_K_STEP);
				if (g_steerK_x1e6 < STEER_K_MIN) g_steerK_x1e6 = STEER_K_MIN;
				if (g_steerK_x1e6 > STEER_K_MAX) g_steerK_x1e6 = STEER_K_MAX;
			} else if (sel == 1) {
				*spd += dir * (fast ? DRIVE_SPD_STEP_FAST : DRIVE_SPD_STEP);
				if (*spd < DRIVE_SPD_MIN) *spd = DRIVE_SPD_MIN;
				if (*spd > DRIVE_SPD_MAX) *spd = DRIVE_SPD_MAX;
			} else {
				Ramp_Set_Accel(Ramp_Get_Accel()
						+ dir * (fast ? RAMP_ACCEL_FAST : RAMP_ACCEL_STEP));
			}
		}

		/* 값이 안 바뀌었는데 매 루프 다시 그리면 SPI가 버튼 응답을 잡아먹는다 */
		if (g_steerK_x1e6 != drawnK || *spd != drawnS
				|| Ramp_Get_Accel() != drawnA || sel != drawnSel) {
			drawnK = g_steerK_x1e6;
			drawnS = *spd;
			drawnA = Ramp_Get_Accel();
			drawnSel = sel;
			UI_Setup_Update(g_steerK_x1e6, *spd, drawnA, sel);
		}
	}
}

static uint8_t Drive_Precheck(const char *title, volatile int32_t *spd) {
	if (!Sensor_Is_Calibrated()) {
		UI_Banner("NO CAL", "run 2 calibrate first", UI_C_BAD, 1800);
		Button_Flush();
		return 0;
	}

	Drive_Setup(title, spd);

	for (int8_t s = 3; s > 0; s--) {
		UI_Countdown(title, s);
		HAL_Delay(1000);
	}

	Button_Flush();
	Button_Stop_Clear();
	return 1;
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
	int8_t drawn = -1;

	if (markLogCount == 0)
		return;

	Button_Flush();
	UI_Log_Frame();

	while (1) {
		UserInput_t btn_input = Button_Get_Input();

		if (i != drawn) {
			drawn = i;
			UI_Log_Update((uint8_t) i, markLogCount, MARK_NAME[markLog[i].type],
					markLog[i].distFromPrev);
		}

		if (btn_input == INPUT_CMD_L_SINGLE) {
			if (i > 0) i--;
		} else if (btn_input == INPUT_CMD_R_SINGLE) {
			if (i < (int8_t) markLogCount - 1) i++;
		} else if (btn_input == INPUT_CMD_K_HOLD) {
			break;
		}
	}
}

/* 주행 종료 후 결과창.
 * ★자동으로 안 닫힌다.★ marks / dist 를 받아적을 시간이 필요하다.
 *   K 짧게 = 메뉴로 나가기
 *   R 길게 = 로그 보기
 * K 길게(=주행 정지 버튼)는 무시한다. 정지 직후라 아직 눌려 있을 수 있고,
 * 그러면 창이 열리자마자 닫힌다 */
static void Drive_Result_Window(uint8_t ok, uint8_t marks, int32_t dist,
		uint8_t allowLog) {
	Button_Flush();
	UI_Drive_Result(ok, marks, dist,
			allowLog ? "R-hold:log    K:exit" : "K: exit");

	while (1) {
		UserInput_t b = Button_Get_Input();

		if (allowLog && b == INPUT_CMD_R_HOLD) {
			Drive_Log_Review();
			/* 로그를 보고 나오면 결과창을 다시 그려서 계속 머문다 */
			Button_Flush();
			UI_Drive_Result(ok, marks, dist, "R-hold:log    K:exit");
			continue;
		}
		if (b == INPUT_CMD_K_SINGLE)
			break;
	}
	Button_Flush();
}

void Drive_First() {
	markLogCount = 0;
	int32_t lastMarkDist = 0;
	uint8_t endCount = 0;
	uint32_t lostSince = 0;
	uint32_t lastRow = 0;
	uint8_t row = 0;
	uint8_t stoppedByLineLoss = 0;
	const char *lastMarkName = "-";
	MarkType_t mt;

	if (!Drive_Precheck("DRIVE 1st", &g_drive1Speed))
		return;

	Mark_FSM_Reset();
	Distance_Reset();
	Ramp_Reset();
	Sensor_Start();
	Motor_Start();
	Ramp_Start();
	UI_Drive_Frame("DRIVE 1st");

	while (1) {
		uint32_t now = HAL_GetTick();

		/* ── 순수 주행 로직. 블로킹 없음, GPIO 읽기 없음 ────── */
		Drive_Steer((float) g_drive1Speed);

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

		if (Mark_Consume(&mt)) {
			int32_t nowDist = Distance_Get_Mm();
			if (markLogCount < MARK_LOG_MAX) {
				markLog[markLogCount].type = mt;
				markLog[markLogCount].distFromPrev = nowDist - lastMarkDist;
				markLogCount++;
			}
			lastMarkDist = nowDist;
			lastMarkName = MARK_NAME[mt];

			if (mt == MARKTYPE_END)
				endCount++;
		}

		if (endCount >= 2)
			break;

		/* 버튼은 SysTick이 이미 읽었다. 여기선 플래그만 본다 */
		if (Button_Stop_Requested()) {
			stoppedByLineLoss = 1;   /* 사용자 취소도 즉시정지 */
			break;
		}
		/* ── 주행 로직 끝 ─────────────────────────────────── */

		if ((now - lastRow) >= DRIVE_ROW_MS) {
			lastRow = now;
			UI_Drive_Row(row, Distance_Get_Mm(), markLogCount, 0, lastMarkName,
					vCurL, vCurR, Sensor_Line_Found());
			row = (uint8_t) ((row + 1) & 0x3);
		}
	}

	if (stoppedByLineLoss)
		Drive_Stop_Immediate();
	else
		Drive_Stop_Graceful();
	Sensor_Stop();

	Drive_Result_Window((uint8_t) !stoppedByLineLoss, markLogCount,
			Distance_Get_Mm(), (uint8_t) (markLogCount > 0));
}

/* ★ 2차 최고속도도 SETUP 화면에서 바꾼다 (g_drive2Speed).
 *   코너 속도는 그 절반. 1차 속도를 올리면 2차도 같이 올려야
 *   "지도로 재현"이 성립한다 — 예전엔 300 고정이라 1차보다 느려지곤 했다 */
#define DRIVE2_TURN_RATIO   2
#define DECEL_DIST_MM     150

void Drive_Second() {
	if (markLogCount == 0) {
		UI_Banner("NO LOG", "run DRIVE 1st first", UI_C_BAD, 1800);
		Button_Flush();
		return;
	}

	uint8_t logIdx = 0;
	int32_t distSinceMark = 0;
	int32_t lastDist = 0;
	uint32_t lostSince = 0;
	uint32_t lastRow = 0;
	uint8_t row = 0;
	uint8_t stoppedByLineLoss = 0;
	MarkType_t mt;

	if (!Drive_Precheck("DRIVE 2nd", &g_drive2Speed))
		return;

	Mark_FSM_Reset();
	Distance_Reset();
	Ramp_Reset();
	Sensor_Start();
	Motor_Start();
	Ramp_Start();
	UI_Drive_Frame("DRIVE 2nd");

	while (1) {
		uint32_t now = HAL_GetTick();

		/* ── 순수 주행 로직. 블로킹 없음, GPIO 읽기 없음 ────── */
		int32_t nowDist = Distance_Get_Mm();
		distSinceMark += (nowDist - lastDist);
		lastDist = nowDist;

		int32_t distToNext = markLog[logIdx].distFromPrev - distSinceMark;
		float v_max = (float) g_drive2Speed;
		float v_turn = v_max / (float) DRIVE2_TURN_RATIO;
		float v_target;

		if (markLog[logIdx].type == MARKTYPE_CROSS
				|| distToNext > DECEL_DIST_MM) {
			v_target = v_max;
		} else if (distToNext > 0) {
			v_target = v_turn
					+ (v_max - v_turn) * ((float) distToNext / DECEL_DIST_MM);
		} else {
			v_target = v_turn;
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

		if (Mark_Consume(&mt)) {
			distSinceMark = 0;
			logIdx++;
			if (logIdx >= markLogCount)
				break;
		}

		if (Button_Stop_Requested()) {
			stoppedByLineLoss = 1;
			break;
		}
		/* ── 주행 로직 끝 ─────────────────────────────────── */

		if ((now - lastRow) >= DRIVE_ROW_MS) {
			lastRow = now;
			UI_Drive_Row(row, nowDist, logIdx, markLogCount,
					MARK_NAME[markLog[logIdx].type], vCurL, vCurR,
					Sensor_Line_Found());
			row = (uint8_t) ((row + 1) & 0x3);
		}
	}

	if (stoppedByLineLoss)
		Drive_Stop_Immediate();
	else
		Drive_Stop_Graceful();
	Sensor_Stop();

	Drive_Result_Window((uint8_t) !stoppedByLineLoss, markLogCount,
			Distance_Get_Mm(), 0);
}
