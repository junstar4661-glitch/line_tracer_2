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

#include <math.h>      /* fabsf  — PID 적분 와인드업 판정 */
#include <stdint.h>    /* INT32_MIN */

#define STEER_MAX           0.9f
/* ★ 시간(ms) 기준 → ★거리(mm)★ 기준으로 바꿨다.
 *   시간으로 잡으면 속도마다 이동거리가 달라져서 1.0~1.5 범위를 못 맞춘다.
 *      (예전 50ms:  1.0m/s → 5cm 인데 1.5m/s → 7.5cm)
 *   거리로 잡으면 ★어떤 속도든 항상 8cm★ 에서 멈춘다.
 *   튜닝 중 잦은 중단이 거슬리면 120으로, 대회 직전엔 50으로 조여라 */
#define LINE_LOST_STOP_MM    80
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
/* ★1425를 넘으면 코너에서 바깥바퀴가 천장(2708)에 걸려
 *   조향 효율이 절반이 된다. 못 가는 건 아니고 손해보면서 가는 것.
 *   목표가 1500 이상이라 상한을 물리 한계 근처까지 열어둔다 */
#define DRIVE_SPD_MAX       2500
#define DRIVE_SPD_STEP       100     /* L·R 짧게 */
#define DRIVE_SPD_STEP_FAST  500     /* L·R 길게 */

/* ★ 주행 전 SETUP 화면에서 바꾼다. 빌드 없이 사다리를 올릴 수 있다 */
volatile int32_t g_drive1Speed = 1500;   // Drive_First 기본 속도
volatile int32_t g_drive2Speed = 1500;   // Drive_Second 최고 속도 (코너는 이것의 절반)

/* ★ 센서 영점 보정 (p offset).
 *   센서바가 살짝 어긋났거나 좌우 캘리 밝기가 달라서, 라인 정중앙에 놔도
 *   p가 0이 안 나오는 경우가 있다. 그 값을 여기 넣으면 빼준다.
 *
 *   넣는 법: 메뉴 4 sen state 에서 라인 정중앙에 놓고 뜨는 p 값을 그대로.
 *            (sen state 화면은 계속 ★보정 전 날값★을 보여준다)
 *   단위는 p와 같은 0.01mm.  +150 = 오른쪽으로 1.5mm 치우쳐 읽히는 걸 상쇄 */
volatile int32_t g_pOffset = 0;
#define P_OFFSET_MIN   (-1500)
#define P_OFFSET_MAX   ( 1500)
#define P_OFFSET_STEP     10
#define P_OFFSET_FAST     50

/* ── PID 조향 ─────────────────────────────────────────────────
 *   오차 e     = 라인 중심에서 벗어난 거리 x100 (0.01mm)   ← 1kHz로 센서가 갱신
 *   변화량 d   = 1ms 동안의 e 변화             ← ★센서 갱신 시점에 차분★
 *
 *      s = ( Kp·e  +  Ki·∫e  +  Kd·d·100 ) x 1e-6
 *
 *   · Ki=Kd=0 이면 예전 P제어와 완전히 같다
 *   · 라인트레이서에서 ★I는 보통 해롭다.★ 긴 코너에서 적분이 쌓여
 *     코너 탈출 때 반대로 튄다. 0으로 두는 게 정석이다
 *   · D는 진동을 눌러줘서 Kp를 더 올릴 수 있게 해준다.
 *     "Kp 올리면 사행 / 내리면 이탈"이 될 때만 켜라
 *
 *   ★제어는 TIM7 인터럽트가 2kHz로 돌린다.★ 폴링이 아니다.
 *     화면을 그리든 버튼을 읽든 제어 주기는 흔들리지 않는다 */
#define PID_I_CLAMP    30000.0f      /* 적분 폭주 백스톱 */
#define PID_D_SCALE       100.0f     /* D 게인을 P와 비슷한 자릿수로 맞추는 배율 */

volatile int32_t g_kp = 180;
volatile int32_t g_ki = 0;
volatile int32_t g_kd = 0;

/* ★곡률 자동 감속★ — 많이 꺾을수록 알아서 느려진다.
 *   조향을 램프 밖으로 뺐기 때문에 바퀴 속도가 급변할 수 있는데,
 *   이게 그걸 억제하는 안전장치이자 코너 통과 성능 자체를 올려준다.
 *      v_reduced = v x CC / (|s|x1000 + CC)
 *   작을수록 많이 감속. 크면 거의 감속 안 함 */
volatile int32_t g_curveCoef = 1000;
#define CURVE_MIN        100
#define CURVE_MAX       5000
#define CURVE_STEP       100
#define CURVE_STEP_FAST  500

#define GAIN_P_MAX      2000
#define GAIN_I_MAX       500
#define GAIN_D_MAX      2000
#define GAIN_STEP         10     /* L·R 짧게 */
#define GAIN_STEP_FAST    50     /* L·R 길게 */

static volatile uint8_t ctrlOn = 0;   /* 0이면 조향 없이 직진만 */
static float   pidI = 0.0f;
static float   pidLastS = 0.0f;

/* ★진단용★ 주행 중 오차를 화면에 그대로 띄운다.
 *   pNow   = 지금 오차 (보정 후)
 *   pStart = 출발 직후 첫 오차. ★출발 튐의 원인을 여기서 본다★ */
volatile int32_t g_pNow = 0;
volatile int32_t g_pStart = 0;

static void Drive_PID_Reset(void) {
	pidI = 0.0f;
	pidLastS = 0.0f;
	g_pStart = 0;
	g_pNow = 0;
}

void Drive_Control_Enable(uint8_t on) {
	ctrlOn = on;
}

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
	Drive_Control_Enable(0);      /* 조향 끄고 */
	Ramp_Stop();
	Motor_Stop();
}

static void Drive_Stop_Graceful(void) {
	Drive_Control_Enable(0);      /* 감속 중엔 조향 안 한다 */
	Ramp_Set_Speed(0);
	HAL_Delay(500);
	Ramp_Stop();
	Motor_Stop();
}

/* 주행 전 설정. K와 속도 둘 다 여기서 바꾼다.
 *   L / R      : 값 -, +      (길게 누르면 큰 폭으로)
 *   K 짧게     : 편집 대상 전환 (K <-> SPD)
 *   K 길게     : 확정하고 출발 */
static int32_t clampi(int32_t v, int32_t lo, int32_t hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static void Drive_Setup(const char *title, volatile int32_t *spd) {
	uint8_t sel = 0, drawnSel = 0xFF;
	int32_t vals[UI_SET_COUNT], drawn[UI_SET_COUNT];

	for (uint8_t i = 0; i < UI_SET_COUNT; i++)
		drawn[i] = INT32_MIN;

	Button_Flush();
	UI_Setup_Frame(title);

	while (1) {
		UserInput_t b = Button_Get_Input();
		int32_t dir = 0;
		uint8_t fast = 0, changed = 0;

		switch (b) {
		case INPUT_CMD_L_SINGLE: dir = -1;            break;
		case INPUT_CMD_R_SINGLE: dir = +1;            break;
		case INPUT_CMD_L_HOLD:   dir = -1; fast = 1;  break;
		case INPUT_CMD_R_HOLD:   dir = +1; fast = 1;  break;
		case INPUT_CMD_K_SINGLE:
			sel = (uint8_t) ((sel + 1u) % UI_SET_COUNT);
			break;
		case INPUT_CMD_K_HOLD:   return;
		default: break;
		}

		if (dir) {
			int32_t gs = dir * (fast ? GAIN_STEP_FAST : GAIN_STEP);

			switch ((UI_SetupItem_t) sel) {
			case UI_SET_KP:
				g_kp = clampi(g_kp + gs, 0, GAIN_P_MAX);
				break;
			case UI_SET_KI:
				g_ki = clampi(g_ki + gs, 0, GAIN_I_MAX);
				break;
			case UI_SET_KD:
				g_kd = clampi(g_kd + gs, 0, GAIN_D_MAX);
				break;
			case UI_SET_SPD:
				*spd = clampi(*spd + dir * (fast ? DRIVE_SPD_STEP_FAST
						: DRIVE_SPD_STEP), DRIVE_SPD_MIN, DRIVE_SPD_MAX);
				break;
			case UI_SET_ACC:
				Ramp_Set_Accel(Ramp_Get_Accel()
						+ dir * (fast ? RAMP_ACCEL_FAST : RAMP_ACCEL_STEP));
				break;
			case UI_SET_DEC:
				Ramp_Set_Decel(Ramp_Get_Decel()
						+ dir * (fast ? RAMP_ACCEL_FAST : RAMP_ACCEL_STEP));
				break;
			case UI_SET_OFS:
				g_pOffset = clampi(g_pOffset + dir * (fast ? P_OFFSET_FAST
						: P_OFFSET_STEP), P_OFFSET_MIN, P_OFFSET_MAX);
				break;
			default:
				g_curveCoef = clampi(g_curveCoef + dir * (fast ? CURVE_STEP_FAST
						: CURVE_STEP), CURVE_MIN, CURVE_MAX);
				break;
			}
		}

		vals[UI_SET_KP]  = g_kp;
		vals[UI_SET_KI]  = g_ki;
		vals[UI_SET_KD]  = g_kd;
		vals[UI_SET_SPD] = *spd;
		vals[UI_SET_ACC] = Ramp_Get_Accel();
		vals[UI_SET_DEC] = Ramp_Get_Decel();
		vals[UI_SET_OFS]   = g_pOffset;
		vals[UI_SET_CURVE] = g_curveCoef;

		for (uint8_t i = 0; i < UI_SET_COUNT; i++)
			if (vals[i] != drawn[i]) { changed = 1; drawn[i] = vals[i]; }

		/* 값이 안 바뀌었는데 매 루프 다시 그리면 SPI가 버튼 응답을 잡아먹는다 */
		if (changed || sel != drawnSel) {
			drawnSel = sel;
			UI_Setup_Update(vals, sel);
		}
	}
}

/* ★★★ 임시 해제 스위치 ★★★
 *   7번 센서 고장으로 캘리가 항상 CAL FAIL 이라 주행 진입이 막힌다.
 *   센서 고치면 아래 줄의 // 를 지워서 다시 켜라.
 *
 *   #define DRIVE_REQUIRE_CAL       ← 이 줄을 살리면 원래대로 (캘리 필수)
 *
 *   해제 상태에서 알아둘 것
 *     · 위치계산은 1~6번만 쓰므로 ★주행은 정상 동작한다★
 *     · 마커는 bit0(왼쪽)만 잡힌다. RIGHT / END / CROSS 는 안 뜬다
 *     · END 가 안 뜨니 Drive_First 가 ★자동 종료를 못 한다★ → K 길게로 직접 정지
 *     · 그래서 1차주행 지도(markLog)는 신뢰할 수 없다. 2차주행은 하지 마라
 */
// #define DRIVE_REQUIRE_CAL

static uint8_t Drive_Precheck(const char *title, volatile int32_t *spd) {
#ifdef DRIVE_REQUIRE_CAL
	if (!Sensor_Is_Calibrated()) {
		UI_Banner("NO CAL", "run 2 calibrate first", UI_C_BAD, 1800);
		Button_Flush();
		return 0;
	}
#else
	/* 캘리 실패 상태로 달리고 있다는 걸 화면으로 계속 알린다 */
	if (!Sensor_Is_Calibrated())
		UI_Banner("NO CAL", "running anyway", UI_C_WARN, 1200);
#endif

	Drive_Setup(title, spd);

	/* ★ 카운트다운 중에 아무 버튼이나 짧게 누르면 취소된다.
	 *   값만 바꾸고 주행은 안 하고 싶을 때 쓰는 탈출구.
	 *   K 길게는 무시한다 — GO 누른 손가락이 아직 붙어 있을 수 있다 */
	for (int8_t s = 3; s > 0; s--) {
		uint32_t t0 = HAL_GetTick();
		UI_Countdown(title, s);
		while (HAL_GetTick() - t0 < 1000) {
			UserInput_t b = Button_Get_Input();
			if (b != INPUT_CMD_NONE && b != INPUT_CMD_K_HOLD) {
				UI_Banner("CANCELLED", "values kept", UI_C_WARN, 900);
				Button_Flush();
				return 0;
			}
		}
	}

	Button_Flush();
	Button_Stop_Clear();
	return 1;
}


/* ══ 제어 1틱 · TIM7이 2kHz로 부른다 ═══════════════════════
 *
 *   들어오는 vBase = 램프를 이미 통과한 기본속도
 *   여기서 하는 일 : PID → 곡률 감속 → 좌우 분배 → 실제 출력
 *
 *   ★조향은 램프를 안 탄다.★ 그래서 ACC가 낮아도 코너에서 즉시 꺾인다.
 *   대신 곡률 감속이 바퀴 속도 급변을 눌러준다
 * ═════════════════════════════════════════════════════════ */
void Drive_Control_Tick(float vBase) {
	int32_t e, d;
	float s, vRed;

	if (!ctrlOn) {                 /* 조향 꺼짐 = 그냥 직진 */
		Motor_Set_Wheels(vBase, vBase);
		return;
	}

	e = Sensor_Get_Position() - g_pOffset;   /* 영점 보정된 오차 */
	d = Sensor_Get_Delta();                  /* 센서 갱신 시점에 이미 뜬 차분 */

	if (g_pStart == 0 && e != 0)
		g_pStart = e;              /* 출발 순간의 오차를 박제 (진단용) */
	g_pNow = e;

	/* I: 조향이 이미 한계면 더 안 쌓는다 (적분 와인드업 방지) */
	if (fabsf(pidLastS) < STEER_MAX)
		pidI += (float) e * 0.0005f;          /* 2kHz → 틱당 0.5ms */
	if (pidI >  PID_I_CLAMP) pidI =  PID_I_CLAMP;
	if (pidI < -PID_I_CLAMP) pidI = -PID_I_CLAMP;

	s = ((float) g_kp * (float) e
	   + (float) g_ki * pidI
	   + (float) g_kd * (float) d * PID_D_SCALE) * 0.000001f;

	if (s > STEER_MAX)
		s = STEER_MAX;
	else if (s < -STEER_MAX)
		s = -STEER_MAX;
	pidLastS = s;

	/* ★곡률 자동 감속★ — 많이 꺾을수록 느려진다 */
	{
		float a = (s < 0) ? -s : s;
		float cc = (float) g_curveCoef;
		vRed = vBase * cc / (a * 1000.0f + cc);
	}

	Motor_Set_Wheels(vRed * (1.0f + s), vRed * (1.0f - s));
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
static void Drive_Result_Window(UI_EndReason_t reason, uint8_t marks,
		int32_t dist, uint8_t allowLog) {
	const char *hint = allowLog ? "R-hold:log    K:exit" : "K: exit";

	Button_Flush();
	UI_Drive_Result(reason, marks, dist, hint);

	while (1) {
		UserInput_t b = Button_Get_Input();

		if (allowLog && b == INPUT_CMD_R_HOLD) {
			Drive_Log_Review();
			/* 로그를 보고 나오면 결과창을 다시 그려서 계속 머문다 */
			Button_Flush();
			UI_Drive_Result(reason, marks, dist, hint);
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
	int32_t lostAtDist = 0;
	uint8_t lostFlag = 0;
	uint32_t lastRow = 0;
	uint8_t row = 0;
	UI_EndReason_t reason = UI_END_COMPLETE;
	const char *lastMarkName = "-";
	MarkType_t mt;

	if (!Drive_Precheck("DRIVE 1st", &g_drive1Speed))
		return;

	Mark_FSM_Reset();
	Sensor_Reset_Line();
	Drive_PID_Reset();
	Distance_Reset();
	Ramp_Reset();
	Sensor_Start();
	Motor_Start();
	Ramp_Set_Speed(0);            /* DRIVE 모드로 전환 + 기본속도 0에서 시작 */
	Drive_Control_Enable(1);      /* ★여기서부터 TIM7이 조향한다★ */
	Ramp_Start();
	UI_Drive_Frame("DRIVE 1st");

	while (1) {
		uint32_t now = HAL_GetTick();

		/* ── 순수 주행 로직. 블로킹 없음, GPIO 읽기 없음 ────── */
		/* ★PID는 2ms 고정 주기로만 돈다. D가 미분이라 주기가 흔들리면 안 된다 */
		/* ★조향은 TIM7(2kHz)이 한다. 루프는 목표속도만 알려준다★ */
		Ramp_Set_Speed((float) g_drive1Speed);

		if (Sensor_Line_Found()) {
			lostFlag = 0;              /* 라인 보임 → 리셋 */
		} else {
			if (!lostFlag) {
				lostFlag = 1;
				lostAtDist = Distance_Get_Mm();   /* 놓친 지점 기록 */
			} else if ((Distance_Get_Mm() - lostAtDist) > LINE_LOST_STOP_MM) {
				reason = UI_END_LINE_LOST;
				break;
			}
		}

		if (Mark_Consume(&mt)) {
			int32_t nowDist = Distance_Get_Mm();
			if (markLogCount < MARK_LOG_MAX) {
				markLog[markLogCount].type = mt;
				markLog[markLogCount].distFromPrev = nowDist - lastMarkDist;
				markLogCount++;
			} else {
				/* 기록칸이 꽉 찼다. 더 달려봐야 지도가 안 남는다 */
				reason = UI_END_LOG_FULL;
				break;
			}
			lastMarkDist = nowDist;
			lastMarkName = MARK_NAME[mt];

			if (mt == MARKTYPE_END)
				endCount++;
		}

		if (endCount >= 2)
			break;                      /* reason 은 COMPLETE 그대로 */

		/* 버튼은 SysTick이 이미 읽었다. 여기선 플래그만 본다 */
		if (Button_Stop_Requested()) {
			reason = UI_END_USER;
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

	/* 정상 완주만 부드럽게 감속. 실패는 즉시 전류 차단 */
	if (reason == UI_END_COMPLETE)
		Drive_Stop_Graceful();
	else
		Drive_Stop_Immediate();
	Sensor_Stop();

	Drive_Result_Window(reason, markLogCount,
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
	int32_t lostAtDist = 0;
	uint8_t lostFlag = 0;
	uint32_t lastRow = 0;
	uint8_t row = 0;
	UI_EndReason_t reason = UI_END_MAP_DONE;
	MarkType_t mt;

	if (!Drive_Precheck("DRIVE 2nd", &g_drive2Speed))
		return;

	Mark_FSM_Reset();
	Sensor_Reset_Line();
	Drive_PID_Reset();
	Distance_Reset();
	Ramp_Reset();
	Sensor_Start();
	Motor_Start();
	Ramp_Set_Speed(0);            /* DRIVE 모드로 전환 + 기본속도 0에서 시작 */
	Drive_Control_Enable(1);      /* ★여기서부터 TIM7이 조향한다★ */
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

		Ramp_Set_Speed(v_target);

		if (Sensor_Line_Found()) {
			lostFlag = 0;              /* 라인 보임 → 리셋 */
		} else {
			if (!lostFlag) {
				lostFlag = 1;
				lostAtDist = Distance_Get_Mm();   /* 놓친 지점 기록 */
			} else if ((Distance_Get_Mm() - lostAtDist) > LINE_LOST_STOP_MM) {
				reason = UI_END_LINE_LOST;
				break;
			}
		}

		if (Mark_Consume(&mt)) {
			distSinceMark = 0;
			logIdx++;
			if (logIdx >= markLogCount)
				break;              /* reason 은 MAP_DONE 그대로 */
		}

		if (Button_Stop_Requested()) {
			reason = UI_END_USER;
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

	if (reason == UI_END_MAP_DONE)
		Drive_Stop_Graceful();
	else
		Drive_Stop_Immediate();
	Sensor_Stop();

	Drive_Result_Window(reason, markLogCount, Distance_Get_Mm(), 0);
}
