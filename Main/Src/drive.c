/*
 * drive.c — 주행 로직만. 화면은 ui.c, 입력은 button.c(SysTick)가 담당한다.
 *
 *  주행 while 안에는 블로킹 함수도 GPIO 읽기도 없다.
 *    · 마커 판정  → ADC ISR (8개 센서 스캔 완료마다)
 *    · 버튼 폴링  → SysTick ISR (1kHz), 루프는 플래그 하나만 읽는다
 *    · 화면 갱신  → 40ms마다 한 줄씩만
 */

#include "drive.h"
#include "sensor.h"
#include "motor.h"
#include "button.h"
#include "ui.h"

#include <stddef.h>   /* NULL */
#include <math.h>     /* fabsf */

#define STEER_MAX           0.9f
/* At the initial 0.50 m/s test speed, 80 ms is 40 mm of blind travel.
 * After this continuous loss, both motor timers are stopped immediately. */
#define LINE_LOST_STOP_MS    80u
#define PIT_IN_ZERO_HOLD_MS  80u
#define PIT_IN_ALIGN_TOL_MM_S  2.0f
#define PIT_IN_ALIGN_TIMEOUT_MS 300u
#define PIT_IN_MIN_BRAKE_DIST_MM 20
#define DRIVE_ROW_MS         40     // 화면은 한 번에 한 줄만
#define DRIVE_STOP_TIMEOUT_MS 4000u

/* ── 속도 천장 및 바퀴 속도 한계 ────────────────────────────────
 *   MM_PER_STEP   = 52.0648 x pi / 400        = 0.4089 mm
 *   MOTOR_ARR_MIN = 150 → 1e6/151             = 6622 step/s
 *   물리 상한                                  = 2708 mm/s
 *   DRIVE 1st 메뉴는 직선 최고속도 2.0 m/s까지 허용한다.
 *   곡선에서 한쪽 바퀴 요구속도가 이 한계를 넘으면 Drive_Steer()가 좌·우
 *   속도를 같은 비율로 축소한다. 따라서 조향비는 유지되고 차속만 낮아진다. */
#define DRIVE_SPD_MIN         50
#define DRIVE_SPD_MAX       2000
#define DRIVE_SPD_STEP       100     /* L·R 짧게 */
#define DRIVE_SPD_STEP_FAST  500     /* L·R 길게 */
#define DRIVE_WHEEL_MAX_MM_S 2708.0f

/* Effective travel after the final END event until the wheel axle reaches
 * the desired pit-stop point.  The default is only a first estimate from
 * the previous master; tune it from actual pit-box error. */
#define FIT_IN_DIST_MIN_MM       50
#define FIT_IN_DIST_MAX_MM     1000
#define FIT_IN_DIST_STEP         10
#define FIT_IN_DIST_STEP_FAST    50

/* ★ 주행 전 SETUP 화면에서 바꾼다. 빌드 없이 사다리를 올릴 수 있다 */
volatile int32_t g_drive1Speed = 900;    // Drive_First initial speed: 0.90 m/s
volatile int32_t g_drive2Speed = 500;    // Drive_Second initial speed: 0.50 m/s
/* Calibrated effective distance from final END confirmation to the desired
 * pit-stop point.  The physical pit box stays 300 mm; this value includes
 * the FSM's late confirmation and the drivetrain's residual travel. */
volatile int32_t g_fitInDistMm = 265;

/* Default P-only test setting: Kp=14.5 1/m, Kd=0.0 s/m. */
volatile int32_t g_steerKp_x1e3 = 14500; // 14.500 1/m
volatile int32_t g_steerKd_x1e3 = 0;     // 0.000 s/m

#define STEER_KP_STEP        500
#define STEER_KP_STEP_FAST  2500
#define STEER_KP_MIN           0
#define STEER_KP_MAX      200000
#define STEER_KD_STEP         10
#define STEER_KD_STEP_FAST    50
#define STEER_KD_MIN           0
/* 2 ms sensor sampling makes 0.20 s/m excessively derivative-dominant.
 * Keep the setup range wide enough for tuning while preventing a decimal
 * point mistake from making the S-curve response unstable. */
#define STEER_KD_MAX         100   /* 0.100 s/m */

/* One complete 8-sensor frame arrives every 2 ms.  Updating P at that
 * cadence preserves the original master's corner-exit response while the
 * derivative path remains low-pass filtered. */
#define PD_CONTROL_PERIOD_MS       2u
/* 미분에 쓰는 dt. 이제 시계가 아니라 ★센서 프레임 완료 신호★로 돌기 때문에
 * 주기가 고정이고, 그래서 상수로 나눠도 된다 */
#define PD_CONTROL_PERIOD_S    0.002f
#define PD_DERIV_FILTER_ALPHA   0.25f
/* Keep only a sub-sensor deadband.  The P path remains unfiltered so a
 * sharp consecutive corner retains the full configured steering authority. */
#define STEER_CENTER_DEADBAND_M  0.0005f
/* D term alone cannot take over steering during a sharp S-curve reversal. */
#define PD_D_STEER_MAX          0.25f

/* ★전부 TIM7 인터럽트 안에서만 쓰인다★ */
static volatile float pdPrevPosition;
static volatile float pdDerivativeFiltered;
static volatile float pdSteer;
static volatile uint8_t pdReady;
/* A full-width marker can make the weighted position falsely look centred. */
static volatile uint8_t pdMarkerMasked;
/* 제어기 on/off. 꺼지면 조향 없이 직진만 한다 (정지·핏인 구간) */
static volatile uint8_t driveControlOn = 0;

/* 진단용 — 화면에 그대로 띄운다 */
volatile float g_steerNow = 0.0f;

/* ★구간(section).★  마커는 "점"이고 구간은 마커와 마커 "사이"다.
 *   맵 제작자가 마커를 코너와 직선의 경계에 놓았으므로,
 *   같은 방향 마커가 두 번 나오면 그 사이가 코너다.
 *     LEFT  … LEFT   →  사이가 좌커브
 *     RIGHT … RIGHT  →  사이가 우커브
 *     LEFT  … RIGHT  →  S자. 앞 코너의 끝이 뒷 코너의 시작이다
 *   CROSS·END 는 방향 정보가 없으므로 구간 상태를 건드리지 않는다 */
typedef enum {
	SEG_STRAIGHT = 0,
	SEG_CURVE_L,
	SEG_CURVE_R
} SegType_t;

#define MARK_LOG_MAX    50
typedef struct {
	MarkType_t type;
	int32_t distFromPrev;   /* 직전 마커 ~ 이 마커 거리 = 구간 길이 */
	SegType_t seg;          /* ★그 구간★의 속성 (이 마커 "앞" 구간) */
} MarkEntry_t;

MarkEntry_t markLog[MARK_LOG_MAX];      /* 1차에서 만든 ★지도★ */
uint8_t markLogCount = 0;

/* ★2차에서 ★실제로 본★ 마커 기록.★  지도(markLog)와 별개다.
 *   지도는 재시도에 계속 써야 하므로 절대 덮어쓰면 안 된다.
 *   실패했을 때 "지도엔 이랬는데 실제론 이랬다"를 비교하는 용도 */
static MarkEntry_t runLog[MARK_LOG_MAX];
static uint8_t runLogCount = 0;

static const char *MARK_NAME[] = { "LEFT", "RIGHT", "END", "CROSS" };
static const char *SEG_NAME[] = { "STRT", "CRV-L", "CRV-R" };

/* 마커 하나를 먹었을 때 구간 상태를 굴린다. 1차·2차가 같은 규칙을 써야
 * 실패 로그의 구간 표시가 지도와 같은 의미를 갖는다 */
static SegType_t Drive_Seg_Advance(SegType_t cur, MarkType_t mt) {
	if (mt == MARKTYPE_LEFT)
		return (cur == SEG_CURVE_L) ? SEG_STRAIGHT : SEG_CURVE_L;
	if (mt == MARKTYPE_RIGHT)
		return (cur == SEG_CURVE_R) ? SEG_STRAIGHT : SEG_CURVE_R;
	return cur;   /* CROSS·END 는 무시 */
}

static uint8_t Drive_Seg_Is_Curve(SegType_t s) {
	return (uint8_t) (s == SEG_CURVE_L || s == SEG_CURVE_R);
}

static int32_t Drive_Clamp_Gain(int32_t value, int32_t min, int32_t max);

static int32_t Drive_Clamp_Gain(int32_t value, int32_t min, int32_t max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

void Drive_Apply_ZN_PD(uint32_t ku_x1e3, uint32_t tu_ms) {
	uint64_t kp = ((uint64_t) ku_x1e3 * 8u + 5u) / 10u;
	/* Kd = Kp * Tu / 8 = 0.1 * Ku * Tu(seconds). */
	uint64_t kd = ((uint64_t) ku_x1e3 * tu_ms + 5000u) / 10000u;

	g_steerKp_x1e3 = Drive_Clamp_Gain((int32_t) kp, STEER_KP_MIN,
			STEER_KP_MAX);
	g_steerKd_x1e3 = Drive_Clamp_Gain((int32_t) kd, STEER_KD_MIN,
			STEER_KD_MAX);
}

static void Drive_PD_Reset(void) {
	pdPrevPosition = 0.0f;
	pdDerivativeFiltered = 0.0f;
	pdSteer = 0.0f;
	pdReady = 0;
	pdMarkerMasked = 0;
}

/* ★ 안전 정지: 라인이탈/사용자취소는 램프 없이 즉시 전류 차단.
 *   정상 완주(트랙 끝)만 부드럽게 감속 — 이미 트랙 안이라 안전함 */
static void Drive_Stop_Immediate(void) {
	Ramp_Stop();
	Motor_Stop();
}

static void Drive_Stop_Graceful(void) {
	Ramp_Set_Target(0, 0);

	/* A fixed 500 ms delay ignores the configured DEC.  Keep TIM7 running
	 * until the velocity ramp is at rest; a timeout preserves a safe stop if
	 * the timer service is unavailable. */
	uint32_t startedAt = HAL_GetTick();
	while ((vCurL > 0.5f || vCurL < -0.5f
			|| vCurR > 0.5f || vCurR < -0.5f)
			&& (HAL_GetTick() - startedAt) < DRIVE_STOP_TIMEOUT_MS) {
		/* TIM7 IRQ performs the deceleration ramp. */
	}
	Ramp_Stop();
	Motor_Stop();
}

/* Stop the mean wheel travel at FIT mm after the final END event.  The
 * normal DEC setting is restored afterwards; it remains the user's driving
 * deceleration, while pit-in uses v^2 = 2*a*s for this one terminal stop. */
static uint8_t Drive_Stop_Fit_In(int32_t fitInDistMm) {
	int32_t savedDecel = Ramp_Get_Decel();
	int32_t pitStartDist = Distance_Get_Mm();
	float straightSpeed = 0.5f * (fabsf(vCurL) + fabsf(vCurR));
	float vL;
	float vR;
	float meanV2;
	float minBrakeDist;
	float coastDist;
	int32_t pitDecel;
	uint32_t startedAt;

	fitInDistMm = Drive_Clamp_Gain(fitInDistMm, FIT_IN_DIST_MIN_MM,
			FIT_IN_DIST_MAX_MM);

	/* END may arrive while the normal line controller still has unequal wheel
	 * speeds.  First converge both wheels to their mean speed, which removes
	 * angular velocity and lets the chassis finish along its current tangent. */
	Ramp_Set_Target(straightSpeed, straightSpeed);
	startedAt = HAL_GetTick();
	while (fabsf(vCurL - straightSpeed) > PIT_IN_ALIGN_TOL_MM_S
			|| fabsf(vCurR - straightSpeed) > PIT_IN_ALIGN_TOL_MM_S) {
		if (Button_Stop_Requested()
				|| (HAL_GetTick() - startedAt) >= PIT_IN_ALIGN_TIMEOUT_MS) {
			return 0;
		}
	}

	/* The alignment phase is part of FIT travel, not an extra distance. */
	fitInDistMm -= Distance_Get_Mm() - pitStartDist;
	if (fitInDistMm < PIT_IN_MIN_BRAKE_DIST_MM)
		fitInDistMm = PIT_IN_MIN_BRAKE_DIST_MM;

	vL = fabsf(vCurL);
	vR = fabsf(vCurR);
	meanV2 = 0.5f * (vL * vL + vR * vR);
	minBrakeDist = (meanV2 > 0.0f)
			? meanV2 / (2.0f * (float) RAMP_DECEL_MIN) : 0.0f;

	/* Ramp_Set_Decel has a lower limit.  For a very long FIT distance, keep
	 * the terminal speed briefly, then apply that minimum deceleration. */
	coastDist = (float) fitInDistMm - minBrakeDist;
	if (coastDist > 0.0f) {
		int32_t coastStartDist = Distance_Get_Mm();
		Ramp_Set_Target(vCurL, vCurR);
		startedAt = HAL_GetTick();
		while ((float) (Distance_Get_Mm() - coastStartDist) < coastDist) {
			if (Button_Stop_Requested()
					|| (HAL_GetTick() - startedAt) >= DRIVE_STOP_TIMEOUT_MS) {
				Ramp_Set_Decel(savedDecel);
				return 0;
			}
		}
		pitDecel = RAMP_DECEL_MIN;
	} else {
		float requiredDecel = meanV2 / (2.0f * (float) fitInDistMm);
		pitDecel = (int32_t) (requiredDecel + 0.5f);
	}

	Ramp_Set_Decel(pitDecel);
	/* Keep the last energised phase at zero speed so holding torque absorbs
	 * residual chassis motion before the driver current is removed. */
	Motor_Set_Zero_Speed_Hold(1u);
	Ramp_Set_Target(0, 0);
	startedAt = HAL_GetTick();
	while (fabsf(vCurL) > 0.5f || fabsf(vCurR) > 0.5f) {
		if (Button_Stop_Requested()
				|| (HAL_GetTick() - startedAt) >= DRIVE_STOP_TIMEOUT_MS) {
			Ramp_Set_Decel(savedDecel);
			return 0;
		}
	}

	startedAt = HAL_GetTick();
	while ((HAL_GetTick() - startedAt) < PIT_IN_ZERO_HOLD_MS) {
		if (Button_Stop_Requested()) {
			Motor_Set_Zero_Speed_Hold(0u);
			Ramp_Set_Decel(savedDecel);
			return 0;
		}
	}

	Ramp_Stop();
	Motor_Set_Zero_Speed_Hold(0u);
	Motor_Stop();
	Ramp_Set_Decel(savedDecel);
	return 1;
}

/* 주행 전 설정. K와 속도 둘 다 여기서 바꾼다.
 *   L / R      : 값 -, +      (길게 누르면 큰 폭으로)
 *   K 짧게     : 편집 대상 전환 (K <-> SPD)
 *   K 길게     : 확정하고 출발 */
static void Drive_Setup(const char *title, volatile int32_t *spd) {
	uint8_t sel = 0;
	int32_t drawnKp = -1, drawnKd = -1, drawnS = -1, drawnA = -1, drawnD = -1;
	int32_t drawnFit = -1;
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
				g_steerKp_x1e3 += dir * (fast ? STEER_KP_STEP_FAST : STEER_KP_STEP);
				g_steerKp_x1e3 = Drive_Clamp_Gain(g_steerKp_x1e3,
						STEER_KP_MIN, STEER_KP_MAX);
			} else if (sel == 1) {
				g_steerKd_x1e3 += dir * (fast ? STEER_KD_STEP_FAST : STEER_KD_STEP);
				g_steerKd_x1e3 = Drive_Clamp_Gain(g_steerKd_x1e3,
						STEER_KD_MIN, STEER_KD_MAX);
			} else if (sel == 2) {
				*spd += dir * (fast ? DRIVE_SPD_STEP_FAST : DRIVE_SPD_STEP);
				if (*spd < DRIVE_SPD_MIN) *spd = DRIVE_SPD_MIN;
				if (*spd > DRIVE_SPD_MAX) *spd = DRIVE_SPD_MAX;
			} else if (sel == 3) {
				Ramp_Set_Accel(Ramp_Get_Accel()
						+ dir * (fast ? RAMP_ACCEL_FAST : RAMP_ACCEL_STEP));
			} else if (sel == 4) {
				Ramp_Set_Decel(Ramp_Get_Decel()
						+ dir * (fast ? RAMP_DECEL_FAST : RAMP_DECEL_STEP));
			} else {
				g_fitInDistMm += dir * (fast ? FIT_IN_DIST_STEP_FAST
						: FIT_IN_DIST_STEP);
				g_fitInDistMm = Drive_Clamp_Gain(g_fitInDistMm,
						FIT_IN_DIST_MIN_MM, FIT_IN_DIST_MAX_MM);
			}
		}

		/* 값이 안 바뀌었는데 매 루프 다시 그리면 SPI가 버튼 응답을 잡아먹는다 */
		if (g_steerKp_x1e3 != drawnKp || g_steerKd_x1e3 != drawnKd
				|| *spd != drawnS
				|| Ramp_Get_Accel() != drawnA || Ramp_Get_Decel() != drawnD
				|| g_fitInDistMm != drawnFit
				|| sel != drawnSel) {
			drawnKp = g_steerKp_x1e3;
			drawnKd = g_steerKd_x1e3;
			drawnS = *spd;
			drawnA = Ramp_Get_Accel();
			drawnD = Ramp_Get_Decel();
			drawnFit = g_fitInDistMm;
			drawnSel = sel;
			UI_Setup_Update(g_steerKp_x1e3, g_steerKd_x1e3, *spd, drawnA,
					drawnD, drawnFit, sel);
		}
	}
}

/* ★★★ 캘리 필수 여부 스위치 ★★★
 *   센서 하나가 고장나면 캘리가 항상 실패해서 주행 진입 자체가 막힌다.
 *   그 상태로도 K·속도 튜닝은 해야 하므로 기본은 ★해제★로 둔다.
 *
 *   아래 줄의 // 를 지우면 원래대로 (캘리 통과해야만 주행 가능)
 *
 *   해제 상태에서 알아둘 것
 *     · 위치계산은 1~6번만 쓰므로 ★주행은 정상 동작한다★
 *     · 죽은 센서가 0·7번이면 마커 판정이 망가진다
 *       → END 가 안 떠서 자동 종료를 못 하고, 지도도 못 믿는다
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
	/* 캘리 실패 상태로 달리고 있다는 걸 출발 전에 알린다 */
	if (!Sensor_Is_Calibrated())
		UI_Banner("NO CAL", "running anyway", UI_C_WARN, 1200);
#endif

	Drive_Setup(title, spd);

	/* ★ 3·2·1 세는 동안 아무 버튼이나 ★짧게★ 누르면 취소하고 메뉴로.
	 *   값만 바꾸고 주행은 안 하고 싶을 때 쓰는 탈출구다.
	 *   K 길게는 무시한다 — 방금 GO 누른 손가락이 아직 붙어 있을 수 있다 */
	for (int8_t s = 3; s > 0; s--) {
		uint32_t t0 = HAL_GetTick();

		UI_Countdown(title, s);
		while ((HAL_GetTick() - t0) < 1000u) {
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

/* PD 재계산. ★센서가 새 프레임을 완성했을 때만★ 부른다.
 *   dt 는 스캔 주기로 고정이라 미분값이 안 튄다 */
static void Drive_PD_Update(void) {
	int32_t p = Sensor_Get_Position();
	float rawError = (float) p / 100000.0f;
	float controlError;

	/* Keep the last valid steering while a marker is physically under the
	 * array.  On release the derivative reference is re-anchored, preventing
	 * the false marker position from producing a correction impulse. */
	if (Sensor_Marker_Active()) {
		pdDerivativeFiltered = 0.0f;
		pdMarkerMasked = 1;
		return;
	}

	if (rawError > STEER_CENTER_DEADBAND_M)
		controlError = rawError - STEER_CENTER_DEADBAND_M;
	else if (rawError < -STEER_CENTER_DEADBAND_M)
		controlError = rawError + STEER_CENTER_DEADBAND_M;
	else
		controlError = 0.0f;

	if (!pdReady || pdMarkerMasked) {
		/* 첫 프레임(또는 마커 통과 직후)은 미분 기준점만 잡는다.
		 * 직전값이 없는데 미분하면 500배 뻥튀기된 값이 나간다 */
		pdPrevPosition = rawError;
		pdDerivativeFiltered = 0.0f;
		pdMarkerMasked = 0;
		pdReady = 1;
		pdSteer = ((float) g_steerKp_x1e3 / 1000.0f) * controlError;
		return;
	}

	{
		float derivative = (rawError - pdPrevPosition) / PD_CONTROL_PERIOD_S;
		float kp = (float) g_steerKp_x1e3 / 1000.0f;
		float kd = (float) g_steerKd_x1e3 / 1000.0f;
		float dSteer;

		pdDerivativeFiltered += PD_DERIV_FILTER_ALPHA
				* (derivative - pdDerivativeFiltered);

		/* At the line-position sign change, a large Kd can otherwise command
		 * almost all available steering in the opposite direction. */
		dSteer = kd * pdDerivativeFiltered;
		if (dSteer > PD_D_STEER_MAX)
			dSteer = PD_D_STEER_MAX;
		else if (dSteer < -PD_D_STEER_MAX)
			dSteer = -PD_D_STEER_MAX;

		pdSteer = kp * controlError + dSteer;
		pdPrevPosition = rawError;
	}
}

/* ★TIM7 인터럽트(2kHz)가 부른다.★
 *   vBase 는 이미 램프를 통과한 값이고, 조향은 여기서 곱해진다.
 *   → 조향은 ACC/DEC 제한을 받지 않는다 (예전 구조의 핵심 병목이었다) */
void Drive_Control_Tick(float vBase) {
	float s, vL, vR, vPeak;

	if (!driveControlOn) {
		Motor_Set_Wheels(vBase, vBase);   /* 조향 없이 직진 */
		return;
	}

	/* 센서가 새 값을 내놨을 때만 PD를 갱신. 나머지 틱은 직전 조향을 유지 */
	if (Sensor_Take_Frame())
		Drive_PD_Update();

	s = pdSteer;
	if (s > STEER_MAX)
		s = STEER_MAX;
	else if (s < -STEER_MAX)
		s = -STEER_MAX;

	g_steerNow = s;

	/* At 2.0 m/s a sharp turn can request a wheel speed beyond the timer's
	 * 2.708 m/s limit.  Scale both wheels together instead of independently
	 * clamping the outer wheel: the requested turn ratio is preserved and the
	 * vehicle automatically slows only while steering demand is high. */
	vL = vBase * (1.0f + s);
	vR = vBase * (1.0f - s);
	vPeak = fmaxf(fabsf(vL), fabsf(vR));

	if (vPeak > DRIVE_WHEEL_MAX_MM_S) {
		float scale = DRIVE_WHEEL_MAX_MM_S / vPeak;
		vL *= scale;
		vR *= scale;
	}

	Motor_Set_Wheels(vL, vR);
}

void Drive_Control_Enable(uint8_t enable) {
	if (enable) {
		Drive_PD_Reset();
		driveControlOn = 1;
	} else {
		driveControlOn = 0;
		/* 조향을 끄고 나면 정지·핏인은 좌우 개별 제어가 필요하다 */
		Ramp_Mode_Manual_From_Current();
	}
}

/* 어느 기록이든 볼 수 있게 인자로 받는다.
 *   1차 → markLog (만든 지도)
 *   2차 → runLog  (실제로 본 것) */
/*   L 짧게/길게 = 위      R 짧게 = 아래      K 짧게 = 5칸 아래
 *   R 길게       = 보기 전환 (마커 ↔ 구간)
 *   K 길게       = 나가기
 *
 * 두 보기가 보여주는 것:
 *   LOG  →  이 지점의 ★마커 종류★ + 직전 마커부터의 거리
 *   SEG  →  거기까지 달려온 ★구간 종류★ + 그 구간 길이 */
static void Drive_Log_Review(const MarkEntry_t *log, uint8_t count) {
	int16_t sel = 0, top = 0;
	int16_t drawnSel = -1, drawnTop = -1;
	uint8_t segView = 0, drawnView = 0xFF;
	uint32_t holdSeen = 0;   /* HOLD 는 100ms마다 되풀이된다. 한 번만 먹는다 */

	if (log == NULL || count == 0)
		return;

	Button_Flush();
	UI_Log_Frame("LOG");

	while (1) {
		UserInput_t b = Button_Get_Input();
		uint32_t now = HAL_GetTick();

		switch (b) {
		case INPUT_CMD_L_SINGLE:
		case INPUT_CMD_L_HOLD:
			if (sel > 0) sel--;
			break;
		case INPUT_CMD_R_SINGLE:
			if (sel < (int16_t) count - 1) sel++;
			break;
		case INPUT_CMD_R_HOLD:
			/* 꾹 누르고 있는 동안 계속 뒤집히면 화면이 깜빡인다.
			 * 400ms 이상 쉬었다 눌러야 한 번 더 전환된다 */
			if (holdSeen == 0 || (now - holdSeen) > 400u)
				segView = (uint8_t) !segView;
			holdSeen = now;
			break;
		case INPUT_CMD_K_SINGLE:
			sel = (int16_t) (sel + UI_LOG_VISIBLE);
			if (sel > (int16_t) count - 1) sel = (int16_t) (count - 1);
			break;
		case INPUT_CMD_K_HOLD:
			return;
		default:
			break;
		}

		/* 선택이 창 밖으로 나가면 창을 끌어당긴다 */
		if (sel < top)
			top = sel;
		else if (sel >= top + UI_LOG_VISIBLE)
			top = (int16_t) (sel - (UI_LOG_VISIBLE - 1));

		if (segView != drawnView) {
			drawnView = segView;
			drawnSel = -1;                       /* 전체 다시 그리게 만든다 */
			UI_Log_Frame(segView ? "SEG" : "LOG");
		}

		if (sel != drawnSel || top != drawnTop) {
			drawnSel = sel;
			drawnTop = top;

			UI_Log_Head((uint8_t) sel, count);
			for (uint8_t r = 0; r < UI_LOG_VISIBLE; r++) {
				int16_t i = (int16_t) (top + r);

				if (i >= (int16_t) count) {
					UI_Log_Row(r, -1, NULL, 0, 0);   /* 남는 줄은 비운다 */
					continue;
				}

				UI_Log_Row(r, i,
						segView ? SEG_NAME[log[i].seg] : MARK_NAME[log[i].type],
						log[i].distFromPrev, (uint8_t) (i == sel));
			}
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
		int32_t dist, const MarkEntry_t *log, uint8_t logCount) {
	uint8_t allowLog = (uint8_t) (log != NULL && logCount > 0);
	const char *hint = allowLog ? "R-hold:log    K:exit" : "K: exit";

	Button_Flush();
	UI_Drive_Result(reason, marks, dist, hint);

	while (1) {
		UserInput_t b = Button_Get_Input();

		if (allowLog && b == INPUT_CMD_R_HOLD) {
			Drive_Log_Review(log, logCount);
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
	uint32_t lostSince = 0;
	uint32_t lastRow = 0;
	uint8_t row = 0;
	uint8_t stoppedByLineLoss = 0;
	uint8_t stoppedAtEnd = 0;
	UI_EndReason_t endReason = UI_END_COMPLETE;   /* 정상 완주가 기본 */
	const char *lastMarkName = "-";
	MarkType_t mt;
	SegType_t curSeg = SEG_STRAIGHT;   /* 출발선~첫 마커는 직선으로 본다 */

	if (!Drive_Precheck("DRIVE 1st", &g_drive1Speed))
		return;

	Mark_FSM_Reset();
	Distance_Reset();
	Ramp_Reset();
	Drive_PD_Reset();
	Sensor_Start();
	Motor_Start();
	/* Set a non-zero forward target before TIM7 starts.  Otherwise its first
	 * 0.5 ms tick sees zero targets and releases the holding phase we just set. */
	Ramp_Set_Speed((float) g_drive1Speed);   /* DRIVE 모드 진입 */
	Mark_Set_Speed(g_drive1Speed);           /* 병합창을 거리기준으로 */
	Ramp_Start();
	Drive_Control_Enable(1);                 /* ★여기서부터 TIM7이 조향한다★ */
	UI_Drive_Frame("DRIVE 1st");

	while (1) {
		uint32_t now = HAL_GetTick();

		/* ── 순수 주행 로직. 블로킹 없음, GPIO 읽기 없음 ──────
		 * ★조향은 TIM7 인터럽트가 한다. 여기선 기본속도만 넘긴다★ */
		Ramp_Set_Speed((float) g_drive1Speed);

		if (Sensor_Line_Found()) {
			lostSince = 0;
		} else {
			if (lostSince == 0)
				lostSince = now;
			else if ((now - lostSince) > LINE_LOST_STOP_MS) {
				stoppedByLineLoss = 1;
				endReason = UI_END_LINE_LOST;
				break;
			}
		}

		if (Mark_Consume(&mt)) {
			/* ★확정 시점이 아니라 마커가 시작된 지점★을 쓴다.
			 * 병합창만큼 확정이 늦는데 그 사이에도 로봇은 달린다 */
			int32_t nowDist = Mark_Get_Dist_Mm();
			if (markLogCount < MARK_LOG_MAX) {
				markLog[markLogCount].type = mt;
				markLog[markLogCount].distFromPrev = nowDist - lastMarkDist;
				/* ★상태를 굴리기 전 값이 "방금 지나온 구간"의 속성이다★ */
				markLog[markLogCount].seg = curSeg;
				markLogCount++;
			}
			curSeg = Drive_Seg_Advance(curSeg, mt);
			lastMarkDist = nowDist;
			lastMarkName = MARK_NAME[mt];

			if (mt == MARKTYPE_END)
				endCount++;
		}

		if (endCount >= 2) {
			stoppedAtEnd = 1;
			break;
		}

		/* 버튼은 SysTick이 이미 읽었다. 여기선 플래그만 본다 */
		if (Button_Stop_Requested()) {
			stoppedByLineLoss = 1;   /* 사용자 취소도 즉시정지 */
			endReason = UI_END_USER;
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

	/* ★조향을 끄고 MANUAL 로 승계한다.★
	 * 핏인·감속은 좌우를 개별로 몰아야 해서 DRIVE 모드로는 안 된다.
	 * 현재 바퀴속도를 그대로 물려받으므로 전환 순간 튐이 없다 */
	Drive_Control_Enable(0);

	if (stoppedByLineLoss)
		Drive_Stop_Immediate();
	else if (stoppedAtEnd && !Drive_Stop_Fit_In(g_fitInDistMm)) {
		stoppedByLineLoss = 1;
		Drive_Stop_Immediate();
	}
	else
		Drive_Stop_Graceful();
	Sensor_Stop();

	/* 핏인이 중간에 실패하면 이유를 사용자 취소로 승격한다 */
	if (stoppedByLineLoss && endReason == UI_END_COMPLETE)
		endReason = UI_END_USER;

	Drive_Result_Window(endReason, markLogCount, Distance_Get_Mm(),
			markLog, markLogCount);
}

/* ★ 2차 최고속도도 SETUP 화면에서 바꾼다 (g_drive2Speed).
 *   코너 속도는 그 절반. 1차 속도를 올리면 2차도 같이 올려야
 *   "지도로 재현"이 성립한다 — 예전엔 300 고정이라 1차보다 느려지곤 했다 */
#define DRIVE2_TURN_RATIO   2
#define DECEL_MARGIN_MM    20.0f

/* ★구간 단위 속도 결정.★
 *
 *   idx        = 지금 향해 가고 있는 마커 번호
 *   markLog[idx].seg  = ★지금 달리고 있는 구간★의 속성
 *   markLog[idx].distFromPrev = 그 구간의 전체 길이
 *
 *   커브 구간  →  구간 내내 코너속도. 마커 앞에서만 찔끔 줄이던 예전 방식과
 *                 다른 점이 여기다
 *   직선 구간  →  최고속. 단 다음이 커브면 제동거리 앞에서 미리 떨군다
 *
 *   짧은 직선을 vMax 로 잡으면 올리자마자 내려야 해서 덜덜거린다.
 *   그 직선에서 ★실제로 도달 가능한 속도★를 먼저 구한다:
 *
 *     가속거리 d, 감속거리 L-d, 양끝 속도 vTurn 일 때
 *       vPeak² = vTurn² + 2·ACC·DEC·L / (ACC + DEC)
 */
static float Drive_Segment_Speed(uint8_t idx, int32_t distToNext, float vMax,
		float vTurn) {
	float acc = (float) Ramp_Get_Accel();
	float dec = (float) Ramp_Get_Decel();
	float L, vPeak, brake;
	uint8_t nextIsSlow;

	if (Drive_Seg_Is_Curve(markLog[idx].seg))
		return vTurn;

	/* 마지막 마커(END)로 향하는 중이면 핏인 정지가 기다린다.
	 * 최고속으로 들이받으면 v²=2as 로 계산한 정지거리를 넘긴다 */
	nextIsSlow = (uint8_t) ((idx + 1u >= markLogCount)
			|| Drive_Seg_Is_Curve(markLog[idx + 1u].seg));

	if (!nextIsSlow)
		return vMax;

	L = (float) markLog[idx].distFromPrev;
	if (acc <= 0.0f || dec <= 0.0f || L <= 0.0f)
		return vTurn;

	vPeak = sqrtf(vTurn * vTurn + 2.0f * acc * dec * L / (acc + dec));
	if (vPeak > vMax)
		vPeak = vMax;

	brake = (vPeak * vPeak - vTurn * vTurn) / (2.0f * dec) + DECEL_MARGIN_MM;

	return ((float) distToNext <= brake) ? vTurn : vPeak;
}

void Drive_Second() {
	if (markLogCount == 0) {
		UI_Banner("NO LOG", "run DRIVE 1st first", UI_C_BAD, 1800);
		Button_Flush();
		return;
	}

	uint8_t logIdx = 0;
	int32_t lastMarkDist = 0;   /* 직전 마커가 ★시작된★ 지점 */
	uint32_t lostSince = 0;
	uint32_t lastRow = 0;
	uint8_t row = 0;
	uint8_t stoppedByLineLoss = 0;
	uint8_t stoppedByTrackMismatch = 0;
	uint8_t stoppedAtEnd = 0;
	UI_EndReason_t endReason = UI_END_MAP_DONE;   /* 지도 재생 완료가 기본 */
	MarkType_t mt;
	SegType_t runSeg = SEG_STRAIGHT;   /* 1차와 같은 규칙으로 다시 굴린다 */

	if (!Drive_Precheck("DRIVE 2nd", &g_drive2Speed))
		return;

	runLogCount = 0;          /* 이번 주행에서 본 것만 남긴다 */
	Mark_FSM_Reset();
	Distance_Reset();
	Ramp_Reset();
	Drive_PD_Reset();
	Sensor_Start();
	Motor_Start();
	/* Set a non-zero forward target before TIM7 starts.  Otherwise its first
	 * 0.5 ms tick sees zero targets and releases the holding phase we just set. */
	Ramp_Set_Speed((float) g_drive2Speed);   /* DRIVE 모드 진입 */
	Ramp_Start();
	Drive_Control_Enable(1);                 /* ★여기서부터 TIM7이 조향한다★ */
	UI_Drive_Frame("DRIVE 2nd");

	while (1) {
		uint32_t now = HAL_GetTick();

		/* ── 순수 주행 로직. 블로킹 없음, GPIO 읽기 없음 ────── */
		int32_t nowDist = Distance_Get_Mm();
		int32_t distSinceMark = nowDist - lastMarkDist;
		int32_t distToNext = markLog[logIdx].distFromPrev - distSinceMark;
		float v_max = (float) g_drive2Speed;
		float v_turn = v_max / (float) DRIVE2_TURN_RATIO;
		float v_target = Drive_Segment_Speed(logIdx, distToNext, v_max, v_turn);

		/* ★조향은 TIM7 인터럽트가 한다. 여기선 기본속도만 넘긴다★ */
		Ramp_Set_Speed(v_target);
		/* 병합창을 거리기준으로 유지한다. 2차는 속도가 계속 바뀌므로 매번 */
		Mark_Set_Speed((int32_t) v_target);

		if (Sensor_Line_Found()) {
			lostSince = 0;
		} else {
			if (lostSince == 0)
				lostSince = now;
			else if ((now - lostSince) > LINE_LOST_STOP_MS) {
				stoppedByLineLoss = 1;
				endReason = UI_END_LINE_LOST;
				break;
			}
		}

		if (Mark_Consume(&mt)) {
			MarkType_t expectedType = markLog[logIdx].type;
			int32_t markDist = Mark_Get_Dist_Mm();   /* 마커가 시작된 지점 */

			/* ★실제로 본 것을 그대로 남긴다.★ 종류가 어긋나서 멈추는
			 * 경우에도 "무엇을 봤는지"가 기록돼야 원인을 찾는다.
			 * 거리는 지도값이 아니라 ★이번에 실제로 달린 거리★다 */
			if (runLogCount < MARK_LOG_MAX) {
				runLog[runLogCount].type = mt;
				runLog[runLogCount].distFromPrev = markDist - lastMarkDist;
				runLog[runLogCount].seg = runSeg;   /* 방금 지나온 구간 */
				runLogCount++;
			}
			runSeg = Drive_Seg_Advance(runSeg, mt);

			/* The learned distance profile is only valid when the physical
			 * marker sequence repeats.  Do not advance the log on a false or
			 * missed marker, because that would apply later corner speeds at
			 * the wrong places. */
			if (mt != expectedType) {
				stoppedByTrackMismatch = 1;
				endReason = UI_END_MISMATCH;
				break;
			}
			lastMarkDist = markDist;
			logIdx++;
			if (logIdx >= markLogCount) {
				stoppedAtEnd = (uint8_t) (expectedType == MARKTYPE_END);
				break;
			}
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

	/* ★조향을 끄고 MANUAL 로 승계한다.★
	 * 핏인·감속은 좌우를 개별로 몰아야 해서 DRIVE 모드로는 안 된다.
	 * 현재 바퀴속도를 그대로 물려받으므로 전환 순간 튐이 없다 */
	Drive_Control_Enable(0);

	if (stoppedByLineLoss || stoppedByTrackMismatch)
		Drive_Stop_Immediate();
	else if (stoppedAtEnd && !Drive_Stop_Fit_In(g_fitInDistMm)) {
		stoppedByLineLoss = 1;
		Drive_Stop_Immediate();
	}
	else
		Drive_Stop_Graceful();
	Sensor_Stop();

	if (stoppedByLineLoss && endReason == UI_END_MAP_DONE)
		endReason = UI_END_USER;

	/* ★2차 결과에는 "지도"가 아니라 "실제로 본 것"을 띄운다.★
	 *   marks 도 지도 크기가 아니라 실제 통과한 개수여야 어디서 끊겼는지 보인다 */
	Drive_Result_Window(endReason, runLogCount, Distance_Get_Mm(),
			runLog, runLogCount);
}
