/*
 * button.h
 *
 *  Created on: 2026. 6. 25.
 *      Author: kth59
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

typedef enum {
    INPUT_CMD_NONE = 0,

    INPUT_CMD_L_SINGLE,
    INPUT_CMD_L_DOUBLE,
    INPUT_CMD_L_HOLD,

    INPUT_CMD_R_SINGLE,
    INPUT_CMD_R_DOUBLE,
    INPUT_CMD_R_HOLD,

    INPUT_CMD_K_SINGLE,
    INPUT_CMD_K_DOUBLE,
    INPUT_CMD_K_HOLD,
} UserInput_t;

typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK,
    BTN_EVENT_LONG_PRESS_HOLD
} ButtonEvent_t;

typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_PRESSED,
    BTN_STATE_LONG_PRESS,
    BTN_STATE_WAIT_RELEASE
} ButtonState_t;

typedef struct {
    ButtonState_t state;
    uint32_t start_time;
    uint32_t last_repeat_time;

    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_state;
} ButtonHandle_t;

extern ButtonHandle_t btn_l;
extern ButtonHandle_t btn_r;
extern ButtonHandle_t btn_k;

void MX_Button_Init(void);

/* ★ SysTick(1kHz)에서만 호출한다. 여기서 세 버튼 FSM을 굴리고
 *   결과 이벤트를 큐에 넣는다. 메인루프는 GPIO를 직접 읽지 않는다.
 *   TIM7(램프)이 아니라 SysTick에 붙인 이유: TIM7은 주행/모터테스트에서만
 *   켜지므로 메뉴 화면에서 버튼이 죽는다. SysTick은 부팅부터 항상 돈다 */
void Button_Poll(void);

/* 메뉴·테스트 화면용. 큐에서 이벤트 1건을 꺼낸다 (없으면 NONE) */
UserInput_t Button_Get_Input(void);

/* ★ 주행루프 전용. 큐를 건드리지 않고 플래그 하나만 읽는다.
 *   K 롱프레스가 들어오면 ISR이 즉시 세운다 */
uint8_t Button_Stop_Requested(void);
void Button_Stop_Clear(void);

/* 큐에 남은 이벤트를 전부 버린다 (화면 전환 직후 잔여입력 제거용) */
void Button_Flush(void);

void Button_Wait_Release(ButtonHandle_t *btn);

#endif /* INC_BUTTON_H_ */
