/*
 * button.c
 *
 *  Created on: 2026. 6. 25.
 *      Author: kth59
 */


#include "button.h"

#define BTN_DEBOUNCE_TIME       20
#define BTN_LONG_PRESS_TIME     500
#define BTN_DOUBLE_CLICK_GAP    250
#define BTN_LONG_PRESS_REPEAT   100

#define BTN_QUEUE_LEN           8       // 2의 거듭제곱

ButtonHandle_t btn_l;
ButtonHandle_t btn_r;
ButtonHandle_t btn_k;

static volatile UserInput_t btnQueue[BTN_QUEUE_LEN];
static volatile uint8_t     btnQHead = 0;   // ISR이 쓴다
static volatile uint8_t     btnQTail = 0;   // 메인이 읽는다
static volatile uint8_t     btnStopReq = 0;
static volatile uint8_t     btnPollReady = 0;

void Button_Init_Internal(ButtonHandle_t *btn, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState active_state) {
    btn->state = BTN_STATE_IDLE;
    btn->start_time = 0;
    btn->last_repeat_time = 0;
    btn->port = port;
    btn->pin = pin;
    btn->active_state = active_state;
}

static ButtonEvent_t Button_Get_Event(ButtonHandle_t *btn) {
    uint32_t now = HAL_GetTick();

    bool is_pressed = (HAL_GPIO_ReadPin(btn->port, btn->pin) == btn->active_state);

    ButtonEvent_t event = BTN_EVENT_NONE;

    switch (btn->state) {
        case BTN_STATE_IDLE:
            if (is_pressed) {
                btn->state = BTN_STATE_PRESSED;
                btn->start_time = now;
            }
            break;

        case BTN_STATE_PRESSED:
            if (!is_pressed) {
                if (now - btn->start_time >= BTN_DEBOUNCE_TIME) {
                    event = BTN_EVENT_SINGLE_CLICK;
                    btn->state = BTN_STATE_IDLE;
                } else {
                    btn->state = BTN_STATE_IDLE;
                }
            } else if (now - btn->start_time >= BTN_LONG_PRESS_TIME) {
                btn->state = BTN_STATE_LONG_PRESS;
                btn->last_repeat_time = now;
                event = BTN_EVENT_LONG_PRESS_HOLD;
            }
            break;

        case BTN_STATE_LONG_PRESS:
            if (!is_pressed) {
                btn->state = BTN_STATE_IDLE;
            } else {
                if (now - btn->last_repeat_time >= BTN_LONG_PRESS_REPEAT) {
                    event = BTN_EVENT_LONG_PRESS_HOLD;
                    btn->last_repeat_time = now;
                }
            }
            break;

        case BTN_STATE_WAIT_RELEASE:
            if (!is_pressed) {
                btn->state = BTN_STATE_IDLE;
            }
            break;
    }
    return event;
}

static void Button_Queue_Push(UserInput_t cmd) {
    uint8_t next = (uint8_t) ((btnQHead + 1u) & (BTN_QUEUE_LEN - 1u));

    if (next == btnQTail)
        return;                 // 가득 참. 가장 오래된 걸 지키고 새 걸 버린다

    btnQueue[btnQHead] = cmd;
    btnQHead = next;
}

/* SysTick(1kHz) 전용 */
void Button_Poll(void) {
    if (!btnPollReady)
        return;

    ButtonEvent_t evt_l = Button_Get_Event(&btn_l);
    ButtonEvent_t evt_r = Button_Get_Event(&btn_r);
    ButtonEvent_t evt_k = Button_Get_Event(&btn_k);

    if (evt_k == BTN_EVENT_LONG_PRESS_HOLD)
        btnStopReq = 1;         // 주행루프가 볼 정지 요청

    if (evt_k == BTN_EVENT_SINGLE_CLICK)         Button_Queue_Push(INPUT_CMD_K_SINGLE);
    else if (evt_k == BTN_EVENT_LONG_PRESS_HOLD) Button_Queue_Push(INPUT_CMD_K_HOLD);

    if (evt_l == BTN_EVENT_SINGLE_CLICK)         Button_Queue_Push(INPUT_CMD_L_SINGLE);
    else if (evt_l == BTN_EVENT_LONG_PRESS_HOLD) Button_Queue_Push(INPUT_CMD_L_HOLD);

    if (evt_r == BTN_EVENT_SINGLE_CLICK)         Button_Queue_Push(INPUT_CMD_R_SINGLE);
    else if (evt_r == BTN_EVENT_LONG_PRESS_HOLD) Button_Queue_Push(INPUT_CMD_R_HOLD);
}

UserInput_t Button_Get_Input(void) {
    UserInput_t cmd;

    if (btnQTail == btnQHead)
        return INPUT_CMD_NONE;

    cmd = btnQueue[btnQTail];
    btnQTail = (uint8_t) ((btnQTail + 1u) & (BTN_QUEUE_LEN - 1u));
    return cmd;
}

uint8_t Button_Stop_Requested(void) {
    return btnStopReq;
}

void Button_Stop_Clear(void) {
    btnStopReq = 0;
}

void Button_Flush(void) {
    btnQTail = btnQHead;
    btnStopReq = 0;
}

void Button_Wait_Release(ButtonHandle_t *btn) {
	while (HAL_GPIO_ReadPin(btn->port, btn->pin) == btn->active_state);
}

void MX_Button_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // K버튼: PC13
    GPIO_InitStruct.Pin = KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

    // L버튼: SWL
    GPIO_InitStruct.Pin = SWL_Pin;
    HAL_GPIO_Init(SWL_GPIO_Port, &GPIO_InitStruct);

    // R버튼: SWR
    GPIO_InitStruct.Pin = SWR_Pin;
    HAL_GPIO_Init(SWR_GPIO_Port, &GPIO_InitStruct);

    Button_Init_Internal(&btn_k, KEY_GPIO_Port, KEY_Pin, GPIO_PIN_SET);
    Button_Init_Internal(&btn_l, SWL_GPIO_Port, SWL_Pin, GPIO_PIN_SET);
    Button_Init_Internal(&btn_r, SWR_GPIO_Port, SWR_Pin, GPIO_PIN_SET);

    btnQHead = 0;
    btnQTail = 0;
    btnStopReq = 0;
    btnPollReady = 1;       // 여기부터 SysTick이 폴링해도 안전하다
}
