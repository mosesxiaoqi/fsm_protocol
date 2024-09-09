//
// Created by 王鹤 on 2021/4/14.
//

#ifndef _SIMPLE_FSM_H_
#define _SIMPLE_FSM_H_

#include "stdint.h"

typedef void(*fsm_action)(uint32_t, uint32_t, uint32_t); // (uint32_t fsm_id, uint32_t event, uint32_t new_state)

void* fsm_create(uint8_t state_cnt, uint8_t event_cnt, uint32_t id);
uint32_t fsm_current_state(void* fsm);
void fsm_reg_state(void* fsm, uint32_t state_id, uint8_t is_init_state);
void fsm_reg_event(void* fsm, uint32_t event_id);
void fsm_reg_transaction(void* fsm, uint32_t cur_state_id, uint32_t nxt_state_id, uint32_t evt_id, fsm_action action);
void fsm_on_event(void* fsm, uint32_t event_id);


#endif //_SIMPLE_FSM_H_
