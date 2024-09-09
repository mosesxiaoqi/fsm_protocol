//
// Created by 王鹤 on 2021/4/14.
//

#include "stdlib.h"
#include "string.h"
#include "simple_fsm.h"


struct Transition
{
    uint32_t id_next_state;
    fsm_action action;
};


struct FSMCB
{
    uint32_t current_state;

    uint8_t state_capacity;
    uint8_t event_capacity;

    uint8_t state_count;
    uint8_t event_count;

    uint32_t* state_cache;
    uint32_t* event_cache;

    uint32_t id;

    struct Transition** transitions;
};

static int16_t index_content(uint32_t* cache, uint32_t content, uint8_t cache_len)
{
    int16_t index = -1;
    for(uint8_t i = 0; i < cache_len; i++)
    {
        if(content == cache[i])
        {
            index = i;
            break;
        }
    }

    return index;
}


uint32_t fsm_current_state(void* fsm)
{
    return ((struct FSMCB*)fsm)->current_state;
}


void* fsm_create(uint8_t state_cnt, uint8_t event_cnt, uint32_t id)
{
    struct FSMCB* fsm_cb = (struct FSMCB*)malloc(sizeof(struct FSMCB));
    memset(fsm_cb, 0, sizeof(struct FSMCB));

    fsm_cb->state_capacity = state_cnt;
    fsm_cb->event_capacity = event_cnt;
    fsm_cb->state_count = 0;
    fsm_cb->event_count = 0;
    fsm_cb->id = id;

    fsm_cb->state_cache = (uint32_t*)malloc(sizeof(uint32_t) * fsm_cb->state_capacity);
    memset(fsm_cb->state_cache, 0, state_cnt);
    fsm_cb->event_cache = (uint32_t*)malloc(sizeof(uint32_t) * fsm_cb->event_capacity);
    memset(fsm_cb->event_cache, 0, event_cnt);

    uint32_t trans_table_size = sizeof(struct Transition*) * fsm_cb->state_capacity * fsm_cb->event_capacity;
    fsm_cb->transitions = (struct Transition**)malloc(trans_table_size);
    memset(fsm_cb->transitions, 0, trans_table_size);

    return fsm_cb;
}


void fsm_reg_state(void* fsm, uint32_t state_id, uint8_t is_init_state)
{
    struct FSMCB* pfsm_cb = (struct FSMCB*)fsm;
    int16_t stt_id_idx = index_content(pfsm_cb->state_cache, state_id, pfsm_cb->state_capacity);
    if(stt_id_idx < 0)
    {
        stt_id_idx = pfsm_cb->state_count;
    }
    pfsm_cb->state_cache[stt_id_idx] = state_id;
    pfsm_cb->state_count += 1;

    if(is_init_state)
    {
        pfsm_cb->current_state = state_id;
    }

    return;
}


void fsm_reg_event(void* fsm, uint32_t event_id)
{
    struct FSMCB* pfsm_cb = (struct FSMCB*)fsm;
    int16_t evt_id_idx = index_content(pfsm_cb->event_cache, event_id, pfsm_cb->event_capacity);
    if(evt_id_idx < 0)
    {
        evt_id_idx = pfsm_cb->event_count;
    }
    pfsm_cb->event_cache[evt_id_idx] = event_id;
    pfsm_cb->event_count += 1;

    return;
}


void fsm_reg_transaction(void* fsm, uint32_t cur_state_id, uint32_t nxt_state_id, uint32_t evt_id, fsm_action action)
{
    struct FSMCB* pfsm_cb = (struct FSMCB*)fsm;

    int16_t idx_cur_state = index_content(pfsm_cb->state_cache, cur_state_id, pfsm_cb->state_capacity);
    int16_t idx_nxt_state = index_content(pfsm_cb->state_cache, nxt_state_id, pfsm_cb->state_capacity);
    int16_t idx_evt = index_content(pfsm_cb->event_cache, evt_id, pfsm_cb->event_capacity);

    if(idx_cur_state < 0 || idx_nxt_state < 0 || idx_evt < 0)
    {
        return;
    }

    struct Transition* trans = (struct Transition*)malloc(sizeof(struct Transition));
    trans->id_next_state = nxt_state_id;
    trans->action = action;

    uint32_t trans_offset = idx_cur_state * pfsm_cb->event_capacity + idx_evt;
    pfsm_cb->transitions[trans_offset] = trans;

    return;
}


void fsm_on_event(void* fsm, uint32_t event_id)
{
    struct FSMCB* pfsm_cb = (struct FSMCB*)fsm;
    uint32_t cur_state_id = pfsm_cb->current_state;

    uint32_t idx_evt = index_content(pfsm_cb->event_cache, event_id, pfsm_cb->event_capacity);
    int16_t idx_cur_state = index_content(pfsm_cb->state_cache, cur_state_id, pfsm_cb->state_capacity);

    if(idx_evt < 0 || idx_cur_state < 0)
    {
        return;
    }

    uint32_t trans_offset = idx_cur_state * pfsm_cb->event_capacity + idx_evt;
    struct Transition* trans = pfsm_cb->transitions[trans_offset];
    if(trans != 0)
    {
        if(trans->action != 0)
        {
            trans->action(pfsm_cb->id, event_id, trans->id_next_state);
        }
        pfsm_cb->current_state = trans->id_next_state;
    }

    return;
}