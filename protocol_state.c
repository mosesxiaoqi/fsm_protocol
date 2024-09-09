//
// Created by 王鹤 on 2021/5/31.
//

#include "string.h"
#include "simple_fsm.h"
#include "protocol_app.h"
#include "protocol_state.h"


#define BUFFER_LENGTH 512
#define HEADER_LENGTH 16
#define TAIL_LENGTH   4

#define HEAD_MAGIC 0x5748   //// little endian for 0x4857 to uint16
#define TAIL_MAGIC 0xFECA   //// little endian for 0xCAFE to uint16

struct ProtocolMCB
{
    void* protocol_fsm;

    uint8_t receiving_buf[BUFFER_LENGTH];
    uint8_t* buf_ready_pos;

    struct FrameHeader current_header;
    uint8_t frame_body[BUFFER_LENGTH];
    struct FrameTail current_tail;


    uint8_t sending_buf[BUFFER_LENGTH];
    struct FrameHeader sending_header;
    struct FrameTail sending_tail;

    uint32_t transmit_frame_count;
    uint8_t timeout_count;
};


static TIM_HandleTypeDef* pTransmissionTimer = 0;
static struct ProtocolMCB Protocol_MCBs[PROTOCOL_CHANNEL_COUNT];

extern void transmit_data(enum ProtocolChannel channel, uint8_t* data, uint16_t data_len);


enum ProtocolStateDef
{
    PS_WAITING_HEADER,
    PS_RECEIVING_HEADER,
    PS_WAITING_TAIL,
    PS_STATE_COUNT
};


enum ProtocolEventDef
{
    PE_HEADER_MAGIC_FRONT_APPEARANCE,
    PE_HEADER_MAGIC_APPEARANCE,

    PE_GET_VALID_HEADER,
    PE_GET_INVALID_HEADER,

    PE_TAIL_MAGIC_APPEARANCE,
    PE_GET_VALID_BODY,
    PE_GET_INVALID_BODY,

    PE_RECEIVING_TIMEOUT,

    PE_NONE,

    PE_EVENT_COUNT
};


static uint16_t crc_16(uint8_t* data, uint16_t len)
{
    uint16_t crc_reg = 0xffff;
    for(uint16_t i = 0; i < len; i++)
    {
        crc_reg ^= data[i];
        for(uint16_t j = 0; j < 8; j++)
        {
            if(crc_reg & 0x01)
            {
                crc_reg = ((crc_reg >> 1) ^ 0xa001);
            }
            else
            {
                crc_reg = crc_reg >> 1;
            }
        }
    }

    return crc_reg;
}


#pragma region export interface
__attribute__((weak)) void on_header_received(enum ProtocolChannel channel, const struct FrameHeader* const fh)
{
    return;
}


__attribute__((weak)) void on_frame_received(
enum ProtocolChannel channel,
const struct FrameHeader* const fh,
const struct FrameTail* const ft,
const uint8_t* const fb)
{
    return;
}
#pragma endregion


#pragma region state machine actions
static void action_clear_buffer(uint32_t protocol_channel, uint32_t event, uint32_t new_state)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[protocol_channel];
    pmcb->buf_ready_pos = pmcb->receiving_buf;
    __HAL_TIM_SET_COUNTER(pTransmissionTimer, 0);
    pmcb->timeout_count = 0;
    memset(pmcb->receiving_buf, 0, BUFFER_LENGTH);

    return;
}


static void action_notify_header(uint32_t protocol_channel, uint32_t event, uint32_t new_state)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[protocol_channel];
    memcpy((uint8_t*)&pmcb->current_header, pmcb->receiving_buf, sizeof(struct FrameHeader));
    on_header_received(protocol_channel, &pmcb->current_header);
    return;
}


static void action_notify_frame(uint32_t protocol_channel, uint32_t event, uint32_t new_state)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[protocol_channel];
    memcpy((uint8_t*)&pmcb->current_tail, pmcb->buf_ready_pos - 4, sizeof(struct FrameTail));
    memcpy(pmcb->frame_body, pmcb->receiving_buf + HEADER_LENGTH, pmcb->current_header.body_length);
    on_frame_received(protocol_channel, &pmcb->current_header, &pmcb->current_tail, pmcb->frame_body);
    action_clear_buffer(protocol_channel, event, new_state);

    return;
}


static void action_disposal_invalid_head(uint32_t protocol_channel, uint32_t event, uint32_t new_state)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[protocol_channel];
    struct FrameHeader* header = (struct FrameHeader*)pmcb->receiving_buf;
    uint32_t seq_no = header->sequence;
    action_clear_buffer(protocol_channel, event, new_state);
    enum NWStatus status = NW_STATUS_HEAD_CRC_ERROR;
    perform_intent(protocol_channel, INTENT_CFM, seq_no, (uint8_t*)&status, 4);

    return;
}


static void action_disposal_invalid_body(uint32_t protocol_channel, uint32_t event, uint32_t new_state)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[protocol_channel];
    uint32_t seq_no = pmcb->current_header.sequence;
    action_clear_buffer(protocol_channel, event, new_state);
    enum NWStatus status = NW_STATUS_BODY_CRC_ERROR;
    perform_intent(protocol_channel, INTENT_CFM, seq_no, (uint8_t*)&status, 4);

    return;
}
#pragma endregion


#pragma region state machine events process
typedef enum ProtocolEventDef(*fp_ck_event)(enum ProtocolChannel);

static enum ProtocolEventDef ck_wait_header_event(enum ProtocolChannel channel)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[channel];

    uint32_t ready_pos = pmcb->buf_ready_pos - pmcb->receiving_buf;
    if(ready_pos == 1 && *pmcb->receiving_buf == (HEAD_MAGIC & 0x00FF))
    {
        return PE_HEADER_MAGIC_FRONT_APPEARANCE;
    }
    else if(ready_pos == 2 && *(uint16_t*)pmcb->receiving_buf == HEAD_MAGIC)
    {
        return PE_HEADER_MAGIC_APPEARANCE;
    }
    else if(pmcb->timeout_count > 0)
    {
        return PE_RECEIVING_TIMEOUT;
    }
    else
    {
        return PE_NONE;
    }
}


static enum ProtocolEventDef ck_receiving_header_event(enum ProtocolChannel channel)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[channel];
    if(pmcb->buf_ready_pos - pmcb->receiving_buf == HEADER_LENGTH)
    {
        uint16_t r_crc = ((struct FrameHeader*)pmcb->receiving_buf)->crc;
        uint16_t c_crc = crc_16(pmcb->receiving_buf, HEADER_LENGTH - 2);

        if(r_crc == c_crc)
        {
            return PE_GET_VALID_HEADER;
        }
        else
        {
            return PE_GET_INVALID_HEADER;
        }
    }
    else if(pmcb->timeout_count > 0)
    {
        return PE_RECEIVING_TIMEOUT;
    }
    else
    {
        return PE_NONE;
    }
}


static enum ProtocolEventDef ck_waiting_tail_event(enum ProtocolChannel channel)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[channel];
    struct FrameHeader* buf_header = (struct FrameHeader*)pmcb->receiving_buf;
    if(pmcb->buf_ready_pos - pmcb->receiving_buf == buf_header->body_length + HEADER_LENGTH + TAIL_LENGTH)
    {
        struct FrameTail* buf_tail = (struct FrameTail*)(pmcb->receiving_buf + HEADER_LENGTH + buf_header->body_length);
        uint16_t r_crc = buf_tail->crc;
        uint16_t c_crc = 0x0000;
        if(buf_header->body_length >= 0)
        {
            c_crc = crc_16(pmcb->receiving_buf + HEADER_LENGTH, buf_header->body_length);
        }
        if(r_crc == c_crc && buf_tail->magic_number == TAIL_MAGIC)
        {
            return PE_GET_VALID_BODY;
        }
        else
        {
            return PE_GET_INVALID_BODY;
        }
    }
    else if(pmcb->timeout_count > 0)
    {
        return PE_RECEIVING_TIMEOUT;
    }
    else
    {
        return PE_NONE;
    }
}


static fp_ck_event Event_Check_Functions[PS_STATE_COUNT] =
{
    [PS_WAITING_HEADER]   = ck_wait_header_event,
    [PS_RECEIVING_HEADER] = ck_receiving_header_event,
    [PS_WAITING_TAIL]     = ck_waiting_tail_event,
};


static enum ProtocolEventDef check_event(enum ProtocolChannel channel)
{
    void* fsm = Protocol_MCBs[channel].protocol_fsm;
    enum ProtocolStateDef c_state = fsm_current_state(fsm);
    fp_ck_event ck_event_fun = Event_Check_Functions[c_state];
    return ck_event_fun(channel);
}


static void on_protocol_event(enum ProtocolChannel channel, enum ProtocolEventDef event)
{
    void* fsm = Protocol_MCBs[channel].protocol_fsm;
    fsm_on_event(fsm, event);

    return;
}


static void transaction(enum ProtocolChannel channel)
{
    enum ProtocolEventDef evt = check_event(channel);
    on_protocol_event(channel, evt);

    return;
}
#pragma endregion


void init_protocol_state(TIM_HandleTypeDef* timer)
{
    pTransmissionTimer = timer;
    HAL_TIM_Base_Start_IT(pTransmissionTimer);
    HAL_TIM_Base_Start(pTransmissionTimer);

    // init protocol fsm
    for(uint32_t pc = PROTOCOL_CHANNEL_USB; pc < PROTOCOL_CHANNEL_COUNT; pc++)
    {
        memset(&Protocol_MCBs[pc], 0, sizeof(struct ProtocolMCB));
        Protocol_MCBs[pc].buf_ready_pos = Protocol_MCBs[pc].receiving_buf;

        void* fsm = fsm_create(PS_STATE_COUNT, PE_EVENT_COUNT, pc);

        fsm_reg_state(fsm, PS_WAITING_HEADER, 1);
        fsm_reg_state(fsm, PS_RECEIVING_HEADER, 0);
        fsm_reg_state(fsm, PS_WAITING_TAIL, 0);

        fsm_reg_event(fsm, PE_HEADER_MAGIC_FRONT_APPEARANCE);
        fsm_reg_event(fsm, PE_HEADER_MAGIC_APPEARANCE);
        fsm_reg_event(fsm, PE_GET_VALID_HEADER);
        fsm_reg_event(fsm, PE_GET_INVALID_HEADER);
        fsm_reg_event(fsm, PE_TAIL_MAGIC_APPEARANCE);
        fsm_reg_event(fsm, PE_GET_VALID_BODY);
        fsm_reg_event(fsm, PE_GET_INVALID_BODY);
        fsm_reg_event(fsm, PE_RECEIVING_TIMEOUT);
        fsm_reg_event(fsm, PE_NONE);

        fsm_reg_transaction(fsm, PS_WAITING_HEADER, PS_RECEIVING_HEADER, PE_HEADER_MAGIC_APPEARANCE, 0);
        fsm_reg_transaction(fsm, PS_WAITING_HEADER, PS_WAITING_HEADER, PE_HEADER_MAGIC_FRONT_APPEARANCE, 0);
        fsm_reg_transaction(fsm, PS_WAITING_HEADER, PS_WAITING_HEADER, PE_NONE, action_clear_buffer);

        fsm_reg_transaction(fsm, PS_RECEIVING_HEADER, PS_WAITING_TAIL, PE_GET_VALID_HEADER, action_notify_header);
        fsm_reg_transaction(fsm, PS_RECEIVING_HEADER, PS_WAITING_HEADER, PE_GET_INVALID_HEADER, action_disposal_invalid_head);

        fsm_reg_transaction(fsm, PS_WAITING_TAIL, PS_WAITING_HEADER, PE_GET_VALID_BODY, action_notify_frame);
        fsm_reg_transaction(fsm, PS_WAITING_TAIL, PS_WAITING_HEADER, PE_GET_INVALID_BODY, action_disposal_invalid_body);

        fsm_reg_transaction(fsm, PS_WAITING_HEADER, PS_WAITING_HEADER, PE_RECEIVING_TIMEOUT, action_clear_buffer);
        fsm_reg_transaction(fsm, PS_RECEIVING_HEADER, PS_WAITING_HEADER, PE_RECEIVING_TIMEOUT, action_clear_buffer);
        fsm_reg_transaction(fsm, PS_WAITING_TAIL, PS_WAITING_HEADER, PE_RECEIVING_TIMEOUT, action_clear_buffer);

        Protocol_MCBs[pc].protocol_fsm = fsm;
    }

    return;
}


void on_protocol_data_received(enum ProtocolChannel channel, uint8_t* d, uint32_t len)
{
    __HAL_TIM_SET_COUNTER(pTransmissionTimer, 0);

    struct ProtocolMCB* pmcb = &Protocol_MCBs[channel];
    for(uint32_t i = 0; i < len; i++)
    {
        pmcb->timeout_count = 0;
        *(pmcb->buf_ready_pos) = d[i];
        pmcb->buf_ready_pos += 1;
        if(pmcb->buf_ready_pos - pmcb->receiving_buf >= BUFFER_LENGTH)
        {
            pmcb->buf_ready_pos = pmcb->receiving_buf;
        }
        transaction(channel);
    }

    return;
}


void on_protocol_timeout()
{
    for(uint32_t pc = PROTOCOL_CHANNEL_USB; pc < PROTOCOL_CHANNEL_COUNT; pc++)
    {
        Protocol_MCBs[pc].timeout_count++;
        transaction(pc);
    }
    return;
}


uint8_t perform_intent(enum ProtocolChannel channel, uint8_t intent, uint32_t ack_no, uint8_t* data, uint16_t data_len)
{
    struct ProtocolMCB* pmcb = &Protocol_MCBs[channel];
    pmcb->sending_header.magic_number = HEAD_MAGIC;
    pmcb->sending_header.intent = intent;
    pmcb->sending_header.sequence = pmcb->transmit_frame_count;
    pmcb->sending_header.acknowledgement = ack_no;
    pmcb->sending_header.body_length = data_len;
    pmcb->sending_header.crc = crc_16((uint8_t*)&pmcb->sending_header, HEADER_LENGTH - 2);

    pmcb->sending_tail.magic_number = TAIL_MAGIC;
    if(data_len > 0)
    {
        pmcb->sending_tail.crc = crc_16(data, data_len);
    }
    else
    {
        pmcb->sending_tail.crc = 0;
    }

    memcpy(pmcb->sending_buf, &pmcb->sending_header, HEADER_LENGTH);
    if(data_len > 0)
    {
        memcpy(pmcb->sending_buf + HEADER_LENGTH, data, data_len);
    }
    memcpy(pmcb->sending_buf + HEADER_LENGTH + data_len, &pmcb->sending_tail, TAIL_LENGTH);

    transmit_data(channel, pmcb->sending_buf, HEADER_LENGTH + data_len + TAIL_LENGTH);
    pmcb->transmit_frame_count++;

    return 0;
}
