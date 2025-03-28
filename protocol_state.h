#ifndef _PROTOCOL_STATE_H_
#define _PROTOCOL_STATE_H_

#include "tim.h"


enum NWStatus
{
    NW_STATUS_OK                 = 0x00000000U,
    NW_STATUS_BUSY               = 0x00000001U,
    NW_STATUS_HEAD_CRC_ERROR     = 0x00000002U,
    NW_STATUS_BODY_CRC_ERROR     = 0x00000003U,
    NW_STATUS_ILLEGAL_DATA       = 0x00000004U,
    NW_STATUS_ILLEGAL_CHANNEL    = 0x00000005U,
    NW_STATUS_NONE               = 0xFFFFFFFEU,
    NW_STATUS_UNKNOWN_ERROR      = 0xFFFFFFFFU
};


struct FrameHeader
{
    uint16_t  magic_number;
    uint8_t   revision;
    uint8_t   intent;
    uint32_t  sequence;
    uint32_t  acknowledgement;
    uint16_t  body_length;
    uint16_t  crc;
};


struct FrameTail
{
    uint16_t  crc;
    uint16_t  magic_number;
};


enum ProtocolChannel
{
    PROTOCOL_CHANNEL_USB,
    PROTOCOL_CHANNEL_UART,
    PROTOCOL_CHANNEL_GENERIC,
    PROTOCOL_CHANNEL_COUNT
};


void init_protocol_state(TIM_HandleTypeDef* timer);
void on_protocol_data_received(enum ProtocolChannel channel, uint8_t* d, uint32_t len);
void on_protocol_timeout();

uint8_t perform_intent(enum ProtocolChannel channel, uint8_t intent, uint32_t ack_no, uint8_t* data, uint16_t data_len);
#endif //_PROTOCOL_STATE_H_
