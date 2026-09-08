#ifndef SELF_CONTROL_COMMAND_H
#define SELF_CONTROL_COMMAND_H

#include "pyro_databoard.h"
#include "protocol.h"
#include "pyro_typedef.h"

typedef struct  __attribute__((packed))
{
    pyro::frame_header_t header;
    uint16_t cmd_id;
    float data[7];
    uint8_t uesd_data[2];   // 数据大小固定为30
    uint16_t CRC16;
}
referee_datalink_frame_t;

#endif