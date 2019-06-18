#ifndef REQUEST_RESPONSE_TDIM_H
#define	REQUEST_RESPONSE_TDIM_H

#include "common.h"
#include "init.h"
#include "CRC16.h"

void request_TDIM(void);
void response_TDIM(uint8_t block_num);
void send_data_to_KVF(request_frame_UMV32 tmp, uint8_t block);

void init_answer_frame(void);

#endif	/* REQUEST_RESPONSE_TDIM_H */