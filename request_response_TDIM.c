#include "request_response_TDIM.h"

static answer_frame_UMV32 answer_frame_UMV32_to_RS;
static answer_frame_UMV32 answer_frame_UMV32_to_RS_0;
static answer_frame_UMV32 answer_frame_UMV32_to_RS_1;
static bool update_data = true;
static uint8_t block_num = BLOCK_NUM_1;
static uint8_t err_block_0 = 0;
static uint8_t err_block_1 = 0;
static uint8_t err_block_2 = 0;
static uint8_t err_block_3 = 0;

static void send_uint8_RS485(uint8_t send_data);
static void send_uint16_RS485(uint16_t send_data);
static void send_byte_line(uint8_t byte);
static uint8_t receive_byte_line(void);
static void send_carrier(void);
static void send_request_TDIM(void);
static uint16_t form_CRC16 (answer_frame_UMV32 answer_frame);
static uint8_t form_answer_data(uint8_t data, uint8_t half_position);

void init_answer_frame(void)
{
    uint8_t i;
    
    answer_frame_UMV32_to_RS_0.null_byte = NULL_BYTE;
    answer_frame_UMV32_to_RS_0.addr = get_addr();
    for (i = 0; i < UMV32_DATA_SIZE; i++)
        answer_frame_UMV32_to_RS_0.data[i] = INIT_ANSWER_DATA;
    answer_frame_UMV32_to_RS_0.diag = DIAG_BYTE_UMV32;
    answer_frame_UMV32_to_RS_0.crc16 = form_CRC16(answer_frame_UMV32_to_RS_0);
    
    answer_frame_UMV32_to_RS_1.null_byte = NULL_BYTE;
    answer_frame_UMV32_to_RS_1.addr = get_addr();
    for (i = 0; i < UMV32_DATA_SIZE; i++)
        answer_frame_UMV32_to_RS_1.data[i] = INIT_ANSWER_DATA;
    answer_frame_UMV32_to_RS_1.diag = DIAG_BYTE_UMV32;
    answer_frame_UMV32_to_RS_1.crc16 = form_CRC16(answer_frame_UMV32_to_RS_1);
}

void __attribute__((__interrupt__)) _T3Interrupt(void)
{
    switch (block_num)
    {
        case BLOCK_NUM_0:
        {
            if (err_block_0 > 3)
            {
                answer_frame_UMV32_to_RS_0.data[0] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_0.data[1] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_0.data[2] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_0.data[3] = ERR_ANSWER_DATA;
            }
            else err_block_0++;
            break;
        }
        case BLOCK_NUM_1:
        {
            if (err_block_1 > 3)
            {
                answer_frame_UMV32_to_RS_0.data[4] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_0.data[5] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_0.data[6] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_0.data[7] = ERR_ANSWER_DATA;
            }
            else err_block_1++;
            break;
        }
        case BLOCK_NUM_2:
        {
            if (err_block_2 > 3)
            {
                answer_frame_UMV32_to_RS_1.data[0] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_1.data[1] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_1.data[2] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_1.data[3] = ERR_ANSWER_DATA;
            }
            else err_block_2++;
            break;
        }
        case BLOCK_NUM_3:
        {
            if (err_block_3 > 3)
            {
                answer_frame_UMV32_to_RS_1.data[4] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_1.data[5] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_1.data[6] = ERR_ANSWER_DATA;
                answer_frame_UMV32_to_RS_1.data[7] = ERR_ANSWER_DATA;
            }
            else err_block_3++;
            break;
        }
        default:
        {
            block_num = BLOCK_NUM_0;
            break;
        }
    }
    
    update_data = true;
    //request_TDIM();
    TMR3 = 0x00; // Clear 32-bit Timer (msw)
    TMR2 = 0x00; // Clear 32-bit Timer (lsw)
    IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
    T2CONbits.TON = 1; // Start 32-bit Timer
}

void request_TDIM(void)
{
    if (!update_data) return;
    
    HL1 = 1;
    ClrWdt();
    update_data = false;
    switch (block_num)
    {
        case BLOCK_NUM_0:
        {
            block_num = BLOCK_NUM_1;
            break;
        }
        case BLOCK_NUM_1:
        {
            block_num = BLOCK_NUM_2;
            break;
        }
        case BLOCK_NUM_2:
        {
            block_num = BLOCK_NUM_3;
            break;
        }
        case BLOCK_NUM_3:
        {
            block_num = BLOCK_NUM_0;
            break;
        }
        default:
        {
            block_num = BLOCK_NUM_0;
            break;
        }
    }
    
    send_request_TDIM();
    
    TMR3 = 0x00; // Clear 32-bit Time`r (msw)
    TMR2 = 0x00; // Clear 32-bit Timer (lsw)
    IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
    T2CONbits.TON = 1; // Start 32-bit Timer
    HL1 = 0;
}

static void send_request_TDIM(void)
{
    TX_FX604;
    send_carrier();
    send_byte_line(block_num);
    send_byte_line(0x55);
    send_byte_line(block_num);
    RX_FX604;
}

void response_TDIM(uint8_t block_num_from_TDIM) //принимаются данные от ТДИМ и формируется кадр ответа в КВФ
{
    uint8_t in_1_8, in_9_16;
    
    if (block_num_from_TDIM != BLOCK_NUM_0 && block_num_from_TDIM != BLOCK_NUM_1 &&
        block_num_from_TDIM != BLOCK_NUM_2 && block_num_from_TDIM != BLOCK_NUM_3)
        return;
    
    /*if (block_num_from_TDIM != block_num)
        return;*/
    
    /*__delay_ms(40);//40
    if (0 == DET) return; // если запрос в ТДИМ от другого модема, то игнорировать
    */
    in_1_8 = receive_byte_line();
    in_9_16 = receive_byte_line();
    
    answer_frame_UMV32_to_RS_0.null_byte = NULL_BYTE;
    answer_frame_UMV32_to_RS_0.addr = get_addr();
    answer_frame_UMV32_to_RS_1.null_byte = NULL_BYTE;
    answer_frame_UMV32_to_RS_1.addr = get_addr();
    switch (block_num)
    {
        case BLOCK_NUM_0:
        {
            answer_frame_UMV32_to_RS_0.data[0] = form_answer_data(in_1_8, FIRST_HALF);
            answer_frame_UMV32_to_RS_0.data[1] = form_answer_data(in_1_8, SECOND_HALF);
            answer_frame_UMV32_to_RS_0.data[2] = form_answer_data(in_9_16, FIRST_HALF);
            answer_frame_UMV32_to_RS_0.data[3] = form_answer_data(in_9_16, SECOND_HALF);
            err_block_0 = 0;
            break;
        }
        case BLOCK_NUM_1:
        {
            answer_frame_UMV32_to_RS_0.data[4] = form_answer_data(in_1_8, FIRST_HALF);
            answer_frame_UMV32_to_RS_0.data[5] = form_answer_data(in_1_8, SECOND_HALF);
            answer_frame_UMV32_to_RS_0.data[6] = form_answer_data(in_9_16, FIRST_HALF);
            answer_frame_UMV32_to_RS_0.data[7] = form_answer_data(in_9_16, SECOND_HALF);
            err_block_1 = 0;
            break;
        }
        case BLOCK_NUM_2:
        {
            answer_frame_UMV32_to_RS_1.data[0] = form_answer_data(in_1_8, FIRST_HALF);
            answer_frame_UMV32_to_RS_1.data[1] = form_answer_data(in_1_8, SECOND_HALF);
            answer_frame_UMV32_to_RS_1.data[2] = form_answer_data(in_9_16, FIRST_HALF);
            answer_frame_UMV32_to_RS_1.data[3] = form_answer_data(in_9_16, SECOND_HALF);
            err_block_2 = 0;
            break;
        }
        case BLOCK_NUM_3:
        {
            answer_frame_UMV32_to_RS_1.data[4] = form_answer_data(in_1_8, FIRST_HALF);
            answer_frame_UMV32_to_RS_1.data[5] = form_answer_data(in_1_8, SECOND_HALF);
            answer_frame_UMV32_to_RS_1.data[6] = form_answer_data(in_9_16, FIRST_HALF);
            answer_frame_UMV32_to_RS_1.data[7] = form_answer_data(in_9_16, SECOND_HALF);
            err_block_3 = 0;
            break;
        }
        default:
        {
            break;
        }
    }
    
    update_data = true;
    TMR3 = 0x00; // Clear 32-bit Timer (msw)
    TMR2 = 0x00; // Clear 32-bit Timer (lsw)
    IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
    T2CONbits.TON = 1; // Start 32-bit Timer
}


void send_data_to_KVF(request_frame_UMV32 frame_from_KVF, uint8_t block)
{
    //проверить CRC
    //если правильно, то отправить в КВФ кадр ответа УМВ
    uint8_t tmp[SIZE_OF_REQUEST_FRAME], i;
    
    tmp[0] = frame_from_KVF.null_byte;
    tmp[1] = frame_from_KVF.addr;
    tmp[2] = frame_from_KVF.setting_byte;
    
    if (frame_from_KVF.crc16 != CRC16(tmp, SIZE_OF_REQUEST_FRAME))
    {
        return;
    }
    
    switch(block)
    {
        case 0:
        {
            answer_frame_UMV32_to_RS_0.crc16 = form_CRC16(answer_frame_UMV32_to_RS_0);
            answer_frame_UMV32_to_RS = answer_frame_UMV32_to_RS_0;
            break;
        }
        case 1:
        {
            answer_frame_UMV32_to_RS_1.crc16 = form_CRC16(answer_frame_UMV32_to_RS_1);
            answer_frame_UMV32_to_RS = answer_frame_UMV32_to_RS_1;
            break;
        }
    }
    
    U1MODEbits.STSEL = 1; // два стоповых бита
    TX_RS485 = 1;
    
    send_uint8_RS485(answer_frame_UMV32_to_RS.null_byte);
    send_uint8_RS485(answer_frame_UMV32_to_RS.addr);
    for (i = 0; i < UMV32_DATA_SIZE; i++)
    {
        send_uint8_RS485(answer_frame_UMV32_to_RS.data[i]);
    }
    send_uint8_RS485(answer_frame_UMV32_to_RS.diag);
    send_uint16_RS485(answer_frame_UMV32_to_RS.crc16);
    
    while (!U1STAbits.TRMT);
    TX_RS485 = 0;
    U1MODEbits.STSEL = 0; // один стоповый бит
}


static void send_uint8_RS485(uint8_t send_data)
{
    U1TXREG = send_data;
    while (U1STAbits.UTXBF);
}

static void send_uint16_RS485(uint16_t send_data)
{
    U1TXREG = send_data >> 8;
    while (U1STAbits.UTXBF);
    U1TXREG = send_data;
    while (U1STAbits.UTXBF);
}


static void send_byte_line(uint8_t byte)
{
    uint8_t i;
    SCLK = 0;
    while (RDYN);

    MOSI = 0;
    __delay_us(100);
    SCLK = 1;
    __delay_us(100);
    SCLK = 0;
    while (RDYN);
    
    for (i = 0; i < 8; i++)
    {
        MOSI = byte >> i;
        __delay_us(100);
        SCLK = 1;
        __delay_us(100);
        SCLK = 0;
        while (RDYN);
    }

    MOSI = 1;//0;
    __delay_us(100);
    SCLK = 1;
    __delay_us(100);
    SCLK = 0;
    while (RDYN);

    MOSI = 1;
    __delay_us(100);
    SCLK = 1;
    __delay_us(100);
    SCLK = 0;
    while (RDYN);

    
}

static uint8_t receive_byte_line(void)
{
    uint8_t byte = 0, i;

    SCLK = 0;
    while(RDYN);                        ///// НУЖНО ЧТОБЫ ПРОДОЛЖЕНИЕ ШЛО ОТ ЗАДНЕГО ФРОНТА (ОТ ПЕРЕХОДА RDYN ИЗ НУЛЯ В ЕДИНИЦУ)
    for (i = 0; i < 8; i++)
    {
        __delay_us(100);
        SCLK = 1;
        byte = byte | (MISO << i);
        __delay_us(100);
        SCLK = 0;
    }
    __delay_us(100);
    SCLK = 1;
    __delay_us(100);
    SCLK = 0;
    return byte;
}

static void send_carrier(void)
{
    uint16_t i;
    SCLK = 0;
    while (RDYN);

    for (i = 0; i < 40; i++)//30
    {
        MOSI = 1;
        __delay_us(100);
        SCLK = 1;
        __delay_us(100);
        SCLK = 0;
        while (RDYN);
    }
}

static uint16_t form_CRC16(answer_frame_UMV32 answer_frame)
{
    uint8_t tmp [SIZE_OF_ANSWER_FRAME], i;
    
    tmp[0] = answer_frame.null_byte;
    tmp[1] = answer_frame.addr;   //CRC в ответе УМВ-64 считается только для поля данных -.- 
    for (i = 0; i < UMV32_DATA_SIZE; i++)
        tmp[i+2] = answer_frame.data[i];
    tmp[10] = DIAG_BYTE_UMV32;
    return CRC16(tmp, 11);
}

static uint8_t form_answer_data(uint8_t data, uint8_t half_position)
{
    uint8_t answer = INIT_ANSWER_DATA;
    
    if (half_position == SECOND_HALF)
    {
        if (data & 0b00000001)
            answer &= 0b00111111;
        else
            answer |= 0b11000000;
        
        if (data & 0b00000010)
            answer &= 0b11001111;
        else
            answer |= 0b00110000;
        
        if (data & 0b00000100)
            answer &= 0b11110011;
        else
            answer |= 0b00001100;
        
        if (data & 0b00001000)
            answer &= 0b11111100;
        else
            answer |= 0b00000011;
    }
    if (half_position == FIRST_HALF)
    {
        if (data & 0b00010000)
            answer &= 0b00111111;
        else
            answer |= 0b11000000;
        
        if (data & 0b00100000)
            answer &= 0b11001111;
        else
            answer |= 0b00110000;
        
        if (data & 0b01000000)
            answer &= 0b11110011;
        else
            answer |= 0b00001100;
        
        if (data & 0b10000000)
            answer &= 0b11111100;
        else
            answer |= 0b00000011;
    }
    return answer;
}
