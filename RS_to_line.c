#include <p24HJ128GP502.h>

#include "RS_to_line.h"
#include "CRC8.h"
#include "init.h"
#include "request_response_TDIM.h"

static frame frame_to_line;
static data data_to_line;

static void receive_data_RS485(void);
static void receive_data_RS232(void);
static void answer_addr(void);
static void send_data(void);
static void send_byte(uint8_t byte);
static void send_carrier(void);
static bool request_UMV64(void);

void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
    bool is_request_UMV64;
    
    ClrWdt();
    HL2 = 1;

    receive_data_RS485();
    is_request_UMV64 = request_UMV64();
    
    if (!is_request_UMV64)
    {
        HL2 = 0;
        HL1 = 1;
        send_data();
        TMR3 = 0x00; // Clear 32-bit Timer (msw)
        TMR2 = 0x00; // Clear 32-bit Timer (lsw)
        IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
        T2CONbits.TON = 1; // Start 32-bit Timer
    }

    HL1 = 0;
    HL2 = 0;

    IFS0bits.U1RXIF = 0;
}

static void receive_data_RS485(void)
{
    data_to_line.ptr = frame_to_line.data_buffer;
    data_to_line.cnt = 0;
    data_to_line.len = 0;

    // Timer ON, clear flag
    IFS0bits.T1IF = 0;
    TMR1 = 0;
    T1CONbits.TON = 1;
    while (!IFS0bits.T1IF)
    {
        if (U1STAbits.URXDA == 1)
        {
            *data_to_line.ptr = U1RXREG;
            data_to_line.ptr++;
            data_to_line.len++;
            TMR1 = 0;
        }
    }
    // Timer OFF
    IFS0bits.T1IF = 0;
    TMR1 = 0;
    T1CONbits.TON = 0;
}

static bool request_UMV64(void)
{
    request_frame_UMV64 tmp;
    uint8_t addr;
    
    addr = get_addr();
    
    tmp.null_byte = frame_to_line.data_buffer[0];
    tmp.addr = frame_to_line.data_buffer[1];
    tmp.setting_byte = frame_to_line.data_buffer[2];
    
    tmp.crc16 = frame_to_line.data_buffer[3];
    tmp.crc16 = (tmp.crc16 << 8) | frame_to_line.data_buffer[4];
    
    if (tmp.null_byte == NULL_BYTE &&
        tmp.addr == addr &&
        tmp.setting_byte == SETTING_BYTE)
    {
        send_data_to_KVF(tmp);
        return 1;
    }
    return 0;
}

void __attribute__((__interrupt__)) _U2RXInterrupt(void)
{
    ClrWdt();
    HL2 = 1;
    receive_data_RS232();

    if (data_to_line.len == 2 && frame_to_line.data_buffer[0] == 0x14 && frame_to_line.data_buffer[1] == 0xFF)
    {
        answer_addr();
    }
    else
    {
        HL2 = 0;
        HL1 = 1;
        send_data();
        /*TMR3 = 0x00; // Clear 32-bit Timer (msw)
        TMR2 = 0x00; // Clear 32-bit Timer (lsw)
        IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
        T2CONbits.TON = 1; // Start 32-bit Timer*/
    }

    HL1 = 0;
    HL2 = 0;

    TMR3 = 0x00; // Clear 32-bit Timer (msw)
    TMR2 = 0x00; // Clear 32-bit Timer (lsw)
    IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
    T2CONbits.TON = 1; // Start 32-bit Timer
    IFS1bits.U2RXIF = 0;
}

static void receive_data_RS232(void)
{
    data_to_line.ptr = frame_to_line.data_buffer;
    data_to_line.cnt = 0;
    data_to_line.len = 0;

    // Timer ON, clear flag
    IFS0bits.T1IF = 0;
    TMR1 = 0;
    T1CONbits.TON = 1;
    while (!IFS0bits.T1IF)
    {
        if (U2STAbits.URXDA == 1)
        {
            *data_to_line.ptr = U2RXREG;
            data_to_line.ptr++;
            data_to_line.len++;
            TMR1 = 0;
        }
    }
    // Timer OFF
    IFS0bits.T1IF = 0;
    TMR1 = 0;
    T1CONbits.TON = 0;
}

static void answer_addr(void)
{
    frame_to_line.data_buffer[0] = 0xEB;
    frame_to_line.data_buffer[1] = get_addr();
    data_to_line.len = 2;

    // RS-232
    data_to_line.ptr = frame_to_line.data_buffer;
    data_to_line.cnt = data_to_line.len;
    while (data_to_line.cnt)
    {
        U2TXREG = *(data_to_line.ptr);
        (data_to_line.ptr)++;
        (data_to_line.cnt)--;
        while (U2STAbits.UTXBF);
    }
}

static void send_data(void)
{
    uint8_t head[SIZE_OF_HEAD];
    uint8_t crc;

    TX_FX604;

    send_carrier();

    head[NULL_BYTE] = COMMAND_SEND_TO_RS;
    head[DATA_SIZE_H] = data_to_line.len >> 8;
    head[DATA_SIZE_L] = data_to_line.len;
    crc = Crc8(head, SIZE_OF_HEAD);

    send_byte(head[NULL_BYTE]);
    send_byte(head[DATA_SIZE_H]);
    send_byte(head[DATA_SIZE_L]);
    send_byte(crc);

    data_to_line.ptr = frame_to_line.data_buffer;
    for (data_to_line.cnt = 0; data_to_line.cnt < data_to_line.len; data_to_line.cnt++)
    {
        send_byte(*data_to_line.ptr);
        data_to_line.ptr++;
    }
    RX_FX604;
}

void __attribute__((__interrupt__)) _U1ErrInterrupt(void)
{
    uint8_t i;
    
    if (U1STAbits.FERR == 1);

    if (U1STAbits.OERR == 1)
    {
        i = U1RXREG;
        i = U1RXREG;
        i = U1RXREG;
        U1STAbits.OERR = 0;
    }
    
    IFS4bits.U1EIF = 0;
    IFS0bits.U1RXIF = 0;
}

void __attribute__((__interrupt__)) _U2ErrInterrupt(void)
{
    uint8_t i;
    
    if (U2STAbits.FERR == 1);

    if (U2STAbits.OERR == 1)
    {
        i = U2RXREG;
        i = U2RXREG;
        i = U2RXREG;
        U2STAbits.OERR = 0;
    }
        
    IFS4bits.U2EIF = 0;
    IFS1bits.U2RXIF = 0;
}

static void send_byte(uint8_t byte)
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

    MOSI = 0;
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

static void send_carrier(void)
{
    uint16_t i;
    SCLK = 0;
    while (RDYN);

    MOSI = 1;
    __delay_us(100);
    SCLK = 1;
    __delay_us(100);
    SCLK = 0;
    while (RDYN);

    for (i = 0; i < 35; i++)
    {
        MOSI = 1;
        __delay_us(100);
        SCLK = 1;
        __delay_us(100);
        SCLK = 0;
        while (RDYN);
    }

    MOSI = 1;
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
