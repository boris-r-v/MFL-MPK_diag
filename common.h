#ifndef COMMON_DRIVER_H
#define COMMON_DRIVER_H

#define SYSCLK              14318180                         //частота Fosc
#define FCY                 SYSCLK/2


#define BAUDRATE_19200      0
#define BAUDRATE_38400      1
#define BAUDRATE_57600      2
#define BAUDRATE_115200     3

#define BAUDRATE_19200_BRG  ((FCY/19200)/4) - 1
#define BAUDRATE_38400_BRG  ((FCY/38400)/4) - 1
#define BAUDRATE_57600_BRG  ((FCY/57600)/4) - 1
#define BAUDRATE_115200_BRG ((FCY/115200)/4) - 1




//******************************************************************************                                          
//*                          НОЖКИ МИКРОКОНТРОЛЛЕРА                            *
//******************************************************************************
#define HL1                 LATBbits.LATB6
#define HL2                 LATBbits.LATB7
#define M0                  LATAbits.LATA0
#define M1                  LATAbits.LATA1
#define DET                 PORTAbits.RA4
#define RX_FX604            M1 = 1; M0 = 0; __delay_ms(4);//M1 = 1; M0 = 0; __delay_ms(4);                   //настройка модема на прием
#define TX_FX604            M0 = 1; M1 = 0; __delay_us(200);//M0 = 1; M1 = 0; __delay_ms(4);                //настройка модема на передачу
#define MISO                PORTBbits.RB8
#define MOSI                LATBbits.LATB9
#define RDYN                PORTBbits.RB12
#define SCLK                LATBbits.LATB10
#define TX_RS485            LATBbits.LATB15
//пины, связанные с адресом модема
#define ADRES               PORTBbits.RB5
#define LOAD                LATBbits.LATB4
#define SS                  LATBbits.LATB11


//                                          КОНСТАНТЫ
// временные промежутки для FX604
#define Ts_us               1
#define Th_us               1
#define Td_us               1
#define Tchi_us             1


#define BUFF_SIZE           2048
#define SIZE_OF_HEAD        3
#define NULL_BYTE           0
#define DATA_SIZE_H         1
#define DATA_SIZE_L         2

#define COMMAND_SEND_TO_RS  0x77

// константы для таймера
#define TIMEOUT_FOR_19200     (16*FCY)/19200            // если время превышает время принятия 16 бит - то считаем, что пакет полностью принят
#define TIMEOUT_FOR_38400     (16*FCY)/38400
#define TIMEOUT_FOR_57600     (16*FCY)/57600
#define TIMEOUT_FOR_115200    (16*FCY)/115200

//константы для опроса ТДИМ
#define BLOCK_NUM_0             0x87//0xF0
#define BLOCK_NUM_1             0xC3//0xE1
#define BLOCK_NUM_2             0xA5//0xD2
#define BLOCK_NUM_3             0xE1//0xC3
#define UMV64_DATA_SIZE         16//8
#define UMV32_DATA_SIZE         8
#define SETTING_BYTE_UMV64      0xE7
#define SETTING_BYTE_UMV32      0xE3
#define DIAG_BYTE_UMV32         0x00
#define SIZE_OF_REQUEST_FRAME   3
#define SIZE_OF_ANSWER_FRAME    11//10
#define INIT_ANSWER_DATA        0x00
#define ERR_ANSWER_DATA         INIT_ANSWER_DATA
#define NUM_OF_NOT_ANSWER       3
#define FIRST_HALF              0
#define SECOND_HALF             1
#define SET_1_SEC_T3            PR3 = 0x0072; PR2 = 0x0000;//72

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <xc.h>
#include <libpic30.h>

typedef struct
{
    uint8_t* ptr;
    uint16_t len;
    uint16_t cnt;
} data;

typedef struct
{
    uint8_t null_byte;
    uint16_t data_size;
    uint8_t CRC8;
    uint8_t data_buffer[BUFF_SIZE];
} frame;

typedef struct
{
    uint8_t null_byte;
    uint8_t addr;
    uint8_t data[UMV64_DATA_SIZE];
    uint16_t crc16;
} answer_frame_UMV64;

typedef struct
{
    uint8_t null_byte;
    uint8_t addr;
    uint8_t data[UMV32_DATA_SIZE];
    uint8_t diag;//0x00
    uint16_t crc16;
} answer_frame_UMV32;

typedef struct
{
    uint8_t null_byte;
    uint8_t addr;
    uint8_t setting_byte;//0xE7
    uint16_t crc16;
} request_frame_UMV64;

typedef struct
{
    uint8_t null_byte;
    uint8_t addr;
    uint8_t setting_byte;//0xEX
    uint16_t crc16;
} request_frame_UMV32;
#endif // COMMON_DRIVER_H