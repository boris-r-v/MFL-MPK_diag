#include "init.h"

static void init_osc(void); //инициализация генератора частоты
static void remap(void); //ремапинг
static void init_address_baudrate(void); //инициализация адреса/скорости модема
static void init_ports(void); //инициализация портов (настройка на вход-выход)
static void init_timer(void); //инициализация таймера1 (настройка времени), отвечает за время после окончания передачи, при переполнении таймера считаем, что байт с RS принят полностью
static void init_UART1(void);  // RS-485
static void init_UART2(void);  // RS-232

static uint8_t get_baudrate(void);

static struct ADDRBAUDRATEBITS
{

    union
    {
        uint8_t addr_baudrate;

        struct
        {
            unsigned addr : 6;
            unsigned baudrate : 2;
        };
    };
} ADDRBAUDRATEbits;

void init_modem()
{
    init_osc(); //инициализация генератора частоты
    remap(); //ремапинг
    init_ports(); //инициализация портов (настройка на вход-выход)
    init_address_baudrate(); //инициализация адреса/скорости модема
    init_UART1();
    init_UART2();
    init_timer(); //инициализация таймера1 (настройка времени в приеме по RS232 и RS485)
    
    init_answer_frame();
    
    RX_FX604;
    TX_RS485 = 0;
}

/* Настройка генератора (частота Fcy)       НЕ ЗАБУДЬ ПОМЕНЯТЬ В "common.h" SYSCLK
Fosc = Fin * M/(N1*N2), Fcy = Fosc/2,                       
Fosc = 3579545 * 64/(2*8) = 14318180 Hz
Fcy = 14318180/2 = 7159090 Hz */
static void init_osc(void)
{
    PLLFBD = 62; // M = 64,   M = PLLFBD + 2
    CLKDIVbits.PLLPRE = 0; // N1 = 2
    CLKDIVbits.PLLPOST = 3; // N2 = 8
    OSCTUN = 0; // настройка FRC генератора
    RCONbits.SWDTEN = 0; // отключение WDT
    while (0 == OSCCONbits.LOCK); // ждем, пока установится частота
    return;
}

// Ремап ножек

static void remap(void)
{
    __builtin_write_OSCCONL(OSCCON & ~(1 << 6)); // разрешение ремапа

    RPINR18bits.U1RXR = 2; // U1RX на 6 ногу (RP2)
    RPOR1bits.RP3R = 3; // U1TX на 7 ногу (RP3)

    RPINR19bits.U2RXR = 13; // U2RX на 24 ногу (RP13)
    RPOR7bits.RP14R = 5; // U2TX на 25 ногу (RP14)

    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // запрещение ремапа
    return;
}

// Инициализация адреса

static void init_address_baudrate(void)
{
    int8_t clk = 7;

    SCLK = 0;
    LOAD = 1;
    LOAD = 0;
    LOAD = 1;

    ADDRBAUDRATEbits.addr_baudrate = 0;
    ADDRBAUDRATEbits.addr_baudrate = ADDRBAUDRATEbits.addr_baudrate | (ADRES << clk);

    for (clk = 6; clk >= 0; --clk)
    {
        SCLK = 1;
        SCLK = 0;
        ADDRBAUDRATEbits.addr_baudrate = ADDRBAUDRATEbits.addr_baudrate | (ADRES << clk);
    }
    ADDRBAUDRATEbits.addr_baudrate ^= 0xFF; //инвертируем биты, т.к. включенный переключатель подтягивает к нулю 
}

// RS-485

void init_UART1(void)
{
    switch (get_baudrate())
    {
    case BAUDRATE_19200:
        U1BRG = BAUDRATE_19200_BRG;
        break;
    case BAUDRATE_38400:
        U1BRG = BAUDRATE_38400_BRG;
        break;
    case BAUDRATE_57600:
        U1BRG = BAUDRATE_57600_BRG;
        break;
    case BAUDRATE_115200:
        U1BRG = BAUDRATE_115200_BRG;
        break;
    default:
        break;
    }

    U1MODE = 0;
    U1MODEbits.BRGH = 1; // высокоскоростной режим UART
    U1MODEbits.STSEL = 0; // один стоповых бита

    U1STA = 0;
    U1STAbits.URXISEL = 0; // прерывание от приемника после каждого принятого байта
    
    U1MODEbits.UARTEN = 1; // включение модуля UART
    U1STAbits.UTXEN = 1; // разрешение на передачу UART
}

//RS-232

void init_UART2(void)
{
    switch (get_baudrate())
    {
    case BAUDRATE_19200:
        U2BRG = BAUDRATE_19200_BRG;
        break;
    case BAUDRATE_38400:
        U2BRG = BAUDRATE_38400_BRG;
        break;
    case BAUDRATE_57600:
        U2BRG = BAUDRATE_57600_BRG;
        break;
    case BAUDRATE_115200:
        U2BRG = BAUDRATE_115200_BRG;
        break;
    default:
        break;
    }

    U2MODE = 0;
    U2MODEbits.BRGH = 1; // высокоскоростной режим UART
    U2MODEbits.STSEL = 0; // один стоповых бита
    
    U2STA = 0;
    U2STAbits.URXISEL = 0; // прерывание от приемника после каждого принятого байта
        
    U2MODEbits.UARTEN = 1; // включение модуля UART
    U2STAbits.UTXEN = 1; // разрешение на передачу UART
}

static void init_ports(void)
{
    AD1PCFGL = 0xFFFF; //все порты на цифровой режим
    M1 = 1; 
    M0 = 0;
    MOSI = 1;
    
    LATBbits.LATB14 = 1; //RB14 (U2TX), передача в RS-232
    LATBbits.LATB3 = 1; //RB3 (U1TX), передача в RS-485, нужно выставить "1", т.к. между настройкой на вывод и включением UARTa не должно быть нуля
    TX_RS485 = 0;
    
    TRISAbits.TRISA0 = 0; //RA0 (M0) на выход, M0 - прием/передача FX604
    TRISAbits.TRISA1 = 0; //RA1 (M1) на выход, M1 - прием/передача FX604
    TRISAbits.TRISA4 = 1; //RA4 (DET) на вход, детектор несущей в линии
    TRISBbits.TRISB2 = 1; //RB2 (U1RX) на вход, UART1 - прием от RS-485
    TRISBbits.TRISB3 = 0; //RB3 (U1TX) на выход, UART1 - передача в RS-485
    TRISBbits.TRISB4 = 0; //RB4 (LOAD) на выход, LOAD - нужен для сдвигового регистра, который определяет положение переключателей SA2
    TRISBbits.TRISB5 = 1; //RB5 (ADRES) на вход, ADRES - к выходу сдвигового регистра, который определяет положение переключателей SA2
    TRISBbits.TRISB6 = 0; //RB6 (HL1) на выход, HL1 - светодиод
    TRISBbits.TRISB7 = 0; //RB7 (HL2) на выход, HL2 - светодиод
    TRISBbits.TRISB8 = 1; //RB8 (MISO) на вход, MISO данные с линии
    TRISBbits.TRISB9 = 0; //RB9 (MOSI) на выход, MOSI данные в линию
    TRISBbits.TRISB10 = 0; //RB10 (SCLK) на выход, SCLK - частота к модему FX604 и к сдвиговому регистру
    TRISBbits.TRISB11 = 0; //RB11 (SS) на выход, SS - подключена к RXEQ модема FX604, это включение фильтра для приемника
    TRISBbits.TRISB12 = 1; //RB12 (RDYN) на вход, RDYN - Ready for Data Transfer, готовность передать данные с линии
    TRISBbits.TRISB13 = 1; //RB13 (U2RX) на вход, UART2 - прием от RS-232
    TRISBbits.TRISB14 = 0; //RB14 (U2RX) на вход, UART2 - передача в RS-232
    TRISBbits.TRISB15 = 0; //RB15 (T/R) на выход, T/R - настройка на передачу/прием драйвера RS-485 (1 - передача в RS-485; 0 - прием)

    SS = 1; //включение фильтра для приемника FX604
}


static void init_timer(void)
{
    switch (get_baudrate())
    {
    case BAUDRATE_19200:
        PR1 = TIMEOUT_FOR_19200;
        break;
    case BAUDRATE_38400:
        PR1 = TIMEOUT_FOR_38400;
        break;
    case BAUDRATE_57600:
        PR1 = TIMEOUT_FOR_57600;
        break;
    case BAUDRATE_115200:
        PR1 = TIMEOUT_FOR_115200;
        break;
    default:
        break;
    }
    
    //таймер, задающий периодичность опроса ТДИМ
    
    T3CONbits.TON = 0; // Stop any 16-bit Timer3 operation
    T2CONbits.TON = 0; // Stop any 16/32-bit Timer3 operation
    T2CONbits.T32 = 1; // Enable 32-bit Timer mode
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR3 = 0x00; // Clear 32-bit Timer (msw)
    TMR2 = 0x00; // Clear 32-bit Timer (lsw)
    SET_1_SEC_T3;
}


void int_ON(void)
{
    // правила работы прерываний описаны в http://ww1.microchip.com/downloads/en/DeviceDoc/70304A.pdf
    // в частности, что касается прерываний во время обработки другого прерывания ТОГО ЖЕ УРОВНЯ:
    
    // Interrupts with the same user-assigned priority, but a higher natural order priority that become pending 
    // during the exception process, remain pending until the current exception process completes.
    
    // Прерывания с тем же назначенным пользователем приоритетом, но с более высоким приоритетом естественного порядка (определяется положением векторов в памяти),
    // которые становятся ожидающими во время процесса исключения, остаются в ожидании до завершения текущего процесса исключения.
    
    //наивысший приоритет - 7, низший - 1
    IPC16bits.U1EIP = 7; //приоритет прерывания от ошибок UART1 (RS-485)
    IPC16bits.U2EIP = 7; //приоритет прерывания от ошибок UART2 (RS-232)
    IPC2bits.U1RXIP = 6; //приоритет прерывания от приемника UART1 (RS-485)
    IPC7bits.U2RXIP = 6; //приоритет прерывания от приемника UART2 (RS-232)
    IPC4bits.CNIP = 5; //приоритет прерывания от изменения уровня
    IPC2bits.T3IP = 4; // Set Timer3 Interrupt Priority Level
    
    IFS4bits.U1EIF = 0; // сброс флага ошибок у приемника UART1 (RS-485)
    IFS4bits.U2EIF = 0; // сброс флага ошибок у приемника UART2 (RS-232)
    IFS1bits.CNIF = 0; //сброс флага у прерывания от изменения уровня (глобальное)
    IFS0bits.U1RXIF = 0; // сброс флага у приемника UART1 (RS-485)
    IFS1bits.U2RXIF = 0; // сброс флага у приемника UART2 (RS-232)
    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
    
    IEC0bits.T3IE = 1; // Enable Timer3 interrupt
    T2CONbits.TON = 1; // Start 32-bit Timer
    
    CNEN1bits.CN0IE = 1; //разрешение прерывания от изменения уровня DET
    IEC1bits.CNIE = 1; //разрешение прерывания от изменения уровня (глобальное)

    IEC0bits.U1RXIE = 1; // разрешение прерывания от приемника UART1 (RS-485)
    IEC1bits.U2RXIE = 1; // разрешение прерывания от приемника UART2 (RS-232)
    
    IEC4bits.U1EIE = 1; //разрешение прерывания от ошибок UART1 (RS-485)
    IEC4bits.U2EIE = 1; //разрешение прерывания от ошибок UART2 (RS-232)
}

void int_OFF(void)
{
    IEC0bits.U1RXIE = 0; // запрещение прерывания от приемника UART1 (RS-485)
    IEC1bits.U2RXIE = 0; // запрещение прерывания от приемника UART2 (RS-232)
    
    CNEN1bits.CN0IE = 0; //запрещение прерывания от изменения уровня DET
    IEC1bits.CNIE = 0; //запрещение прерывания от изменения уровня (глобальное)
}

uint8_t get_addr(void)
{
    return ADDRBAUDRATEbits.addr;
}

static uint8_t get_baudrate(void)
{
    return ADDRBAUDRATEbits.baudrate;
}
