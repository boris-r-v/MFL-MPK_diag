#include "main.h"

int main(void)
{
    init_modem();

    int_ON();
    RX_FX604;
    while (1)
    {
        //__delay_ms(400);
        request_TDIM();
    }

    return (0);
}