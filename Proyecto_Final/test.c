#include "test.h"


uint16t_t joistick_read(uint8_t x)
{
    uint16_t r = random(3700, 4095);
    return r;
}

uint16_t temperature_read(uint8_t x)
{
    return 250;
}

bool_t bw_fw(uint8_t x)
{
    return 1;
}







