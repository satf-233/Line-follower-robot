#include "pins.h"
#include "driver/gpio.h"
#include "start.h"

#if 0  /* 使用 test_follow.c 作为入口时，此处注释掉，避免重复 app_main */

int app_main(void)
{
    start();
    return 0;
}
#endif