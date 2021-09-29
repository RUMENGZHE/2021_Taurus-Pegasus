#include <app_hello_world.h>

extern hi_void app_i2c_oled_demo(hi_void);
extern hi_u32 app_demo_led_control_task(hi_void);

hi_void hispark_pegasus_hello_world(hi_void)
{
    app_i2c_oled_demo();
    app_demo_led_control_task();
}