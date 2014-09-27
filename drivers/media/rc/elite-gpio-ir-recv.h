#include <linux/wait.h>


#define ELITE_GPIO_RISING               7
#define ELITE_GPIO_FALING               11
#define ELITE_GPIO_INT_RISING           1
#define ELITE_GPIO_INT_FALLING          2
#define ELITE_GPIO_IR_DRIVER_NAME	    "elite-gpio-rc-recv"
#define ELITE_GPIO_IR_DEVICE_NAME   	"elite-gpio-ir-recv"

struct elite_gpio_rc_dev {
	struct rc_dev *rcdev;
        int gpio_rising;    
        int gpio_falling;
        bool active_low;
};

struct elite_gpio_ir_recv_platform_data {
        int gpio_rising;    
        int gpio_falling;
        bool active_low;
};

static struct elite_gpio_ir_recv_platform_data elite_ir_recv_platform_data = {
    .gpio_rising = ELITE_GPIO_RISING,
    .gpio_falling = ELITE_GPIO_FALING,
    .active_low = true,

    
};
static  struct platform_device elite_gpio_rc_device = {
        .name = "elite-gpio-rc-recv",
        .id       = 0,
        .dev    = {
                .platform_data = &elite_ir_recv_platform_data,
            },
};


