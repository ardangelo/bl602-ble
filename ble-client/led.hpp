// Bouffalo device
extern "C" {
#include <bl_sys.h>
#include <bl_gpio.h>
} // extern "C"

namespace led {

static constexpr auto red = 17;
static constexpr auto green = 14;
static constexpr auto blue = 11;

static inline void off()
{
    bl_gpio_output_set(red, 1);
    bl_gpio_output_set(green, 1);
    bl_gpio_output_set(blue, 1);
}

static inline void init()
{
    bl_gpio_enable_output(led::red, 1, 0);
    bl_gpio_enable_output(led::blue, 1, 0);
    bl_gpio_enable_output(led::green, 1, 0);

    off();
}

static inline void red_on()
{
    bl_gpio_output_set(red, 0);
}

static inline void green_on()
{
    bl_gpio_output_set(green, 0);
}

static inline void blue_on()
{
    bl_gpio_output_set(blue, 0);
}

} // namespace led
