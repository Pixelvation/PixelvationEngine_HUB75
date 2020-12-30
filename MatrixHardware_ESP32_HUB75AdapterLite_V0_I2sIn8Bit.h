// include SmartMatrix Library MatrixHardware file first
#include <MatrixHardware_ESP32_HUB75AdapterLite_V0.h>

#define I2S_RISING_EDGE_CLK   true

// IO12 (MTDI), IO15 (MTDO), GPIO0 • GPIO2 • GPIO5 are used for boot strapping, so shouldn't be driven during boot (don't use for I2S In)
#define GPIO_I2S_IN_PIN_CLK (gpio_num_t)36
#define GPIO_I2S_IN_PIN_EN  (gpio_num_t)0

#define GPIO_I2S_IN_PIN_D0  (gpio_num_t)37
#define GPIO_I2S_IN_PIN_D1  (gpio_num_t)38
#define GPIO_I2S_IN_PIN_D2  (gpio_num_t)39
#define GPIO_I2S_IN_PIN_D3  (gpio_num_t)34
#define GPIO_I2S_IN_PIN_D4  (gpio_num_t)13
#define GPIO_I2S_IN_PIN_D5  (gpio_num_t)14
#define GPIO_I2S_IN_PIN_D6  (gpio_num_t)35
#define GPIO_I2S_IN_PIN_D7  (gpio_num_t)12
