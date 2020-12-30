// include SmartMatrix Library MatrixHardware file first
#include <MatrixHardware_ESP32_HUB75Adapter_SMT.h>

// TODO: not sure about this polarity
#define I2S_RISING_EDGE_CLK   false

#define GPIO_I2S_IN_PIN_CLK (gpio_num_t)21
#define GPIO_I2S_IN_PIN_EN  (gpio_num_t)0

#define GPIO_I2S_IN_PIN_D0  (gpio_num_t)33
#define GPIO_I2S_IN_PIN_D1  (gpio_num_t)32
#define GPIO_I2S_IN_PIN_D2  (gpio_num_t)35
#define GPIO_I2S_IN_PIN_D3  (gpio_num_t)34
#define GPIO_I2S_IN_PIN_D4  (gpio_num_t)39
#define GPIO_I2S_IN_PIN_D5  (gpio_num_t)38
#define GPIO_I2S_IN_PIN_D6  (gpio_num_t)37
#define GPIO_I2S_IN_PIN_D7  (gpio_num_t)36
