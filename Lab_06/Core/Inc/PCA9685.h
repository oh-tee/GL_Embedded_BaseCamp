#include <stdbool.h>
#include <stdint.h>

typedef void (* PCA9685_Send_Callback)(uint8_t addr, uint8_t data[], uint32_t len);
typedef void (* PCA9685_GPIO_Set_Callback)(bool state);

void LED_Driver_Init(uint8_t mode, PCA9685_GPIO_Set_Callback gpioCb, PCA9685_Send_Callback sendCb);
void LED_Driver_Set_PWM(uint8_t ledId, uint8_t pulse);
