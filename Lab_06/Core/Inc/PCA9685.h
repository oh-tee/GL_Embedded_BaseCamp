#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	LED_01_ID = 0,
	LED_02_ID,
	LED_03_ID,
	LED_04_ID,
	LED_05_ID,
	LED_06_ID,
	LED_07_ID,
	LED_08_ID,
	LED_09_ID,
	LED_10_ID,
	LED_11_ID,
	LED_12_ID,
	LED_13_ID,
	LED_14_ID,
	LED_15_ID,
	LED_16_ID
}LED_ID_Type;


typedef void (* PCA9685_Send_Callback)(uint8_t addr, uint8_t data[], uint32_t len);
typedef void (* PCA9685_GPIO_Set_Callback)(bool state);

void LED_Driver_Init(PCA9685_GPIO_Set_Callback gpioCb, PCA9685_Send_Callback sendCb);
void LED_Driver_Set_PWM(LED_ID_Type ledId, uint8_t pulse_percent, uint8_t shift_percent);
void LED_Driver_Set_PWM_All(uint8_t pulse_percent, uint8_t shift_percent);
void LED_Driver_Turn_On(LED_ID_Type led_id);
void LED_Driver_Turn_Off(LED_ID_Type led_id);
void LED_Driver_Turn_On_All(void);
void LED_Driver_Turn_Off_All(void);
void LED_Driver_Set_Frequency(uint32_t freq);
void LED_Driver_Set_Sleep_Mode(bool enable);
