#include "PCA9685.h"
#include <stddef.h>

#define PCA9685_ID 0x80

static PCA9685_Send_Callback sendDataCb = NULL;
static PCA9685_GPIO_Set_Callback gpioSetCb = NULL;

static uint8_t MODE1_Register_State = 0;

void LED_Driver_Init(PCA9685_GPIO_Set_Callback gpioCb, PCA9685_Send_Callback sendCb)
{
	if (sendCb == NULL || gpioCb == NULL)
		return;

	sendDataCb = sendCb;
	gpioSetCb = gpioCb;

	gpioSetCb(false);

	MODE1_Register_State = (1 << 0) | (1 << 5); // MODE 1 ALLCALL and AI enable
	uint8_t data[2] = {0x00, MODE1_Register_State};
	sendDataCb(PCA9685_ID, data, sizeof(data));
}

void LED_Driver_Set_PWM(LED_ID_Type led_id, uint8_t pulse_percent, uint8_t shift_percent)
{
	uint8_t data[5];
	uint16_t pulse_value = (pulse_percent > 0) ? (pulse_percent * 4096 / 100 - 1) : 0;
	uint16_t shift_value = (shift_percent > 0) ? (shift_percent * 4096 / 100 - 1) : 0;

	data[0] = led_id * 4 + 6;					// LED_ON_L address
	data[1] = (uint8_t) shift_value;			// LED_ON_L value
	data[2] = shift_value >> 8;					// LED_ON_H value
	data[3] = (uint8_t) pulse_value;			// LED_OFF_L value
	data[4] = pulse_value >> 8;					// LED_OFF_H value

	sendDataCb(PCA9685_ID, data, sizeof(data));
}

void LED_Driver_Set_PWM_All(uint8_t pulse_percent, uint8_t shift_percent)
{
	uint8_t data[5];
	uint16_t pulse_value = (pulse_percent > 0) ? (pulse_percent * 4096 / 100 - 1) : 0;
	uint16_t shift_value = (shift_percent > 0) ? (shift_percent * 4096 / 100 - 1) : 0;

	data[0] = 0xFA;								// ALL_LED_ON_L address
	data[1] = (uint8_t) shift_value;			// ALL_LED_ON_L value
	data[2] = shift_value >> 8;					// ALL_ED_ON_H value
	data[3] = (uint8_t) pulse_value;			// ALL_LED_OFF_L value
	data[4] = pulse_value >> 8;					// ALL_LED_OFF_H value

	sendDataCb(PCA9685_ID, data, sizeof(data));
}

void LED_Driver_Turn_On(LED_ID_Type led_id)
{
	LED_Driver_Set_PWM(led_id, 100, 0);
}

void LED_Driver_Turn_Off(LED_ID_Type led_id)
{
	LED_Driver_Set_PWM(led_id, 0, 0);
}

void LED_Driver_Turn_On_All(void)
{
	LED_Driver_Set_PWM_All(100, 0);
}

void LED_Driver_Turn_Off_All(void)
{
	LED_Driver_Set_PWM_All(0, 0);
}

void LED_Driver_Set_Frequency(uint32_t freq)
{
	if (freq < 24)
	{
		freq = 24;
	}
	else if (freq > 1526)
	{
		freq = 1526;
	}

	uint32_t presc_value = (25000 / (4096 * freq)) - 1;
	uint8_t data[] = {0xFE, (uint8_t) presc_value};

	sendDataCb(PCA9685_ID, data, sizeof(data));
}

void LED_Driver_Set_Sleep_Mode(bool enable)
{
	if (enable)
	{
		MODE1_Register_State |= 1 << 4;
	}
	else
	{
		MODE1_Register_State &= ~(1 << 4);
	}

	uint8_t data[2] = {0x00, MODE1_Register_State};
	sendDataCb(PCA9685_ID, data, sizeof(data));
}
