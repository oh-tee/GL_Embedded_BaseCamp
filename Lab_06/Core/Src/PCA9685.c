#include "PCA9685.h"
#include <stddef.h>

#define PCA9685_ID 0x80

static PCA9685_Send_Callback sendDataCb = NULL;
static PCA9685_GPIO_Set_Callback gpioSetCb = NULL;

void LED_Driver_Init(uint8_t mode, PCA9685_GPIO_Set_Callback gpioCb, PCA9685_Send_Callback sendCb)
{
	if (sendCb == NULL || gpioCb == NULL)
		return;

	sendDataCb = sendCb;
	gpioSetCb = gpioCb;

	gpioSetCb(false);
	uint8_t data[2] = {0x00, 0x01}; //MODE0
	sendDataCb(PCA9685_ID, data, sizeof(data));
}

void LED_Driver_Set_PWM(uint8_t ledId, uint8_t pulse)
{
	uint8_t data[2] = {0x06, 0x00};
	sendDataCb(PCA9685_ID, data, sizeof(data));
}
