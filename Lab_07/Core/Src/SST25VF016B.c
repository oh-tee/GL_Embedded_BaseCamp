#include "SST25VF016B.h"
#include "gpio.h"
#include "spi.h"
#include <stdbool.h>

static uint8_t Read_Byte(void);
static uint8_t Send_Byte(uint8_t data);
static void Select_Chip(bool enable);
static void Send_Command(uint8_t command);
static void Start_Read(uint32_t address);
static void Wait_Write_End(void);
static void Write_Enable(void);
static uint8_t Write_Register(uint8_t reg, uint8_t val);
void Memory_Driver_Erase_Sector(uint32_t address);
void Memory_Driver_Erase_Flash(void);
void Memory_Driver_Read(uint8_t *buf, uint32_t address, uint16_t length);
uint8_t Memory_Driver_Write(uint8_t *buf, uint32_t address, uint16_t length);

static void Write_Enable(void)
{
	Send_Command(WREN); 									// Send Write Enable
}

void Memory_Driver_Read(uint8_t *buf, uint32_t address, uint16_t length)
{
	Start_Read(address);

	while(length--)
		*buf++ = Send_Byte(DUMMY);

	Select_Chip(false);
}

void Memory_Driver_Erase_Sector(uint32_t address)
{
	uint8_t high_nibble = (address & 0xFF0000) >> 16;
	uint8_t middle_nibble = (address & 0xFF00) >> 8;
	uint8_t low_nibble = address & 0xFF;

	Write_Enable();
	Select_Chip(true);
	Send_Byte(SE);											// Send Sector Erase
	Send_Byte(high_nibble);
	Send_Byte(middle_nibble);
	Send_Byte(low_nibble);
	Select_Chip(false);
	Wait_Write_End();
}

void Memory_Driver_Erase_Flash(void)
{
	Write_Enable();
	Select_Chip(true);
	Send_Byte(BE);											// Send Bulk Erase
	Select_Chip(false);
	Wait_Write_End();
}

static uint8_t Send_Byte(uint8_t data)
{
    uint8_t Tx = data;
    uint8_t Rx = 0;

    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET){};

    HAL_SPI_TransmitReceive(&hspi1, &Tx, &Rx, 1, 100);

    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET){};

	return Rx;
}

static uint8_t Read_Byte(void)
{
  return (Send_Byte(DUMMY));
}

static void Wait_Write_End(void)
{
	uint8_t status = 0;

	Select_Chip(true);
	Send_Byte(RDSR);										// Send Read Status Register
	do
	{
		status = Send_Byte(DUMMY);
	} while ((status & WIP) == 1); 							// Wait while write is in progress

	Select_Chip(false);
}

uint8_t Memory_Driver_Write(uint8_t *buf, uint32_t address, uint16_t length)
{
	uint8_t *buffer_start = buf;

	uint8_t high_nibble = (address & 0xFF0000) >> 16;
	uint8_t middle_nibble = (address & 0xFF00) >> 8;
	uint8_t low_nibble = address & 0xFF;

	Send_Command(EWSR);										// Enable Write Status Register
	Write_Register(WRSR, 0x00);								// Disable write protection
	Send_Command(EBSY);										// Enable SO to output RY/BY# status during AAI programming
	Send_Command(WREN);										// Send Write Enable
	Select_Chip(true);
	Send_Byte(AAIWP);										// Auto Address Increment Programming

	Send_Byte(high_nibble);
	Send_Byte(middle_nibble);
	Send_Byte(low_nibble);
	Send_Byte(*buf++);
	Send_Byte(*buf++);

	while(buf < buffer_start + length)
	  {
		Select_Chip(false);
		Select_Chip(true);

		while (Send_Byte(0xFF) != 0xFF){};

		Select_Chip(false);
		Select_Chip(true);

		Send_Byte(AAIWP);

		Send_Byte(*buf++);
		Send_Byte(*buf++);
	  }
	  /* Deselect the FLASH: Chip Select high */
	Select_Chip(false);
	Select_Chip(true);

	while (Send_Byte(0xFF) != 0xFF){};

	Select_Chip(false);
	Send_Command(WRDI);										// Send Write Disable
	Send_Command(DBSY);										// Disable SO as RY/BY# status during AAI programming

	Wait_Write_End();

	return Write_Register(RDSR, 0x00);						// Read Status Register
}

static void Send_Command(uint8_t command)
{
	Select_Chip(true);
	Send_Byte(command);
	Select_Chip(false);
}

static uint8_t Write_Register(uint8_t reg, uint8_t val)
{
	uint8_t result;

	Select_Chip(true);
	Send_Byte(reg);
	result = Send_Byte(val);
	Select_Chip(false);

	return result;
}

static void Start_Read(uint32_t address)
{
	uint8_t high_nibble = (address & 0xFF0000) >> 16;
	uint8_t middle_nibble = (address & 0xFF00) >> 8;
	uint8_t low_nibble = address & 0xFF;

	Select_Chip(true);
	Send_Byte(READ);										// Read From Memory at 25 MHz

	Send_Byte(high_nibble);
	Send_Byte(middle_nibble);
	Send_Byte(low_nibble);

	Send_Byte(0xFF);
}

static void Select_Chip(bool enable)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
