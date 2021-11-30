#include <stdint.h>

#define AAIWP		0xAD	// Auto Address Increment Programming
#define BE         	0xC7	// Bulk Erase
#define DBSY		0x80	// Disable SO as RY/BY# status during AAI programming
#define DUMMY		0xA5	// Dummy Byte
#define EBSY		0x70	// Enable SO to output RY/BY# status during AAI programming
#define EWSR		0x50	// Enable Write Status Register
#define RDSR       	0x05	// Read Status Register
#define READ		0x03	// 25 MHz Read
#define SE          0xD8 	// Sector Erase
#define WIP			0x01	// Write In Progress
#define WRDI		0x04	// Write Disable
#define WREN		0x06  	// Write Enable
#define WRSR        0x01	// Write Status Register

void Memory_Driver_Erase_Sector(uint32_t address);
void Memory_Driver_Erase_Flash(void);
void Memory_Driver_Read(uint8_t *buf, uint32_t address, uint16_t length);
uint8_t Memory_Driver_Write(uint8_t *buf, uint32_t address, uint16_t length);
