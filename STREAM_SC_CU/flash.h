#pragma once
#include "stm32f7xx_hal.h"

#define OTP_SECTOR_SIZE 1024
#define ADDR_OTP_SECTOR ((uint32_t)0x1FF0F000) /* OTP Address, 1024 Bytes */

#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000) /* Base address of Sector 0, 32 Kbytes */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08008000) /* Base address of Sector 1, 32 Kbytes */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08010000) /* Base address of Sector 2, 32 Kbytes */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x08018000) /* Base address of Sector 3, 32 Kbytes */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08020000) /* Base address of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08040000) /* Base address of Sector 5, 256 Kbytes */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08080000) /* Base address of Sector 6, 256 Kbytes */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x080C0000) /* Base address of Sector 7, 256 Kbytes */

int FLASH_Read(uint32_t* buffer, uint32_t address, int size);
int FLASH_Write(uint32_t* data, uint32_t address, int size);

uint8_t FLASH_ReadID();
void    FLASH_WriteID(uint8_t id);

uint8_t OTP_ReadID();
void    OTP_WriteID(uint8_t id);