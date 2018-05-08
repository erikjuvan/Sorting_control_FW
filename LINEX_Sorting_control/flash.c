#include "flash.h"

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_5			/* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_SECTOR_6 - 1)	/* End @ of user Flash area */

static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
  {
    sector = FLASH_SECTOR_7;
  }
  return sector;
}


int FLASH_Read(uint32_t* buffer, uint32_t address, int size) {

	for (int i = 0; i < size; ++i) {
		*buffer = *(uint32_t*)address;
		buffer++;
		address++;
	}
	
	return size;
}

int FLASH_Write(uint32_t* data, uint32_t address, int size) {
	uint32_t FirstSector = 0, NbOfSectors = 0;
	uint32_t Address = 0, SECTORError = 0;
	__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

	/*Variable used for Erase procedure*/
	static FLASH_EraseInitTypeDef EraseInitStruct;
	
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Get the 1st sector to erase */
	FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FirstSector;
	EraseInitStruct.NbSectors     = NbOfSectors;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	you have to make sure that these data are rewritten before they are accessed during code
	execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	DCRST and ICRST bits in the FLASH_CR register. */
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {
		/*
		Error occurred while sector erase.
		User can add here some code to deal with this error.
		SECTORError will contain the faulty sector and then to know the code error on this sector,
		user can call function 'HAL_FLASH_GetError()'
		*/
	}

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	Address = FLASH_USER_START_ADDR;

	while (size > 0) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *data) == HAL_OK) {
			data++;
			Address = Address + 4;
			size--;
		} else {
			/* Error occurred while writing data in Flash memory.
			User can add here some code to deal with this error */
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

 
	return 0;
}	

uint8_t FLASH_ReadID() {
	return *(uint8_t*)FLASH_USER_START_ADDR;
}

void FLASH_WriteID(uint8_t id) {
	uint32_t SECTORError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	HAL_FLASH_Unlock();

	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = GetSector(FLASH_USER_START_ADDR);
	EraseInitStruct.NbSectors     = 1;
	HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_USER_START_ADDR, id);
		
	HAL_FLASH_Lock();
}

static uint8_t OTP_ReadByte(uint32_t address) {
	return *(uint8_t*)address;
}

static uint32_t OTP_GetEmptyByteAddress() {
	uint32_t address = ADDR_OTP_SECTOR;
	
	for (int i = 0; i < OTP_SECTOR_SIZE; ++i) {
		if (OTP_ReadByte(address + i) == 0xFF) {	// empty byte
			return address + i;
		}
	}
	
	return 0;
}

static void OTP_WriteByte(uint8_t byte, uint32_t address) {
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, byte); 
	HAL_FLASH_Lock();
}

uint8_t OTP_ReadID() {
	uint32_t address = OTP_GetEmptyByteAddress() - 1;
	if (address >= ADDR_OTP_SECTOR)
		return *(uint8_t*)address;
	else 
		return 0;
}

void OTP_WriteID(uint8_t id) {
	uint32_t address = OTP_GetEmptyByteAddress();
	OTP_WriteByte(id, address);	
}