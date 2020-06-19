/// @file flash.c
/// <summary>
/// FLASH driver.
/// </summary>
///
/// Supervision: /
///
/// Company: Sensum d.o.o.
///
/// @authors Erik Juvan
///
/// @version /
/////-----------------------------------------------------------
// Company: Sensum d.o.o.

#include "flash.h"

#define FLASH_USER_START_ADDR ADDR_FLASH_SECTOR_5     /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR (ADDR_FLASH_SECTOR_6 - 1) /* End @ of user Flash area */

//---------------------------------------------------------------------
/// <summary> Get FLASH sector number. </summary>
///
/// <param name="address"> Address. </param>
///
/// <returns> Sector number of given address.  </returns>
//---------------------------------------------------------------------
static uint32_t GetSector(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0)) {
        sector = FLASH_SECTOR_0;
    } else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1)) {
        sector = FLASH_SECTOR_1;
    } else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2)) {
        sector = FLASH_SECTOR_2;
    } else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3)) {
        sector = FLASH_SECTOR_3;
    } else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4)) {
        sector = FLASH_SECTOR_4;
    } else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5)) {
        sector = FLASH_SECTOR_5;
    } else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6)) {
        sector = FLASH_SECTOR_6;
    } else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
    {
        sector = FLASH_SECTOR_7;
    }
    return sector;
}

//---------------------------------------------------------------------
/// <summary> Read data from FLASH. </summary>
///
/// <param name="buffer"> Pointer to buffer to read data into. </param>
/// <param name="address"> Address to read from. </param>
/// <param name="size"> Number of bytes to read. </param>
///
/// <returns> Number of bytes read.  </returns>
//---------------------------------------------------------------------
int FLASH_Read(uint32_t* buffer, uint32_t address, int size)
{

    for (int i = 0; i < size; ++i) {
        *buffer = *(uint32_t*)address;
        buffer++;
        address++;
    }

    return size;
}

//---------------------------------------------------------------------
/// <summary> Write data to FLASH. </summary>
///
/// <param name="data"> Pointer to buffer to write data from. </param>
/// <param name="address"> Address to write data to. </param>
/// <param name="size"> Number of bytes to write. </param>
///
/// <returns> Number of bytes written. </returns>
//---------------------------------------------------------------------
int FLASH_Write(uint32_t* data, uint32_t address, int size)
{
    uint32_t FirstSector = 0, NbOfSectors = 0;
    uint32_t Address = 0, SECTORError = 0;
    __IO uint32_t data32 = 0, MemoryProgramStatus = 0;

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
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = FirstSector;
    EraseInitStruct.NbSectors    = NbOfSectors;

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

//---------------------------------------------------------------------
/// <summary> Read uC ID that was programmed by user. </summary>
///
/// <returns> ID of uC. </returns>
//---------------------------------------------------------------------
uint8_t FLASH_ReadID()
{
    return *(uint8_t*)FLASH_USER_START_ADDR;
}

//---------------------------------------------------------------------
/// <summary> Write uC ID to FLASH. </summary>
///
/// <param name="id"> User requested ID of uC. </param>
//---------------------------------------------------------------------
void FLASH_WriteID(uint8_t id)
{
    uint32_t               SECTORError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct;
    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = GetSector(FLASH_USER_START_ADDR);
    EraseInitStruct.NbSectors    = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_USER_START_ADDR, id);

    HAL_FLASH_Lock();
}

//---------------------------------------------------------------------
/// <summary> Read one byte from OTP (One Time Programmable memory). </summary>
///
/// <param name="address"> Address to read byte from. </param>
///
/// <returns> Value of the byte. </returns>
//---------------------------------------------------------------------
static uint8_t OTP_ReadByte(uint32_t address)
{
    return *(uint8_t*)address;
}

//---------------------------------------------------------------------
/// <summary> Find first available memory address in OTP (One Time Programmable memory). </summary>
///
/// <returns> Availabe memory address. </returns>
//---------------------------------------------------------------------
static uint32_t OTP_GetEmptyByteAddress()
{
    uint32_t address = ADDR_OTP_SECTOR;

    for (int i = 0; i < OTP_SECTOR_SIZE; ++i) {
        if (OTP_ReadByte(address + i) == 0xFF) { // empty byte
            return address + i;
        }
    }

    return 0;
}

//---------------------------------------------------------------------
/// <summary> Write one byte to OTP (One Time Programmable memory). </summary>
///
/// <param name="byte"> Byte to write. </param>
/// <param name="address"> Address to write byte to. </param>
//---------------------------------------------------------------------
static void OTP_WriteByte(uint8_t byte, uint32_t address)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, byte);
    HAL_FLASH_Lock();
}

//---------------------------------------------------------------------
/// <summary> Read uC ID from OTP (One Time Programmable memory). </summary>
///
/// <returns> uC ID. </returns>
//---------------------------------------------------------------------
uint8_t OTP_ReadID()
{
    uint32_t address = OTP_GetEmptyByteAddress() - 1;
    if (address >= ADDR_OTP_SECTOR)
        return *(uint8_t*)address;
    else
        return 0;
}

//---------------------------------------------------------------------
/// <summary> Write uC ID to OTP (One Time Programmable memory). </summary>
///
/// <param name="id"> User requested ID of uC. </param>
//---------------------------------------------------------------------
void OTP_WriteID(uint8_t id)
{
    uint32_t address = OTP_GetEmptyByteAddress();
    OTP_WriteByte(id, address);
}