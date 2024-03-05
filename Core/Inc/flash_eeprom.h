/*
 * flash_eeprom.h
 *
 *  Created on: Mar 4, 2024
 *      Author: gvigelet
 */

#ifndef INC_FLASH_EEPROM_H_
#define INC_FLASH_EEPROM_H_

#include "stm32l0xx_hal.h"
#include "memory_map.h"


/* Function prototypes */
HAL_StatusTypeDef EEPROM_Write(uint16_t address, uint8_t* data, uint16_t size);
HAL_StatusTypeDef EEPROM_Read(uint16_t address, uint8_t* data, uint16_t size);

HAL_StatusTypeDef Flash_Write(uint32_t address, uint32_t* data, uint32_t size);
HAL_StatusTypeDef Flash_Read(uint32_t address, uint32_t* data, uint32_t size);
HAL_StatusTypeDef Flash_Erase(uint32_t start_address, uint32_t end_address);


#endif /* INC_FLASH_EEPROM_H_ */
