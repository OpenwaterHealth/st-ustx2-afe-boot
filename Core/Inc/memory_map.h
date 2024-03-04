#ifndef INC_MEMORY_MAP_H_
#define INC_MEMORY_MAP_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* proposed memory map 128 bytes 0-1023 indexes total 128KB Flash 6KB  EEPROM 20KB RAM
 	INDEX_0 				Bootloader					0x08000000		64K
 	INDEX_128 				Application					0x08010000		64K
 	INDEX_640 				Application					0x08020000		64K
 	EEPROM_1			    Config						0x08080000
 	EEPROM_2			    Config						0x08080C00
 */

#define BOOTLOADER_ADDRESS        			((uint32_t)0x08000000)
#define APPLICATION_ADDRESS        			((uint32_t)0x08010000)

#ifdef __cplusplus
}
#endif

#endif /* INC_MEMORY_MAP_H_ */
