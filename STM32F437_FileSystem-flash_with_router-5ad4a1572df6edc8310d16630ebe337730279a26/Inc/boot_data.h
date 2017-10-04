/*
 * boot_data.h
 *
 *  Created on: 14 Ιουν 2017
 *      Author: ath.io
 */

#ifndef BOOT_DATA_H_
#define BOOT_DATA_H_

//#define HANDLE_FIRST_BOOT 1
#ifdef HANDLE_FIRST_BOOT
typedef struct boot_data_t {
	uint32_t is_first_boot;
	uint32_t flash_needs_cleaning[2];
}boot_data_t;

/*
 * boot_data holds all the data needed
 * to describe the differenec between the first boot
 * and others.
 * If somehow the first boot does not go well, then
 * one or more variables will hold their starting values
 * and the software can continue at at the next sucessful
 * boot continue from where it started.
 */
extern boot_data_t boot_data;

#define IS_FIRST_BOOT() (boot_data.is_first_boot)
#define RESET_FIRST_BOOT() (boot_data.is_first_boot = 0)
#define SET_FIRST_BOOT() (boot_data.is_first_boot = 1U)
#define FLASH_NEEDS_CLEANING(FL_NUM) (boot_data.flash_needs_cleaning[FL_NUM])
#define RESET_FLASH_NEEDS_CLEANING(FL_NUM) (boot_data.flash_needs_cleaning[FL_NUM] = 0)
#define SET_FLASH_NEEDS_CLEANING() (boot_data.flash_needs_cleaning = 1U)
#endif

#endif /* BOOT_DATA_H_ */
