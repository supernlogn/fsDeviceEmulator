/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* 
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future. 
 * Kept to ensure backward compatibility with previous CubeMx versions when 
 * migrating projects. 
 * User code previously added there should be copied in the new user sections before 
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
// #include "user_diskio.h"
#include <string.h>
#include "ff_gen_drv.h"
#include <dev_io.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MEM_SIZE (1 << 20) // 1 MByte
#define SECTOR_SIZE _MIN_SS
#define SECTORS_IN_BLOCK (BLOCK_SIZE / _MAX_SS)
#define LAST_BLOCK_ADDR ((NUM_BLOCKS - 1) * BLOCK_SIZE)


/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;


/* each bit in the array repressents a sector and it is
 * 1 if the sector is written
 * and 0 if it is erased or to be erased
 */
uint32_t block_sectors_status[_VOLUMES * (MEM_SIZE / SECTOR_SIZE) / 32];
uint8_t temp_buff[SECTOR_SIZE];

/**
 * @brief Get the index of the block containing this sector
 * @param sector_num the number of the sector starting to count upwards from the start of FLASH
 * @retval the index of the block containing this sector
 */
static inline int sector_to_block_wr_table_index(const int sector_num)
{
	return (sector_num) / SECTORS_IN_BLOCK;
}
/**
 * @brief Get the index of the block containing the addr
 * @param addr the addr of a byte starting to count upwards from the start of FLASH.
 * @retval the index of the block containing the addr
 */
static inline int addr_to_block_wr_table_index(const uint32_t addr)
{
	return addr / BLOCK_SIZE;
}

static dev_io_t dev_io[1];
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/

DSTATUS USER_initialize(BYTE pdrv);
DSTATUS USER_status(BYTE pdrv);
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef USER_Driver =
    {
        USER_initialize,
        USER_status,
        USER_read,
#if _USE_WRITE
        USER_write,
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
        USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize(
    BYTE pdrv /* Physical drive nmuber to identify the drive */
    )
{
  /* USER CODE BEGIN INIT */
  DSTATUS stat = STA_NOINIT;
  uint32_t i,j,k;
  stat = 0;


  /* create the new device */
  dev_link_create(dev_io, NULL, 0);

  /* read block_wr_table from the last real sector of flash */
  stat |= dev_io->read_data(dev_io, 127U * BLOCK_SIZE, (BYTE*) block_sectors_status, sizeof(block_sectors_status));
  stat |= dev_io->wait_till_ready(dev_io, osWaitForever);

  for(i = 0; i < _VOLUMES; ++i)
  {
    for(j = 0; j < NUM_BLOCKS; ++j)
    {
      for(k = 0; k < SECTORS_IN_BLOCK/32; ++k)
      {
        block_sectors_status[(i  * (NUM_BLOCKS) + j) * (SECTORS_IN_BLOCK/32) + k] = 0U;
      }
    }
  }

  return stat;
}
/**
 * whether the media removal is lopcked or not.
 */
static uint8_t device_locked = 0;
/**
 * @brief implementation of a dummy device locking
 * @retval whether lock or unlock was ok.
 */
DRESULT lock_unlock(void)
{
	DRESULT res = RES_OK;
	if(device_locked)
	{	// then unlock
		device_locked = 0;
	}
	else
	{	// then lock
		device_locked = 1;
  }
  
	return res;
}
/**
 * @param sec_num_from the sector number (not its address!) from which to start erasing
 * @param sec_num_to the sector number (not its address!) to end erasing
 * @retval whether the erasing request was ok.
 * @note not used yet
 */
DRESULT erase_block(dev_id_t pdrv, DWORD sec_num_from, DWORD sec_num_to)
{
	DRESULT res = RES_OK;
	uint32_t i,j;
	uint32_t ans = 0;

	if(dev_io->device_id != pdrv)
	{
    res = 0xFF;
    return res; // error
	}

	for(i = sec_num_from; i < sec_num_to; ++i )
	{
		block_sectors_status[(pdrv  * (NUM_BLOCKS)) * (SECTORS_IN_BLOCK/32) + i/32] &= ~(1 << (i % 32));
	}
	
	const uint32_t block_to_check1 = sector_to_block_wr_table_index(sec_num_from);
	const uint32_t block_to_check2 = sector_to_block_wr_table_index(sec_num_to);

	for(i = block_to_check1; i < block_to_check2; ++i)
	{
		ans = 0;
		for(j = i * SECTORS_IN_BLOCK; j < ((i + 1) * SECTORS_IN_BLOCK); j += 32)
		{
      ans |= block_sectors_status[(pdrv  * (NUM_BLOCKS)) * (SECTORS_IN_BLOCK/32) + (j/32)];
			if(ans)
			{
				break;
			}
		}
		if(!ans)
		{
			dev_io->erase_data(dev_io, i * BLOCK_SIZE);
		}
	}
	
	return res;
}

/**
  * @brief  Gets Disk Status 
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status(
    BYTE pdrv /* Physical drive nmuber to identify the drive */
    )
{
  Stat = STA_NOINIT;
  Stat = 0;
  return Stat;
}

/**
  * @brief  Reads Sector(s) 
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read(
    BYTE pdrv,    /* Physical drive nmuber to identify the drive */
    BYTE *buff,   /* Data buffer to store read data */
    DWORD sector, /* Sector address in LBA */
    UINT count    /* Number of sectors to read */
    )
{
	DSTATUS res = RES_OK;
  int i;
  
  if(dev_io->device_id != pdrv)
	{
    res = 0xFF;
    return res; // error
	}
	uint32_t addr = sector * SECTOR_SIZE;

	res |= dev_io->read_data(dev_io, addr, buff, count * SECTOR_SIZE);

  for(i = 0; i < count * SECTOR_SIZE; ++i) {
		buff[i] = ~buff[i];
	}
	return res;

}

/**
  * @brief  Writes Sector(s)  
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write(
    BYTE pdrv,        /* Physical drive nmuber to identify the drive */
    const BYTE *buff, /* Data to be written */
    DWORD sector,     /* Sector address in LBA */
    UINT count        /* Number of sectors to write */
    )
{
  /* USER CODE BEGIN WRITE */
	DSTATUS res = RES_OK;
	if(dev_io->device_id != pdrv)
	{
    res = 0xFF;
    return res; // error
	}
	uint32_t addr = sector * SECTOR_SIZE;

	int i;
	int j;
	int k;

	for(i = 0; i < count; ++i)
	{
		for(k = 0; k < SECTOR_SIZE; ++k) {
			temp_buff[k] = ~buff[(i * SECTOR_SIZE) + k];
		}
		for(j = 0; j < (SECTOR_SIZE/ MAX_WRITE_SIZE); ++j)
		{
			addr = (sector + i) * SECTOR_SIZE + j * MAX_WRITE_SIZE;

			dev_io->write_data(dev_io, addr, &temp_buff[j * MAX_WRITE_SIZE], MAX_WRITE_SIZE);
		}
		/* mark that this sector is written */
		block_sectors_status[(pdrv * NUM_BLOCKS * SECTORS_IN_BLOCK/32) + (sector + i)/32] |= (1 << ((sector + i)%32));
	}

	/* USER CODE HERE */
    return res;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation  
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(
    BYTE pdrv, /* Physical drive nmuber (0..) */
    BYTE cmd,  /* Control code */
    void *buff /* Buffer to send/receive control data */
    )
{
  /* USER CODE BEGIN IOCTL */
  DRESULT res = RES_OK;
  switch (cmd)
  {
  case CTRL_SYNC:
  {
    res = dev_io->wait_till_ready(dev_io, osWaitForever);
    break;
  }
  case GET_SECTOR_COUNT:
  {
    DWORD *dbuff = (buff);
    *dbuff = (DWORD) ((NUM_BLOCKS - 1U) * (SECTORS_IN_BLOCK) ); // 127+1 uniform blocks of 256 kByte each
    res = RES_OK;
    break;
  }
  case GET_SECTOR_SIZE:
  {
    DWORD *dbuff = (buff);
    *dbuff = (DWORD) (SECTOR_SIZE); // 128 uniform blocks of 256 kByte each
    res = RES_OK;
    break;
  }
  case GET_BLOCK_SIZE:
  {
    DWORD *dbuff = (buff);
    *dbuff = (DWORD)(BLOCK_SIZE); // 128 uniform blocks of 256 kByte each in sector unit size
    break;
  }
  case CTRL_TRIM:
  {
    DWORD * sec_nums = (buff);
    res = erase_block(pdrv, sec_nums[0], sec_nums[1]);
    break;
  }
  case CTRL_POWER:
  {

    break;
  }
  case CTRL_LOCK:
  {
    res = lock_unlock();
    break;
  }
  case CTRL_EJECT:
  {

    res = RES_OK;
    break;
  }
  case CTRL_FORMAT:
  {
    // do nothing
    res = RES_OK;
    break;
  }
  case MMC_GET_TYPE:
  {
    BYTE *bbuff = (buff);
    *bbuff = (BYTE)(1 << 3); // LBA
    res = RES_OK;
    break;
  }
  case MMC_GET_CSD:
  {
    res = RES_OK;
    break;
  }
  case MMC_GET_CID:
  {
    res = RES_OK;
    break;
  }
  case MMC_GET_OCR:
  {
    break;
  }
  case MMC_GET_SDSTAT:
  {
    dev_status_t *bbuff = (buff);
    dev_io->get_status(dev_io, bbuff);
    break;
  }
  case ATA_GET_REV:
  {
    //    		res = RES_OK;
    break;
  }
  case ATA_GET_MODEL:
  {
    //    		res = RES_OK;
    break;
  }
  case ATA_GET_SN:
  {
    //    		res = RES_OK;
    break;
  }
  default:
  {
    break;
  }
  }
  return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
