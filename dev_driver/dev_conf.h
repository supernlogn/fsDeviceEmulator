/*
 *  dev_conf.h
 *  Created on: 10-2017
 *  Author: Ioannis Athanasiadis (supernlogn)
 */

#ifndef _DEV_CONF_H_
#define _DEV_CONF_H_

#define osWaitForever (0xFFFFFFFFU)

/* These parameters below can be set outside the program. */

#define NUM_BLOCKS 512
#define BLOCK_SIZE (64 * 1024)
#define FLASH_SIZE (NUM_BLOCKS * BLOCK_SIZE)
#define LAST_BLOCK_ADDR ((NUM_BLOCKS - 1) * BLOCK_SIZE)
#define MAX_WRITE_SIZE 256U

#ifndef DEVICE_FILE_NAME
  #define DEVICE_FILE_NAME "s25fl.dat"
#endif

#ifndef DEVICE_ACTION_FILE_NAME
  #define DEVICE_ACTION_FILE_NAME "s25fl_action.json"
#endif

#define OR_DEV_TYPE 0
#define AND_DEV_TYPE 1

#ifndef DEVICE_TYPE
  #define DEVICE_TYPE AND_DEV_TYPE
#endif

#if DEVICE_TYPE == OR_DEV_TYPE
#define DEV_OP(x,y) x | y
#define DEV_ERASE_CHAR 0
#elif DEVICE_TYPE == AND_DEV_TYPE
#define DEV_OP(x,y) x & y
#define DEV_ERASE_CHAR 0xFF
#endif

#define SEND_DEVICE_RESEND_COMMAND(file) (0)

#define APPLY_DEVICE_OPERATOR(temp_data, data, els_size)\
{\
 int i;\
 for( i = 0; i < els_size; ++i)\
 {\
    temp_data[i] = DEV_OP(temp_data[i], data[i]);\
 }\
}

#define DEVICE_ZERO_DATA(temp_data, BLOCK_SIZE)\
{\
    int i;\
    for( i = 0; i < BLOCK_SIZE; ++i)\
    {\
       temp_data[i] = DEV_ERASE_CHAR;\
    }\
}


typedef unsigned char	BYTE;


#define DEVICE_NEEDS_CLEANING(x) 0
#endif