#ifndef _DEV_IO_H
#define _DEV_IO_H
/**
 * @file This file contains the basic structures and rules for the IO between the program and a theoretical device
 * for now this device is a file into the hard-drive.
 * Developers can build their concept of a filesystem based on this driver before developing it completely.
 */
#include "dev_conf.h"

// device specific includes
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>


#define DEV_OK 0

typedef FILE Comm_HandleTypeDef;
typedef BYTE DSTATUS; // status returned from functions



struct dev_status_t {
	uint8_t stat;
};

typedef struct dev_status_t dev_status_t;

typedef enum
{
	dev_0 = 0,
	dev_1 = 1,
	dev_none = 2
} dev_id_t;

/**
 * This is the class which handles the IO of the theoretical device.
 */
struct dev_io_t
{
    uint8_t initialized;
	dev_id_t device_id;
	Comm_HandleTypeDef* comm_handler;
	
	DSTATUS (*init) (struct dev_io_t* self, Comm_HandleTypeDef *comm_handler, dev_id_t device_id);
	DSTATUS (*deinit) (struct dev_io_t* self);	
	DSTATUS (*reset) (const struct dev_io_t* self);
	DSTATUS (*enable) (const struct dev_io_t* self);
	DSTATUS (*disable) (const struct dev_io_t* self);
	DSTATUS (*get_status) (const struct dev_io_t* self, dev_status_t * dev_status);
	DSTATUS (*write_data) (const struct dev_io_t* self, const uint32_t start_address, const BYTE * data, const uint16_t els_size);
	DSTATUS (*read_data) (const struct dev_io_t* self, const uint32_t start_address, BYTE * data, const uint16_t els_size);
    DSTATUS (*erase_data) (const struct dev_io_t* self, const uint32_t start_address);
	DSTATUS (*wait_till_ready) (const struct dev_io_t* self, const uint32_t timeout);
};

typedef struct dev_io_t dev_io_t;

DSTATUS dev_link_create(dev_io_t * s25fl_io, Comm_HandleTypeDef *comm_handler, dev_id_t device_id);
DSTATUS dev_unlink_destroy(dev_io_t * dev_io);

#endif