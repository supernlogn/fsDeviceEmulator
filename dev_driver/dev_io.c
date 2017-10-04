/*
 *  device_driver.h
 *  Created on: 10-2017
 *  Author: Ioannis Athanasiadis (supernlogn)
 */
#include "dev_io.h"
#include "dev_action.h"

#define ACTION_WRITE 1
#define ACTION_READ 2
#define ACTION_ERASE 3
#define ACTION_FULL_ERASE 4


/* public member declarations */
static DSTATUS dev_io_init (dev_io_t* self, Comm_HandleTypeDef *comm_handler,  dev_id_t device_id);
static DSTATUS dev_io_deinit(dev_io_t* self);
static DSTATUS dev_reset (const dev_io_t* self);
static DSTATUS dev_write_data (const dev_io_t* self, const uint32_t start_address, const BYTE * data, const uint16_t els_size);
static DSTATUS dev_read_data (const dev_io_t* self, const uint32_t start_address, BYTE * data, const uint16_t els_size);
static DSTATUS dev_erase_data (const dev_io_t* self, const uint32_t start_address);
static DSTATUS dev_enable (const dev_io_t* self);
static DSTATUS dev_disable (const dev_io_t* self);
static DSTATUS dev_get_status(const dev_io_t* self, dev_status_t * dev_status);
static DSTATUS dev_wait_till_ready (const dev_io_t* self, const uint32_t timeout);

/* private member declarations */
static DSTATUS _erase_all_device(const dev_io_t * self);

/**
 * @brief initializes the device
 * @param self
 * @param comm_handler low level communication handler e.g. SPI, I2C,...  (here a file)
 * @param device_id id of the new device
 * @returns DEV_OK
 */
static DSTATUS dev_io_init (dev_io_t* self, Comm_HandleTypeDef *comm_handler,  dev_id_t device_id) 
{
    DSTATUS res = DEV_OK;
    if(!(self->initialized))
    {

    	self->device_id = device_id; // default selected device

        /* start registrating actions */
        init_action_registration();

        if(comm_handler == NULL) 
        {
            self->comm_handler = fopen(DEVICE_FILE_NAME, "wb");
            fclose(self->comm_handler);
            _erase_all_device((const dev_io_t*)self);
        }
        else 
        {
            fclose(self->comm_handler);
            self->comm_handler = fopen(DEVICE_FILE_NAME, "wb");
            fclose(self->comm_handler);
            _erase_all_device((const dev_io_t*)self);
        }

        self->disable(self);
        
        /* init whatever else */

		#ifdef HANDLE_FIRST_BOOT
        /* if this is the first time, the software boots */
        if(DEVICE_NEEDS_CLEANING(self->device_id))
        {
            _erase_all_device(self);
		    //RESET_DEVICE_NEEDS_CLEANING(self->device_id); TODO: Cannot change value to flash page this way
        }
        #endif
        self->initialized = 1;
    }

    return res;
}

/**
 * @brief de-initializes the device driver
 * @param self
 * @returns DEV_OK if all parameters are valid
 */
static DSTATUS dev_io_deinit(dev_io_t* self)
{
    DSTATUS res = DEV_OK;

    if(self == NULL)
    {
        return 0xFF; // error
    }

    self->device_id = (dev_id_t) (-1);
    // fclose(self->comm_handler);
    self->comm_handler = NULL;

    self->initialized = 0;
    return res;
}
/**
 * @brief restarts the device.
 * @param self
 * @retval HAL_OK if the device chosen was one of the 2, else 0xFF
 */
static DSTATUS dev_reset (const dev_io_t* self) 
{
    DSTATUS res = DEV_OK;
    
    if(self->device_id != dev_none)
    {
		self->disable(self);

    }
    else
    {
    	res = 0xFF;
    }

    self->enable(self);
    self->disable(self);

    return res;
}
/**
 * @brief writes data to the selected device.
 * @param self
 * @param start_address Address inside a device, not the one used inside the file system.
 * @param data Data to write inside device
 * @param els_size size of data buffer in bytes. Must be <= 512U
 * @returns if writing data was sucessfull
 *   DEV_OK when all were good.
 */
static DSTATUS dev_write_data (const dev_io_t* self, const uint32_t start_address, const BYTE * data, const uint16_t els_size) 
{
    DSTATUS res = DEV_OK;


    if(els_size > MAX_WRITE_SIZE)
    {
		res = 0xF0; // error
		return res;
    }

    BYTE * temp_data = (BYTE*) malloc(els_size * sizeof(BYTE));
    FILE * file = fopen(DEVICE_FILE_NAME, "rb+");
    fseek(file, start_address, SEEK_SET);
    res &= (fread(temp_data, sizeof(BYTE), els_size, file) == els_size);
    APPLY_DEVICE_OPERATOR(temp_data, data, els_size);
    fwrite(temp_data, sizeof(BYTE), els_size, file);
    fseek(file, start_address, SEEK_SET);
    fclose(file);
    if(els_size == fwrite(temp_data, sizeof(BYTE), els_size, file) )
    {
        res = 0;
    }
    else
    {
        res = 0x0F; // error
    }

    register_action(ACTION_WRITE, start_address, start_address + els_size, temp_data, els_size);

    free(temp_data);

    return res;
}
/**
 * @brief reads data from the selected device to a buffer in RAM.
 * @param self
 * @param start_address Address inside the device, not the one used inside the file system.
 * @param data Databuffer to be filled with data from inside the device
 * @param els_size size of data buffer in bytes
 * @returns if reading data was sucessfull
 *   DEV_OK when all were good.
 */
static DSTATUS dev_read_data (const dev_io_t* self, const uint32_t start_address, BYTE * data, const uint16_t els_size) 
{
    DSTATUS res = DEV_OK;

    FILE * file = fopen(DEVICE_FILE_NAME, "rb+");
    fseek(file, start_address, SEEK_SET);
    res &= (fread(data, sizeof(BYTE), els_size, file) == els_size);
    fclose(file);

    register_action(ACTION_READ, start_address, start_address + els_size, data, els_size);
    return res;
}
/**
 * @brief erases FLASH per BLOCK_SIZE.
 * @param self
 * @param start_address starting address from where to start erasing data.
 * @param els_size size of erase part
 * @return whether erase command was sent.
 *   HAL_OK when all were good.
 */
static DSTATUS dev_erase_data (const dev_io_t* self, const uint32_t start_address) 
{
    DSTATUS res = DEV_OK;
    register_action(ACTION_ERASE, start_address, start_address + BLOCK_SIZE, NULL, 0);

    FILE * file = fopen(DEVICE_FILE_NAME, "rb+");
    BYTE * temp_data = (BYTE*) malloc(BLOCK_SIZE * sizeof(BYTE));
    DEVICE_ZERO_DATA(temp_data, BLOCK_SIZE);

    fseek(file, start_address, SEEK_SET);
    fwrite(temp_data, sizeof(BYTE), BLOCK_SIZE, file);
    fclose(file);

    free(temp_data);

    return res;
}
/**
 *  @brief enables the device
 *  @param self
 *  @retval DEV_OK
 */
static DSTATUS dev_enable (const dev_io_t* self) 
{
    DSTATUS res = DEV_OK;

    return res;
}
/**
 *  @brief disables the device
 *  @param self
 *  @retval DEV_OK
 */
static DSTATUS dev_disable (const dev_io_t* self) 
{
    DSTATUS res = DEV_OK;

    return res;
}
/**
 * @brief get status of the device
 * @note for now do nothing
 */
static DSTATUS dev_get_status(const dev_io_t* self, dev_status_t * dev_status) 
{
    DSTATUS res = DEV_OK;

    dev_status->stat = 123;

    return res;
}

/**
 * @brief wait until device has no more pending request
 * @param self
 * @param timeout the amount of time to wait, until the pending requests have been executed
 * @retval if all the requests have been executed before timeout then return DEV_OK, else return 0xFF
 */
static DSTATUS dev_wait_till_ready (const dev_io_t* self, const uint32_t timeout) 
{
    DSTATUS res = DEV_OK;
    
    // do nothing and just return

    return res;
}


static DSTATUS _erase_all_device(const dev_io_t * self)
{
    DSTATUS res = DEV_OK;
    register_action(ACTION_FULL_ERASE, 0, LAST_BLOCK_ADDR + BLOCK_SIZE, NULL, 0);

    int i;

    if(self->comm_handler != NULL)
    {
        BYTE *temp_data = (BYTE*) malloc(BLOCK_SIZE * sizeof(BYTE));

        for(i = 0; i < BLOCK_SIZE; ++i)
        {
            temp_data[i] = DEV_ERASE_CHAR;
        }
        FILE * file = fopen(DEVICE_FILE_NAME,"rb+");
        fwrite(temp_data, sizeof(BYTE), BLOCK_SIZE, file);
        fclose(file);  

        free(temp_data);
    }
    else
    {
        res = 0xFF; // error
    }

    return res;
}


/**
 * This function links the needed resources of a dev_io_t object to initialize a dev_io_t object
 * @param dev_io device handler to create
 * @param comm_handler communication handler (here is just a file, but it can also be an SPI or something else)
 * @param device_id id of the new device
 * @returns 0 if all were ok, something else if sth went bad.
 */
DSTATUS dev_link_create(dev_io_t * dev_io, Comm_HandleTypeDef *comm_handler, dev_id_t device_id)
{
	DSTATUS res = DEV_OK;
	if(dev_io == NULL)
	{
		return 0xFF; // error
	}

	dev_io->initialized = 0;
    dev_io->init = dev_io_init;
	dev_io->deinit = dev_io_deinit;
	dev_io->reset = dev_reset;
	dev_io->write_data = dev_write_data;
	dev_io->read_data = dev_read_data;
	dev_io->erase_data = dev_erase_data;
	dev_io->enable = dev_enable;
	dev_io->disable = dev_disable;
	dev_io->get_status = dev_get_status;

    dev_io->wait_till_ready = dev_wait_till_ready;

	dev_io->init(dev_io, comm_handler, device_id);

	return res;
}

/**
 * This function unlinks all the resources of a dev_io_t object
 * @param dev_io device handler to create
 * @returns 0 if all were ok, something else if sth went bad. 
 */
DSTATUS dev_unlink_destroy(dev_io_t * dev_io)
{
	DSTATUS res = DEV_OK;
	if(dev_io == NULL)
	{
		return 0xFF; // error
	}

	dev_io->deinit(dev_io);
    
    dev_io->init = NULL;
	dev_io->deinit = NULL;
	dev_io->reset = NULL;
	dev_io->write_data = NULL;
	dev_io->read_data = NULL;
	dev_io->erase_data = NULL;
	dev_io->enable = NULL;
	dev_io->disable = NULL;
	dev_io->get_status = NULL;
    
    dev_io->wait_till_ready = NULL;

    // you can now free(dev_io);

	return res;
}