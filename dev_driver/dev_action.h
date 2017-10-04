#ifndef _DEV_ACTION_H
#define _DEV_ACTION_H
#include "dev_conf.h"
#include "dev_io.h"
#include <stdlib.h>
#include <stdio.h>
#define UNW_LOCAL_ONLY
#include <libunwind.h>

typedef enum {
    ACTION_WRITE = 1,
    ACTION_READ = 2,
    ACTION_ERASE = 3,
    ACTION_FULL_ERASE = 4,
} action_name_t;

typedef struct {
    action_name_t name;
    uint32_t start_address;
    uint32_t end_address;
    BYTE * data;
    uint32_t data_size;
    char ** trace_strings;
    int num_frames;
}action_t;


DSTATUS init_action_registration();

DSTATUS register_action(const action_name_t name, const uint32_t start_address, const uint32_t end_address, const BYTE * data, uint32_t data_size);




#endif