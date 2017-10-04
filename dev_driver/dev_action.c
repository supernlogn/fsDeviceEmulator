#include "dev_action.h"


static int action_count = 0;

static DSTATUS _print_action(action_t * action);
static void _print_action_name(FILE * file, const action_name_t name);
static void _get_trace_strings(action_t * action);

static DSTATUS _print_action(action_t * action)
{
    DSTATUS res = DEV_OK;
    FILE * action_file;
    int i, j;

    action_file = fopen(DEVICE_ACTION_FILE_NAME,"r+");
    fseek(action_file, 0, SEEK_END);

    if(action_file != NULL)
    {
        if(action_count >= 1)
        {
            fseek(action_file, -4, SEEK_CUR);
            fprintf(action_file, ",");
        }
        else
        {
            fprintf(action_file, "{\n\"actions\":[");
        }
        fprintf(action_file, "\n{");
        _print_action_name(action_file, action->name);
        fprintf(action_file, ",\n \"start_address\": \"0x%10.10x\"", action->start_address);
        fprintf(action_file, ",\n \"end_address\": \"0x%10.10x\"", action->end_address);        
        fprintf(action_file, ",\n \"stack\":[\n");
        fprintf(action_file, "\t \"%s\"", action->trace_strings[0]);
        for(i = 1; i < action->num_frames; ++i)
        {
            fprintf(action_file, ",\n\t \"%s\"", action->trace_strings[i]);
        }
        fprintf(action_file, "\n\t ]");
        
        if(action->data != NULL)
        {
            fprintf(action_file, ",\n \"data\": [\n\t");

            for( i = 0; i < action->data_size - 1; i+= 16)
            {
                for( j = 0; j < 16; ++j)
                {
                    fprintf(action_file, "\"0x%2.2x\",", action->data[j]);
                }
                fprintf(action_file, "\n\t");
            }
            for( i = i - 16; i < action->data_size - 1; ++i)
            {
                fprintf(action_file, "\"0x%2.2x\",", action->data[i]);
            }

            fprintf(action_file, "\"0x%2.2x\"", action->data[action->data_size - 1]);
            fprintf(action_file, "\n\t]");
        }
        
        fprintf(action_file, "\n}\n]\n}");
        fclose(action_file);

        ++action_count;
    }
    else
    {
        res = 0xFF; // error
    }

    return res;
}
/**
 * @brief print the name of the action inside the file
 * @file the file where to print the action name
 * @name the name of the action
 */ 
static void _print_action_name(FILE * file, const action_name_t name)
{
    switch(name)
    {
        case ACTION_WRITE:
        {
            fprintf(file, "\n \"name\":\"%s\"", "WRITE");
            break;
        }
        case ACTION_READ:
        {
            fprintf(file, "\n \"name\":\"%s\"", "READ");
            break;
        }
        case ACTION_ERASE:
        {
            fprintf(file, "\n \"name\":\"%s\"", "ERASE");
            break;
        }
        case ACTION_FULL_ERASE:
        {
            fprintf(file, "\n \"name\":\"%s\"", "FULL_ERASE");
            break;
        }
    }
}
/**
 * @brief get the strings trace
 * @param action
 */ 
static void _get_trace_strings(action_t * action)
{
    unw_cursor_t cursor; unw_context_t uc;
    int depth = 0;
    int i;
    unw_word_t called_address;
    /* get depth */
    unw_getcontext(&uc);
    unw_init_local(&cursor, &uc);
    unw_step(&cursor);
    unw_step(&cursor); 
    while (unw_step(&cursor) > 0)
    {
        depth++;
    }
    action->num_frames = depth;
    action->trace_strings = (char**) malloc( depth * sizeof(char*));

    unw_getcontext(&uc);
    unw_init_local(&cursor, &uc);
    unw_step(&cursor);
    unw_step(&cursor);
    for(i = 0; i < depth; ++i)
    {
        unw_step(&cursor);
        action->trace_strings[i] = (char*) malloc(100 * sizeof(char));
        unw_get_proc_name(&cursor, action->trace_strings[i], 100, &called_address);
    }

}


/**
 * @brief start the registration actions.
 * If this function is not called before the registration actions, 
 * then the program will crush. 
 */
DSTATUS init_action_registration()
{
    DSTATUS res = DEV_OK;
    FILE * action_file = fopen(DEVICE_ACTION_FILE_NAME, "w");

    if(action_file == NULL)
    {
        res = 0xFF; // error
    }
    else
    {
        fclose(action_file);
    }

    return res;
}

/**
 * @brief registers an action and writes its info to the file defined by DEVICE_ACTION_FILE_NAME
 * @param name the action's name.
 * @param start_address the start address inside the device memory, which was used by this action. 
 * @param end_address the end address inside the device memory, which was used by this action.
 * @param data the data that were sent or received by the device driver.
 * @param data_size the number of bytes inside data.
 */ 
DSTATUS register_action(const action_name_t name, const uint32_t start_address, const uint32_t end_address, const BYTE * data, uint32_t data_size)
{
    DSTATUS res = DEV_OK;

    action_t new_action;
    int i;

    new_action.name = name;
    new_action.start_address = start_address;
    new_action.end_address = end_address;
    new_action.data = data;
    new_action.data_size = data_size;

    _get_trace_strings(&new_action);

    _print_action(&new_action);
    
    for( i = 0; i < new_action.num_frames; ++i)
    {
        free(new_action.trace_strings[i]);        
    }
    free(new_action.trace_strings);
    
    return res;
}



