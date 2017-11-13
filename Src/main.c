#include <ff.h>
#include <stdio.h>
#include <fatfs.h>

#define BLOCK_SIZE 16384

FATFS FatFs;   /* Work area (filesystem object) for logical drive */
FIL fp;
char file_path[100];

unsigned char dataBuffer1[512];
// unsigned char dataBuffer2[512];

int main() 
{
    int i;
    // fill a buffer with values
    for (i = 0; i < sizeof(dataBuffer1); ++i)
    {
        dataBuffer1[i] = i;
    }
    
    MX_FATFS_Init();
    volatile FRESULT fres = FR_OK;
    // create valid fatfs volume
    fres |= disk_initialize(0);
    if(fres == FR_OK)
    {
        printf("%s:%d, fres = %d\n", __FILE__, __LINE__, fres);
    }
    fres = f_mount(&FatFs, USER_Path, 1);
    if(fres == FR_OK)
    {
        printf("%s:%d, fres = %d\n", __FILE__, __LINE__, fres);
    }
    fres = f_mkfs(USER_Path, 1, (BLOCK_SIZE));//512);
    if(fres == FR_OK)
    {
        printf("%s:%d, fres = %d\n", __FILE__, __LINE__, fres);
    }
    f_mount(NULL, USER_Path, 1);

    // mount the valid FATFS volume just created
    fres = f_mount(&FatFs, USER_Path, 1);
    if(fres == FR_OK)
    {
        printf("%s:%d, fres = %d\n", __FILE__, __LINE__, fres);
    }

    sprintf(file_path,"%smydata.txt",USER_Path);
    UINT bytes_written, bytes_read;

    // write databuffer1 to a file named mydata.txt
    fres = f_open(&fp, file_path, FA_CREATE_ALWAYS | FA_WRITE);
    fres = f_write(&fp, dataBuffer1, sizeof(dataBuffer1), &bytes_written);
    fres = f_close(&fp);

    // fres = f_open(&fp, file_path, FA_READ | FA_OPEN_EXISTING);
    // fres = f_read(&fp, dataBuffer2, sizeof(dataBuffer2), &bytes_read);
    // fres = f_close(&fp);
    // for(i = 0; i < 512; ++i)
    // {
    //     printf("%d, ", dataBuffer2[i]);
    // }

    return 0;
}