#include <zephyr/drivers/spi.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>
#include <zephyr/logging/log.h>

#include "SD_card.h"

LOG_MODULE_REGISTER(SDCard, LOG_LEVEL_INF);


static const struct device *spi_dev;

#define SPI_DEVICE_NODE DT_NODELABEL(spi1)
#define SD_CS_PIN 7//3 // Chip Select pin for SD card, chosen as P0.07

static struct spi_cs_control cs_ctrl;

#define SPI_OP  SPI_OP_MODE_MASTER |SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE

#define SD_CARD_MOUNT_POINT "/SD:"
// #define LOG_FILENAME //"test_read.txt" 
#define LOG_FILENAME "adc_log.txt"


// bool   ready_to_save_SD = false;
 FATFS fat_fs;

//  extern fs_mount_t mp;
 struct fs_mount_t mp = {
        .type = FS_FATFS,
        .mnt_point = SD_CARD_MOUNT_POINT,
};
static int init_sd_card(void);
int spi_init(void){
        
    LOG_INF("Initializing SPI device...\n");
    // Initialize SPI device
    spi_dev = DEVICE_DT_GET(SPI_DEVICE_NODE);
    if (!device_is_ready(spi_dev)) {
        LOG_DBG("SPI device not ready\n");
            return -1;
    } else {
        
        LOG_INF("SPI initialization successful!");
    }

      if (init_sd_card() != 0) {
        LOG_ERR("Failed to initialize SD card\n");
            return -1;
    }
    else{
        LOG_INF("SD card initialization successful!");
        return 0;
    }
}

static int init_sd_card(void)
{
        static const char *disk_pdrv = "SD";
        uint64_t memory_size_mb;
        uint32_t block_count;
        uint32_t block_size;
        int err;

        err = disk_access_init(disk_pdrv);
        if (err != 0) {
                printk("disk_access_init failed. Error code: %d (%s)\n", err, strerror(err));
                return -1;
        }

        if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
                printk("Unable to get sector count");
                return -1;
        }

        if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
                printk("Unable to get sector size");
                return -1;
        }

        memory_size_mb = (uint64_t)block_count * block_size / (1024 * 1024);
        printk("Memory Size(MB): %u\n", (uint32_t)memory_size_mb);

        mp.fs_data = &fat_fs;

        err = fs_mount(&mp);
        if (err) {
                printk("Error mounting fat_fs [%d]\n", err);
                return err;
        }
        mounted_card = true;
        printk("Disk mounted.\n");

        // Configure Chip Select
        cs_ctrl.gpio.port = DEVICE_DT_GET(DT_NODELABEL(gpio0));
        cs_ctrl.gpio.pin = SD_CS_PIN;
        cs_ctrl.gpio.dt_flags = GPIO_ACTIVE_LOW;
        cs_ctrl.delay = 0;
        
        k_sleep(K_MSEC(500));  // Wait 500ms before SD init
        
        return 0;
}

struct fs_file_t file;
char log_entry[100];
size_t bytes_written;

 int sd_file_open(void)
{
       
    fs_file_t_init(&file);

    LOG_DBG("File System Initialized. Opening the file now. \n");
    LOG_INF("File System Initialized. Opening the file now. \n");
    
    
    // int err = fs_open(&file, SD_CARD_MOUNT_POINT "/" LOG_FILENAME, FS_O_READ);// | FS_O_APPEND);
    int err = fs_open(&file, SD_CARD_MOUNT_POINT "/" LOG_FILENAME, FS_O_RDWR);    
    if (err) {
        LOG_ERR("Error opening file [%d]\n", err);
        return err;
    }
    LOG_DBG("File opened\n");

    opened_file = true;
    
    return 0;
}

void list_files_on_sd(void) {
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int ret;

    fs_dir_t_init(&dir);

    ret = fs_opendir(&dir, SD_CARD_MOUNT_POINT);
    if (ret != 0) {
        printk("Failed to open SD directory: %d\n", ret);
        return;
    }

    printk("Files on SD card:\n");
    while ((ret = fs_readdir(&dir, &entry)) == 0 && entry.name[0] != 0) {
        printk("  %s%s\n", entry.name, entry.type == FS_DIR_ENTRY_DIR ? "/" : "");
    }

    fs_closedir(&dir);
}
#define MAX_LINE_LENGTH 128
#define MAX_LINES 1000
int sd_file_read_line(void) {
        char read_buf[MAX_LINE_LENGTH];
        
        
        // Check if the file is opened
        if (!opened_file) {
                LOG_INF("File is not opened\n");
            sd_file_open();
            if (!opened_file) {
                LOG_ERR("Failed to open file\n");
                return -1;
            }
        }
        else{
            LOG_INF("File is opened\n");
        }
        
    
        LOG_INF("Reading contents of the file...\n");
    
        // Read line-by-line
        while (true) {
            ssize_t bytes = fs_read(&file, read_buf, sizeof(read_buf) - 1);
            if (bytes <= 0) {
                LOG_INF("End of File Reached or Error\n");

                return -1; 
            }
            
            read_buf[bytes] = '\0'; // Null-terminate string
            printk("%s", read_buf);
        }
    
        fs_close(&file);
        return 0;
    }
// the function should be able to read the data just enough to fil the memory block pasased to it. 
int sd_read_fixed_bytes(uint8_t *out_data, size_t out_size) {
        

    ssize_t bytes_read = fs_read(&file, out_data, out_size - 1);
    if (bytes_read < 0) {
        LOG_ERR("Error reading file: %zd\n", bytes_read);        
        // *out_data = NULL;
        return -EIO;
    }
    // buffer[bytes] = '\0'; // Null-terminate string
    out_data[bytes_read] = '\0'; // Null-terminate string
    printk("%s", out_data);
    

    return bytes_read; // Return the number of bytes read
}
 int sd_card_unmount(){
    int err = fs_unmount(&mp);
    if (err) {
        printk("Error unmounting file system [%d]\n", err);
        return err;
    }
    mounted_card = false;
    LOG_DBG("Disk unmounted.\n");
    return 0;
}
 int sd_file_close(void)
{
        int err = fs_close(&file);
        if (err){
            LOG_ERR("Error closing the file %d. \n", err);
            return err; 
        }
        opened_file=false;
        LOG_INF("Closed the file on SD Card. \n");
 
        return 0;
}


int sd_safe_close(){

        if (opened_file && safe_to_close_SD) {
                sd_file_close();
                opened_file = false;
        }
        if (mounted_card){
                sd_card_unmount();
                mounted_card = false;
        }
        return 0;
}

size_t sd_get_file_size(void) {
    struct fs_dirent file_info;
    int err = fs_stat(SD_CARD_MOUNT_POINT "/" LOG_FILENAME , &file_info);
    if (err == 0) {
        LOG_INF("File size of '%s': %zu bytes", SD_CARD_MOUNT_POINT "/" LOG_FILENAME, file_info.size);
    } else {
        LOG_ERR("Failed to get file info (%d)", err);
    }

    return file_info.size; // Return the size of the file
}


// int sd_file_write(uint16_t adc_raw, float voltage){
    
//     int64_t timestamp_ms = k_uptime_get();  // Get system uptime in ms
//     // snprintf(log_entry, sizeof(log_entry), "Time: %lld ms, ADC raw: %d, Voltage: %.2f V\n", timestamp_ms, adc_raw, voltage);


//     snprintf(log_entry, sizeof(log_entry), "%lld ms, %d, %.2f V\n", timestamp_ms, adc_raw, (double)voltage);
    
//     bytes_written = fs_write(&file, log_entry, strlen(log_entry));
//     if (bytes_written < 0) {
//         LOG_ERR("Error writing to file [%zd]\n", bytes_written);
//         fs_close(&file);
//         return (int)bytes_written;
//     }

//     LOG_DBG("Data logged to SD card\n");
//     return 0;
// }