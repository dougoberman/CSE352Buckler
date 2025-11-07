
#ifndef SD_CARD_H
#define SD_CARD_H

#include <zephyr/fs/fs.h>
#include <ff.h>



// Audio Configuration
#define SAMPLE_FREQUENCY 16000
#define SAMPLE_BIT_WIDTH 32
#define BYTES_PER_SAMPLE sizeof(int32_t)
#define NUMBER_OF_CHANNELS 1
#define SAMPLES_PER_BLOCK ((SAMPLE_FREQUENCY /16) * NUMBER_OF_CHANNELS) // 1000 samples per block
#define BLOCK_SIZE (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)   // 400 bytes per block
#define BLOCK_COUNT 10  // Memory slab of 10 blocks
#define RECORD_DURATION_MS 4800
#define MAX_SESSIONS 10

typedef struct audio_block_t {
    uint8_t data[BLOCK_SIZE];
    size_t size;
} audio_block_t;


#define SD_CARD_MOUNT_POINT "/SD:"
// #define LOG_FILENAME "adc_log.txt"

extern FATFS fat_fs;
extern struct fs_file_t file;
extern struct fs_mount_t mp;
//  = {
//         .type = FS_FATFS,
//         .mnt_point = SD_CARD_MOUNT_POINT,
// };

// static const struct device *spi_dev;
// static struct spi_cs_control cs_ctrl;

extern char log_entry[100];
extern size_t bytes_written;
extern bool mounted_card;
extern bool opened_file;
extern bool ready_to_save_SD;
extern bool safe_to_close_SD;

// Function prototypes
int spi_init(void);
int sd_file_open(void);
size_t sd_get_file_size(void);
int sd_file_read_line(void);
int sd_read_fixed_bytes(uint8_t *out_data, size_t out_size);
int sd_file_write(uint16_t adc_raw, float voltage);
int sd_card_unmount(void);
void list_files_on_sd(void);
int sd_file_close(void);
int sd_safe_close(void);
#endif // SD_CARD_H
