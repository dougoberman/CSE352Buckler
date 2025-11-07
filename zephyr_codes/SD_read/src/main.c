/*
Update:

Made solely for reading operations of the SD card
*/

// Consolidating the SD card read and write operations here. 
/*
SD card library broughht in fomr the project of microophone reading datainto the SD card with the help of a FIFO instance.

SDcard.c
SDcard.h

*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>
#include <zephyr/logging/log.h>
#include "SD_card.h"
#define SPI_DEVICE_NODE DT_NODELABEL(spi1)
// microphone functions proptotypes

int init_recording();
int start_recording();
int stop_recording();
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

//////////////////
K_FIFO_DEFINE(i2s_block_fifo);
bool opened_file = false;
bool mounted_card = false;

bool recoding_init = false;

LOG_MODULE_REGISTER(Main, LOG_LEVEL_INF);

// Define memory structures

// BLOCK_COUNT and BLOCK_SIZE are defiend inside the SD_card.h file. 
// K_MEM_SLAB_DEFINE_STATIC(mem_slab, sizeof(audio_block_t), BLOCK_COUNT, 4);
K_SEM_DEFINE(enable_recording, 0, 1);
K_SEM_DEFINE(data_ready, 0, 1);

// static const struct device *mic;
static bool recording_active = false;

/// All Buttons in the System

#define BUTTON_1_NODE DT_ALIAS(sw0)


static const struct gpio_dt_spec button_1 = GPIO_DT_SPEC_GET_OR(BUTTON_1_NODE, gpios, {0});


// Callback structures for each button
static struct gpio_callback button_cb_data_1;

// GPIO Configuration for Button and LED
#define LED0_NODE DT_ALIAS(led0)


static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});

// Button Press Callback
void button_pressed_1(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (pins & BIT(button_1.pin)) {
        
            
            if (recording_active) {

                LOG_INF("Stopping the Recording!! \n");
                
                stop_recording();
            } else {
                LOG_INF("Starting the Recording! \n");
                
                k_sem_give(&enable_recording);
                k_sem_give(&data_ready);


            }
    }
}

    
// Initialize Button and LED


int init_button_led() {
    int ret;
    //////// Button 1 /////////////
    if (!device_is_ready(button_1.port)) {
        LOG_ERR("Button device not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&button_1, GPIO_INPUT);
    if (ret != 0) return ret;

    ret = gpio_pin_interrupt_configure_dt(&button_1, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) return ret;

    gpio_init_callback(&button_cb_data_1, button_pressed_1, BIT(button_1.pin)); //| BIT(button_2.pin)
                    // | BIT(button_3.pin) | BIT(button_4.pin));
    ret = gpio_add_callback(button_1.port, &button_cb_data_1);
    if (ret != 0) {
        LOG_ERR("Error %d: Failed to add callback on %s pin %d", ret, button_1.port->name, button_1.pin);
        return ret;
    }

    /////////////////// LED ///////////////////////
    ret = device_is_ready(led.port);
    if (!ret) {
        LOG_ERR("Error: LED device %s is not ready", led.port->name);
        return ret;
    }

        ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
        LOG_ERR("Error %d: Failed to configure LED pin", ret);
        return ret;
    }
    gpio_pin_set(led.port, led.pin, 0);

    LOG_INF("Set up LED on %s pin %d", led.port->name, led.pin);
    LOG_DBG("Set up LED on %s pin %d", led.port->name, led.pin);
    // LOG_DBG("Set up button at %s pin %d", button_1.port->name, button_1.pin);
        return 0;
}

/* SD Card Reading Operations*/

// once initiatied, the SD card is repeatedly read from and the data read is displayed on the UART terminal

void sd_reading(void) {
        LOG_INF("SD Card Reading Function\n");
    
        k_sem_take(&data_ready, K_FOREVER);
        if (sd_file_open() != 0) {
            LOG_ERR("Failed to open file\n");
            return;
        } else {
            opened_file = true;
            LOG_INF("File Opened Successfully\n");
        }
        // List all the files in the SD card

        
        while (1) {
            LOG_INF("Reading from SD card...\n");
            // Call the function to read from the SD card
            if (sd_file_read_line() != 0) {
                LOG_ERR("Failed to read from SD card");
                break;
            }            
            k_sleep(K_MSEC(10)); // Sleep for 10 milli second before next read
        }        
}

// application function to read the data in successive bytes

 // Function to read bytes from SD card and print to UART
#define SD_DATA_READ_BUFFER_SIZE 100
void sd_read_bytes_and_print(void) {
    uint8_t buffer[SD_DATA_READ_BUFFER_SIZE];
    size_t bytes_read = 0;
    size_t total_bytes = sd_get_file_size(); // Assume this function returns the file size

    if (sd_file_open() != 0) {
        LOG_ERR("Failed to open file for reading");
        return;
    }

    while (bytes_read < total_bytes) {
        
        // separate function to read bytes from the SD card
        /////////////////////////////////////////////////////
        // 1. fixed number of predefined bytes -- v
        // 2. dynamic number of bytes (availability basis)
        int ret = sd_read_fixed_bytes(buffer, SD_DATA_READ_BUFFER_SIZE);
        if (ret <= 0) {
            LOG_ERR("Error reading bytes from SD card.");
            break;
        }
        bytes_read += ret;
        /////////////////////////////////////////////////////
        /////////////////////////////////////////////////////
        // // Print bytes to UART
        // for (int i = 0; i < ret; i++) {
        //     printk("%02X ", buffer[i]);
        // }
        // printk("\n");
        // printk("Read %zd bytes: \n", ret);
    }

    sd_file_close();
    LOG_INF("Finished reading SD card file.");    
}


// Main Function
int main(void) {
        LOG_INF("Starting main thread");
        
        // LOG_INF("Assuming a data file with binary data in the SD card, the program will be reading from it and displaying the data on ther UART terminal \n");

        if (init_button_led() != 0) {
            LOG_ERR("Failed to initialize button");
            return -1;
        }

        // initialize the SPI and SD card           

        if (spi_init() != 0) {
            LOG_ERR("Failed to initialize SPI");
            return -1;
        }        
        
        // List all the files in the SD card
        k_sem_give(&data_ready);

        list_files_on_sd();
        
        // to read and print all the data in the SD card
        // sd_reading();

        // to read the data in bytes and print it on the UART terminal
        sd_read_bytes_and_print();

       
        while (1) {
            k_sleep(K_FOREVER);
        }
        return 0;
    }
    