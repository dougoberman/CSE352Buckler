#include <stdint.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"
#include "buckler.h"
#include "simple_logger.h"
#include "app_error.h"
#include "gpio.h"

void timer_init();

float get_timestamp();

void init_SDCard();

uint32_t read_timer();