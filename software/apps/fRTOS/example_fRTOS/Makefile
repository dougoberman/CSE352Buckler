# nRF application makefile
PROJECT_NAME = $(shell basename "$(realpath ./)")

# Configurations
NRF_IC = nrf52832
SDK_VERSION = 15
SOFTDEVICE_MODEL = blank

# Source and header files
APP_HEADER_PATHS += .
APP_HEADER_PATHS += ../include
APP_HEADER_PATHS += ../portable/GCC/nrf52/
APP_SOURCE_PATHS += .
APP_SOURCE_PATHS += ../Source
APP_SOURCE_PATHS += ../portable/GCC/nrf52/
APP_SOURCE_PATHS += ../MemMang

# APP_SOURCES = $(notdir $(wildcard ./*.c))

APP_SOURCES = main.c
APP_SOURCES += ../Source/tasks.c
#APP_SOURCES += ./Source/supporting_functions.c
APP_SOURCES += ../Source/list.c
APP_SOURCES += ../Source/queue.c
APP_SOURCES += ../Source/timers.c
APP_SOURCES += ../portable/GCC/nrf52/port.c
APP_SOURCES += ../portable/GCC/nrf52/port_cmsis.c
APP_SOURCES += ../portable/GCC/nrf52/port_cmsis_systick.c
APP_SOURCES += ../MemMang/heap_2.c



$(info $$APP_SOURCES is [${APP_SOURCES}])

# APP_SOURCES += ./MemMang/heap_5.c
# APP_SOURCES += ./source/tasks.c

# Path to FreeRTOSs
#FreeRTOS_BASE_DIR = freertos/source/include/*.h

#include $(FreeRTOS_BASE_DIR)

# Path to base of nRF52-base repo
NRF_BASE_DIR = ../../../nrf52x-base/

# Include board Makefile (if any)
include ../../../boards/buckler_revC/Board.mk

# Include main Makefile
include $(NRF_BASE_DIR)make/AppMakefile.mk


