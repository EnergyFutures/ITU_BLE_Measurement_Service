# List all source files the application uses.
APPLICATION_SRCS = $(notdir $(wildcard ../*.c))
APPLICATION_SRCS += softdevice_handler.c
APPLICATION_SRCS += ble_advdata.c
APPLICATION_SRCS += ble_debug_assert_handler.c
APPLICATION_SRCS += ble_srv_common.c
APPLICATION_SRCS += ble_error_log.c
APPLICATION_SRCS += pstorage.c
APPLICATION_SRCS += crc16.c
APPLICATION_SRCS += app_scheduler.c
APPLICATION_SRCS += app_timer.c
APPLICATION_SRCS += ble_conn_params.c
APPLICATION_SRCS += app_button.c
APPLICATION_SRCS += app_gpiote.c
APPLICATION_SRCS += nrf_delay.c








PROJECT_NAME = $(shell basename "$(realpath ../)") #get project name from folder name

DEVICE = NRF51
BOARD = BOARD_PCA10001
#SEGGER_SERIAL =

#SDK_VERSION = 7 #default is version 6
USE_SOFTDEVICE = s110

SDK_PATH = $(HOME)/Dropbox/12-coding/nRF51/nrf51-mac-setup/sdk/nrf51_sdk_v6_1_0/nrf51822/
TEMPLATE_PATH = $(HOME)/Dropbox/12-coding/nRF51/nrf51-pure-gcc-setup/template/

CFLAGS = -Os #no compiler optimization for debugging

GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile
