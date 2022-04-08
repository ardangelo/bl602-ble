# Project configuration
PROJECT_NAME := ble-client
PROJECT_PATH := $(abspath .)
PROJECT_BOARD := evb
export CONFIG_CHIP_NAME=BL602
export PROJECT_PATH PROJECT_BOARD

-include ./proj_config.mk

# Check API path
ifeq ($(origin BL60X_SDK_PATH), undefined)
$(error ****** Please SET BL60X_SDK_PATH ******)
endif

# Disable wifi (still need includes for BT)
CFLAGS := $(CFLAGS) -DFEATURE_WIFI_DISABLE=1

# BL602 system
INCLUDE_COMPONENTS += bl602 bl602_std bltime blfdt blmtd bloop loopadc looprt loopset
# FreeRTOS
INCLUDE_COMPONENTS += freertos_riscv_ram
# HAL
INCLUDE_COMPONENTS += hal_drv utils cli
# VFS
INCLUDE_COMPONENTS += vfs romfs
# AliOS Things
INCLUDE_COMPONENTS += yloop
# Bluetooth (bl602_wifi required for BT device)
INCLUDE_COMPONENTS += blecontroller blestack bl602_wifi blog

# Project
INCLUDE_COMPONENTS += $(PROJECT_NAME)

include $(BL60X_SDK_PATH)/make_scripts_riscv/project.mk
