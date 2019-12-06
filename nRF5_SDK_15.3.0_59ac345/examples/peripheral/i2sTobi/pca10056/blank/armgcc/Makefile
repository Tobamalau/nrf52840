PROJECT_NAME     := i2s_pca10056
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := $(HOME)/src/nrf52840-DK/nRF5_SDK_15.3.0_59ac345
TOP_ROOT := $(HOME)/src/nrf52840-DK
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := i2s_gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_uart.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_uart.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_i2s.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \
  $(SDK_ROOT)/components/libraries/mem_manager/mem_manager.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_twi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twi.c \
  $(PROJ_DIR)/main.c \
  $(TOP_ROOT)/src/opus/opusTobi.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52840.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/components/libraries/uart/app_uart_fifo.c \
  $(SDK_ROOT)/components/libraries/uart/retarget.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_core.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_core_hooks.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_critical_section.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_debug.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_pib.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_rssi.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_rx_buffer.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_timer_coord.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/nrf_802154_csma_ca.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/nrf_802154_delayed_trx.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/nrf_802154_filter.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/nrf_802154_frame_parser.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/nrf_802154_precise_ack_timeout.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/platform/hp_timer/nrf_802154_hp_timer.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/rsch/nrf_802154_rsch.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/rsch/nrf_802154_rsch_crit_sect.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/timer_scheduler/nrf_802154_timer_sched.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_notification_swi.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_priority_drop_swi.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_request_swi.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_swi.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/platform/lp_timer/nrf_802154_lp_timer_nodrv.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/rsch/raal/single_phy/single_phy.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/platform/clock/nrf_802154_clock_nodrv.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/ack_generator/nrf_802154_ack_data.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/ack_generator/nrf_802154_ack_generator.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/ack_generator/nrf_802154_enh_ack_generator.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features/ack_generator/nrf_802154_imm_ack_generator.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/platform/temperature/nrf_802154_temperature_none.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/platform/coex/nrf_802154_wifi_coex_none.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/platform/random/nrf_802154_random_stdlib.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/fal/nrf_802154_fal.c \

#  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twim.c 
#  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_revision.c \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/nrf_802154_ack_pending_bit.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(PROJ_DIR) \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/libraries/util \
  ../config \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/components/libraries/atomic \
  $(TOP_ROOT)/src/opus/include/opus \
  $(SDK_ROOT)/components/libraries/uart \
  $(SDK_ROOT)/components/libraries/fifo \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/rsch/raal \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/mac_features \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src/rsch \
  $(TOP_ROOT)/nRF-IEEE-802.15.4-radio-driver/src \



# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O0 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52840_XXAA
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs
LDFLAGS += -L$(SDK_ROOT)/external/opus/lib


nrf52840_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lopus -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52840_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52840_xxaa
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := ../config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
