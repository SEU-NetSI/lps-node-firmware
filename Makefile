OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2.cfg
OPENOCD_CMDS      ?=
REV               ?= B
PYTHON           ?= python3
# CFLAGS          += -fdiagnostics-color=auto
# CFLAGS += -DUSE_FTDI_UART

BOOTLOAD          ?= 0

ifeq ($(strip $(REV)),A)
$(error Rev.A not supported anymore)
else ifeq ($(strip $(REV)),B)
HAL_ROOT=hal/stm32f0xx
CPU=f0
PROCESSOR=-mthumb -mcpu=cortex-m0 -DHSI48_VALUE="((uint32_t)48000000)" -DSTM32F072xB
OPENOCD_TARGET    ?= target/stm32f0x_stlink.cfg
else
$(error Rev.$(REV) unknown)
endif

INCLUDES=-Iinc -Iinc/$(CPU) -I$(HAL_ROOT)/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc

# FreeRTOS
FREERTOS_OBJS=list queue timers tasks port event_groups
OBJS+=$(foreach mod, $(FREERTOS_OBJS), lib/freertos/src/$(mod).o)
INCLUDES+=-Ilib/freertos/inc

# Platform specific files
OBJS+=src/f0/startup_stm32f072xb.o src/f0/system_stm32f0xx.o src/f0/stm32f0xx_it.o src/f0/stm32f0xx_hal_msp.o
OBJS+=src/f0/gpio.o src/f0/i2c.o src/f0/spi.o src/f0/system.o src/f0/usart.o
OBJS+=src/f0/usbd_conf.o src/eeprom.o src/bootmode.o
HALS+=i2c_ex

OBJS+=src/main.o
OBJS+=src/usb_device.o src/usbd_cdc_if.o src/usbd_desc.o src/lps25h.o src/led.o src/button.o
OBJS+=src/cfg.o src/usbcomm.o src/test_support.o src/production_test.o
OBJS+=src/uwb.o # src/uwb_twr_anchor.o src/uwb_sniffer.o src/uwb_twr_tag.o
# OBJS+=src/lpp.o src/uwb_tdoa_anchor2.o src/uwb_tdoa_anchor3.o

HALS+=gpio rcc cortex i2c pcd dma pcd_ex rcc_ex spi uart pwr
OBJS+=$(foreach mod, $(HALS), $(HAL_ROOT)/Src/stm32$(CPU)xx_hal_$(mod).o)
OBJS+=$(HAL_ROOT)/Src/stm32$(CPU)xx_hal.o

USB_CORES=core ctlreq ioreq
USB_CDC=cdc
OBJS+=$(foreach mod, $(USB_CORES), Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_$(mod).o)
OBJS+=$(foreach mod, $(USB_CDC), Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_$(mod).o)

#libdw1000
# INCLUDES+=-Ivendor/libdw1000/inc
# OBJS+=vendor/libdw1000/src/libdw1000.o vendor/libdw1000/src/libdw1000Spi.o

OBJS+=src/dwOps.o

#libdw3000
INCLUDES+=-Isrc/decadriver -Isrc/platform -Iconfig_options.h
OBJS+=src/decadriver/deca_device.o \
src/platform/deca_mutex.o \
src/platform/deca_sleep.o \
src/platform/deca_spi.o \
src/platform/port.o \
src/config_options.o

#libdw3000 examples
# INCLUDES+=-Isrc/examples/examples_info/ \
# -Isrc/examples/shared_data/  \
# -Isrc

# OBJS+=src/examples/ex_00a_reading_dev_id/read_dev_id.o \
# src/examples/ex_01a_simple_tx/simple_tx.o \
# src/examples/ex_01b_tx_sleep/tx_sleep.o \
# src/examples/ex_01b_tx_sleep/tx_sleep_idleRC.o \
# src/examples/ex_01c_tx_sleep_auto/tx_sleep_auto.o \
# src/examples/ex_01d_tx_timed_sleep/tx_timed_sleep.o \
# src/examples/ex_01e_tx_with_cca/tx_with_cca.o \
# src/examples/ex_01g_simple_tx_sts_sdc/simple_tx_sts_sdc.o \
# src/examples/ex_01h_simple_tx_pdoa/simple_tx_pdoa.o \
# src/examples/ex_01i_simple_tx_aes/simple_tx_aes.o \
# src/examples/ex_02a_simple_rx/simple_rx.o \
# src/examples/ex_02c_rx_diagnostics/rx_diagnostics.o \
# src/examples/ex_02d_rx_sniff/rx_sniff.o \
# src/examples/ex_02f_rx_with_crystal_trim/rx_with_xtal_trim.o \
# src/examples/ex_02g_simple_rx_sts_sdc/simple_rx_sts_sdc.o \
# src/examples/ex_02h_simple_rx_pdoa/simple_rx_pdoa.o \
# src/examples/ex_02i_simple_rx_aes/simple_rx_aes.o \
# src/examples/ex_03a_tx_wait_resp/tx_wait_resp.o \
# src/examples/ex_03b_rx_send_resp/rx_send_resp.o \
# src/examples/ex_03d_tx_wait_resp_interrupts/tx_wait_resp_int.o \
# src/examples/ex_04a_cont_wave/continuous_wave.o \
# src/examples/ex_04b_cont_frame/continuous_frame.o \
# src/examples/ex_05a_ds_twr_init/ds_twr_initiator.o \
# src/examples/ex_05a_ds_twr_init/ds_twr_initiator_sts.o \
# src/examples/ex_05b_ds_twr_resp/ds_twr_responder.o \
# src/examples/ex_05b_ds_twr_resp/ds_twr_responder_sts.o \
# src/examples/ex_05c_ds_twr_init_sts_sdc/ds_twr_sts_sdc_initiator.o \
# src/examples/ex_05d_ds_twr_resp_sts_sdc/ds_twr_sts_sdc_responder.o \
# src/examples/ex_06a_ss_twr_initiator/ss_twr_initiator.o \
# src/examples/ex_06a_ss_twr_initiator/ss_twr_initiator_sts.o \
# src/examples/ex_06a_ss_twr_initiator/ss_twr_initiator_sts_no_data.o \
# src/examples/ex_06b_ss_twr_responder/ss_twr_responder.o \
# src/examples/ex_06b_ss_twr_responder/ss_twr_responder_sts.o \
# src/examples/ex_06b_ss_twr_responder/ss_twr_responder_sts_no_data.o \
# src/examples/ex_06e_AES_ss_twr_initiator/ss_aes_twr_initiator.o \
# src/examples/ex_06f_AES_ss_twr_responder/ss_aes_twr_responder.o \
# src/examples/ex_07a_ack_data_tx/ack_data_tx.o \
# src/examples/ex_07b_ack_data_rx/ack_data_rx.o \
# src/examples/ex_07c_ack_data_rx_dbl_buff/ack_data_rx_dbl_buff.o \
# src/examples/ex_11a_spi_crc/spi_crc.o \
# src/examples/ex_13a_gpio/gpio_example.o \
# src/examples/ex_14_otp_write/otp_write.o \
# src/examples/ex_15_le_pend/le_pend_rx.o \
# src/examples/ex_15_le_pend/le_pend_tx.o \
# src/examples/examples_info/example_info.o \
# src/examples/shared_data/shared_functions.o \

# #libdw3000 examples mac802.15
# INCLUDES+=-Isrc/MAC_802_15_8/ -Isrc/MAC_802_15_4/
# OBJS+=src/MAC_802_15_8/mac_802_15_8.o src/MAC_802_15_4/mac_802_15_4.o


CFLAGS+=$(PROCESSOR) $(INCLUDES) -O3 -g3 -Wall -Wno-pointer-sign -std=gnu11
LDFLAGS+=$(PROCESSOR) --specs=nano.specs --specs=nosys.specs -lm -lc -u _printf_float

ifeq ($(strip $(BOOTLOAD)),0)
LDFLAGS+=-Ttools/make/stm32f072.ld
LOAD_ADDRESS = 0x8000000
else
LDFLAGS+=-Ttools/make/stm32f072_bootload.ld
LOAD_ADDRESS = 0x8005000
endif

# Remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS+=-Wl,-Map=bin/$(PROG).map,--cref,--gc-sections


PREFIX=arm-none-eabi-

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
AS=$(PREFIX)as
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size

all: check_submodules bin/lps-node-firmware.elf bin/lps-node-firmware.dfu

bin/lps-node-firmware.elf: $(OBJS)
	$(LD) -o $@ $^ $(LDFLAGS)
	$(SIZE) $@
	@echo BOOTLOADER Support: $(BOOTLOAD)

clean:
	rm -f bin/lps-node-firmware.elf bin/lps-node-firmware.dfu bin/.map $(OBJS)

flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
	           -c "flash write_image erase bin/lps-node-firmware.elf" -c "verify_image bin/lps-node-firmware.elf" -c "reset run" -c shutdown
erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
	           -c "stm32f1x mass_erase 0" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets

dfu:
	dfu-util -d 0483:df11 -a 0 -D bin/lps-node-firmware.dfu -s :leave

reset_and_dfu:
	tools/make/reset-to-dfu.py
	dfu-util -d 0483:df11 -a 0 -D bin/lps-node-firmware.dfu -s :leave

# Generic rules
%.bin: %.elf
	$(OBJCOPY) $^ -O binary $@

%.dfu: %.bin
	$(PYTHON) tools/make/dfu-convert.py -b $(LOAD_ADDRESS):$^ $@

check_submodules:
	$(PYTHON) tools/make/check-for-submodules.py
