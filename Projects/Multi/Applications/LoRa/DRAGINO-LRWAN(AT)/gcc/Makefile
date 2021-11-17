# GNU Makefile for building DRAGINO-LRWAN (AT) firmware with GCC ARM toolchain
#
# Copyright 2021 Tormod Volden

NAME = lgt92

REGION = EU868

BUILDDIR = build

DRIVERS_BSP_COMPONENTS = \
        flash_eraseprogram.c \
        iwdg.c \
        sx1276.c \
        gpio_exti.c \
        filter.c \
        IIC.c \
        gps.c \
        mpu9250.c \
        bsp_usart2.c
DRIVERS_BSP_STM32L0xx_NUCLEO = \
        stm32l0xx_nucleo.c
DRIVERS_BSP_SX1276MB1LAS = \
        sx1276mb1las.c
DRIVERS_CMSIS = \
        system_stm32l0xx.c
DRIVERS_STM32L0xx_HAL_DRIVER = \
        stm32l0xx_hal_spi.c \
        stm32l0xx_hal_uart.c \
        stm32l0xx_hal_adc.c \
        stm32l0xx_hal_rcc.c \
        stm32l0xx_hal_rcc_ex.c \
        stm32l0xx_hal.c \
        stm32l0xx_hal_cortex.c \
        stm32l0xx_hal_gpio.c \
        stm32l0xx_hal_dma.c \
        stm32l0xx_hal_pwr.c \
        stm32l0xx_hal_pwr_ex.c \
        stm32l0xx_hal_rtc.c \
        stm32l0xx_hal_rtc_ex.c \
        stm32l0xx_hal_tim.c \
        stm32l0xx_hal_tim_ex.c \
        stm32l0xx_hal_adc_ex.c \
        stm32l0xx_hal_uart_ex.c \
        stm32l0xx_hal_i2c.c \
        stm32l0xx_hal_i2c_ex.c \
        stm32l0xx_hal_flash_ex.c \
        stm32l0xx_hal_flash.c \
        stm32l0xx_hal_iwdg.c
MIDDLEWARES_LORA_MAC_REGION = \
        Region.c \
        RegionAS923.c \
        RegionAU915.c \
        RegionCN470.c \
        RegionCN779.c \
        RegionCommon.c \
        RegionEU433.c \
        RegionEU868.c \
        RegionIN865.c \
        RegionKR920.c \
        RegionUS915.c \
        RegionRU864.c \
        RegionKZ865.c \
        LoRaMac.c \
        LoRaMacCrypto.c \
        lora-test.c
MIDDLEWARES_LORA_UTILITIES = \
        delay.c \
        low_power_manager.c \
        timeServer.c \
        utilities.c \
        trace.c \
        queue.c
MIDDLEWARES_LORA_CRYPTO = \
        aes.c \
        cmac.c
PROJECTS_END_NODE = \
        bsp.c \
        debug.c \
        hw_gpio.c \
        hw_rtc.c \
        hw_spi.c \
        main.c \
        stm32l0xx_hal_msp.c \
        stm32l0xx_hw.c \
        stm32l0xx_it.c \
        vcom.c \
        at.c \
        command.c \
        lora.c \
        tiny_sscanf.c \
        tiny_vsnprintf.c
PROJECTS_STARTUP = \
        startup_stm32l072xx.s

CSRC := \
        $(DRIVERS_BSP_COMPONENTS) \
        $(DRIVERS_BSP_STM32L0xx_NUCLEO) \
        $(DRIVERS_BSP_SX1276MB1LAS) \
        $(DRIVERS_CMSIS) \
        $(DRIVERS_STM32L0xx_HAL_DRIVER) \
        $(MIDDLEWARES_LORA_MAC_REGION) \
        $(MIDDLEWARES_LORA_UTILITIES) \
        $(MIDDLEWARES_LORA_CRYPTO) \
        $(PROJECTS_END_NODE)

ASRC := \
        $(PROJECTS_STARTUP)

OBJS := $(CSRC:%=$(BUILDDIR)/%.o) $(ASRC:%=$(BUILDDIR)/%.o)

INCDIRS = \
Drivers/BSP/STM32L0xx_Nucleo \
Drivers/STM32L0xx_HAL_Driver/Inc \
Drivers/CMSIS/Device/ST/STM32L0xx/Include \
Drivers/CMSIS/Include \
Drivers/BSP/Components/Common \
Drivers/BSP/Components/sx1276 \
Drivers/BSP/sx1276mb1las \
Drivers/BSP/Components/flash_eraseprogram \
Drivers/BSP/Components/gpio_exti \
Drivers/BSP/Components/iwdg \
Drivers/BSP/Components/filter \
Drivers/BSP/Components/gps \
Drivers/BSP/Components/mpu9250 \
Drivers/BSP/Components/IIC \
Drivers/BSP/Components/usart \
Middlewares/Third_Party/Lora/Crypto \
Middlewares/Third_Party/Lora/Mac \
Middlewares/Third_Party/Lora/Phy \
Middlewares/Third_Party/Lora/Utilities \
Middlewares/Third_Party/Lora/Core

CSRCDIRS = \
        Drivers/CMSIS/Device/ST/STM32L0xx/Source/Templates \
        Drivers/STM32L0xx_HAL_Driver/Src \
        Middlewares/Third_Party/Lora/Mac/region \
        $(INCDIRS)

ASRCDIRS = \
        Drivers/CMSIS/Device/ST/STM32L0xx/Source/Templates/gcc

vpath %.c ../src $(addprefix ../../../../../../,$(CSRCDIRS))
vpath %.s $(addprefix ../../../../../../,$(ASRCDIRS))
INCFLAGS = -I../inc $(patsubst %,-I../../../../../../%,$(INCDIRS))

# Toolchain setup
CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld -v
AR = $(CROSS_COMPILE)ar
AS = $(CROSS_COMPILE)as
CP = $(CROSS_COMPILE)objcopy
OD = $(CROSS_COMPILE)objdump
SZ = $(CROSS_COMPILE)size

ARCHFLAGS += -mcpu=cortex-m0plus -mthumb
ASFLAGS += $(ARCHFLAGS)
CFLAGS += $(ARCHFLAGS) -Os
LDFLAGS += $(ARCHFLAGS)

CPPFLAGS += $(INCFLAGS)

CFLAGS += -ffunction-sections
LDFLAGS += -Wl,--gc-sections

LDFLAGS += -T gcc-stm32l072cz.ld
LDLIBS += -specs=nano.specs -specs=nosys.specs -lm

CFLAGS += -DSTM32L072xx -DUSE_STM32L0XX_NUCLEO -DUSE_HAL_DRIVER -DUSE_SHT -DREGION_$(REGION)


all: $(BUILDDIR)/$(NAME).hex

$(BUILDDIR)/$(NAME).hex: $(BUILDDIR)/$(NAME).elf
	$(CP) -O ihex $^ $@

$(BUILDDIR)/$(NAME).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILDDIR)/%.c.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILDDIR)/%.s.o: %.s
	$(AS) $(ASFLAGS) -c $< -o $@

$(OBJS): | $(BUILDDIR)
$(BUILDDIR):
	mkdir $@

size: $(BUILDDIR)/$(NAME).elf
	$(SZ) --format=SysV --radix=16 $^ | egrep "^.text|^.data|^.bss"

OPENOCD = openocd -f interface/stlink-v2.cfg -c "transport select hla_swd" \
           -f target/stm32l0.cfg

flash: $(BUILDDIR)/$(NAME).hex
	$(OPENOCD) -c "program $< verify reset exit"

clean:
	rm -f $(OBJS) $(BUILDDIR)/$(NAME).hex $(BUILDDIR)/$(NAME).elf
