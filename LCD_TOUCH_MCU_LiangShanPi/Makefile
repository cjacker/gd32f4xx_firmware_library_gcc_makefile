######################################
# target
######################################
TARGET = gd32f470zgt6


######################################
# building variables
######################################
# debug build?
DEBUG = 0 
# optimization for size
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
../Firmware/CMSIS/GD/GD32F4xx/Source/system_gd32f4xx.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_gpio.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_timer.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_rcu.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_usart.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_spi.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_adc.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_exti.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_misc.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_crc.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_dac.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_tli.c \
../Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_exmc.c \
Hardware/key/key.c \
Hardware/LED/led.c \
Hardware/LCD/lcd.c \
Hardware/LCD/pic_480800.c \
Hardware/touch/touch.c \
Hardware/usart0/usart0.c \
User/gd32f4xx_it.c \
User/systick.c \
User/main.c


#Hardware/LCD/pic_480800.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_can.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_ctc.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_dbg.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_dci.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_dma.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_enet.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_fmc.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_fwdgt.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_i2c.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_ipa.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_iref.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_pmu.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_rtc.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_sdio.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_syscfg.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_trng.c \
#Firmware/GD32F4xx_standard_peripheral/Source/gd32f4xx_wwdgt.c \

#Hardware/RGB/spiInit.c \

# ASM sources
ASM_SOURCES = ../Firmware/CMSIS/GD/GD32F4xx/Source/GCC/startup_gd32f450_470.S


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_STDPERIPH_DRIVER \
-DGD32F470


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-I../Firmware/GD32F4xx_standard_peripheral/Include \
-I../Firmware/CMSIS/Include \
-I../Firmware/CMSIS/GD/GD32F4xx/Include/ \
-I../Firmware/CMSIS \
-IHardware/usart0 \
-IHardware/LCD \
-IHardware/touch \
-IHardware/LED \
-IHardware/SDRAM \
-IHardware/key \
-IHardware/RGB \
-IUser

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = ../Firmware/Ld/Link.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -u_printf_float -specs=nano.specs -flto -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# program
#######################################
program:
	openocd -f /usr/share/openocd/scripts/interface/cmsis-dap.cfg -f /usr/share/openocd/scripts/target/stm32f4x.cfg -c "program build/$(TARGET).elf verify reset exit"

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
