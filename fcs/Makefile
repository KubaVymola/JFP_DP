DEBUG = 1
OPT = 

BUILD_DIR = build

AUTOARM =
TARGET = 
FAMILY = 
VENDOR =
OUTPUT_TYPE =

# ==== TARGET SETTINGS ====
# Target list:
#   - STM32F405

ifeq ($(TARGET), STM32F405)
FAMILY = STM32F4
VENDOR = ST
OUTPUT_TYPE = MCU
endif

ifeq ($(TARGET),)
OUTPUT_TYPE = CPU
endif

# ==== END TARGET SETTINGS ====

.PHONY: clean

C_SOURCES = 

CXX_SOURCES = \
src/Adafruit_AHRS_Madgwick.cpp \
src/demo_sequence.cpp \
src/flash_logging.cpp \
src/flight_events.cpp \
src/j_packet_recv_callback.cpp \
src/j_packet_send_callback.cpp \
src/main.cpp \
src/mixer.cpp \
src/pid.cpp \
src/sitl_data_interface.cpp \
src/sitl_tunning.cpp \
src/user_commands.cpp \
src/utils.cpp \
../utils/j_packets.cpp

ASM_SOURCES = 

# ==== BINARIES ====
PREFIX = 

CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
CXX = $(PREFIX)g++
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
# ==== END BINARIES ====

# ==== FLAGS ====
CPU = 
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

INCLUDES = -Isrc -Isrc/drivers -I../utils
DEFS = 

AS_DEFS = $(DEFS)
C_DEFS = $(DEFS)
CXX_DEFS = $(DEFS)

ASFLAGS = $(AS_DEFS) $(INCLUDES) $(OPT) -Wall
CFLAGS = $(C_DEFS) $(INCLUDES) $(OPT) -Wall
CXXFLAGS = $(CXX_DEFS) $(INCLUDES) $(OPT) -Wall -std=c++17

ifeq ($(DEBUG),1)
CFLAGS += -g -gdwarf-2
CXXFLAGS += -g -gdwarf-2
endif

# ==== END FLAGS ====


# ==== LD FLAGS ====

LIBS = -lc -lm
LDFLAGS = $(LIBS)

# ==== END LD FLAGS ====

####################################################################################################
# OUTPUT TYPE SETTINGS
####################################################################################################
ifeq ($(OUTPUT_TYPE),MCU)
PREFIX = arm-none-eabi-
ASFLAGS += $(MCU) -fdata-sections -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += $(MCU) -fdata-sections -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst))
CXXFLAGS += $(MCU) -fdata-sections -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst))
LIBS += -lnosys
OPT = -Og
LDFLAGS = $(MCU) -specs=nosys.specs -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
DEFS += -DMCU
endif

ifeq ($(OUTPUT_TYPE),CPU)
DEFS += -DCPU
endif

####################################################################################################
# VENDOR SETTINGS
####################################################################################################
ifeq ($(VENDOR),ST)
CXX_SOURCES += \
src/drivers/bmi160.cpp \
src/drivers/hp203b.cpp \
src/usb/usbd_cdc_if.cpp
C_SOURCES += \
src/drivers/w25qxx.c \
src/usb/usbd_conf.c \
src/usb/usb_device.c \
src/usb/usbd_desc.c \
src/usb/usbd_core.c \
src/usb/usbd_ctlreq.c \
src/usb/usbd_ioreq.c \
src/usb/usbd_cdc.c
INCLUDES += -Isrc/hal/st -Isrc/hal/st/cmsis -Isrc/usb
DEFS += -DUSE_HAL_DRIVER -DVENDOR_ST
LDFLAGS += -u _printf_float
endif



####################################################################################################
# FAMILY SETTINGS
####################################################################################################

ifeq ($(FAMILY),STM32F4)
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
C_SOURCES += \
src/hal/st/stm32f4/config.c \
src/hal/st/stm32f4/gpio.c \
src/hal/st/stm32f4/spi.c \
src/hal/st/stm32f4/tim.c \
src/hal/st/stm32f4/adc.c \
src/hal/st/stm32f4/i2c.c \
src/hal/st/stm32f4/usart.c \
src/hal/st/stm32f4/core/stm32f4xx_it.c \
src/hal/st/stm32f4/core/stm32f4xx_hal_msp.c \
src/hal/st/stm32f4/core/system_stm32f4xx.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_pcd.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_pcd_ex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_ll_usb.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_rcc.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_rcc_ex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_flash.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_flash_ex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_flash_ramfunc.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_gpio.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_dma_ex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_dma.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_pwr.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_pwr_ex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_cortex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_exti.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_spi.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_tim.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_tim_ex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_adc.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_adc_ex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_ll_adc.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_i2c.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_i2c_ex.c \
src/hal/st/stm32f4/hal_driver/stm32f4xx_hal_uart.c
INCLUDES += \
-Isrc/hal/st/stm32f4 \
-Isrc/hal/st/stm32f4/core \
-Isrc/hal/st/stm32f4/cmsis \
-Isrc/hal/st/stm32f4/hal_driver \
-Isrc/hal/st/stm32f4/hal_driver/Legacy
DEFS += -DSTM32F4
endif

####################################################################################################
# TARGET SETTINGS
####################################################################################################

ifeq ($(TARGET),STM32F405)
ASM_SOURCES += src/hal/st/stm32f4/stm32f405/gcc/startup_stm32f405xx.s
DEFS += -DSTM32F405xx
LDSCRIPT = src/hal/st/stm32f4/stm32f405/gcc/STM32F405RGTx_FLASH.ld
endif

####################################################################################################
# AUTOARM
####################################################################################################
ifeq ($(AUTOARM),1)
DEFS += -DAUTOARM
endif

# all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
all: sitl



# ==== BUILD TARGETS ====

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))




$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

# ==== END BUILD TARGETS ====

clean:
	-rm -fR $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)



####################################################################################################
# TARGETS
#
# sitl target defines -DSITL. TARGET from CLI should not be set -> that will also define -DCPU
# htil target defines -DHITL. TARGET should be set in the CLI -> that will define -DMCU
# indep target defines -DINDEP. TARGET should be set in the CLI -> that will define -DMCU
#
# flash target should always follow either hitl, or indep target
#
# if changing build type (e.g. from sitl to hitl, or from hitl to indep) always run clean target first
####################################################################################################

# ==== SITL ====
sitl: DEFS += -DSITL
sitl: $(OBJECTS)
	$(CXX) -shared -o $(BUILD_DIR)/libfcs.so $(CXXFLAGS) $(OBJECTS)

# ==== HITL ====
hitl: DEFS += -DHITL
hitl: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# ==== NORMAL MCU FW ====
indep: DEFS += -DINDEP
indep: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# ==== UPLOAD TO MCU ====
flash: $(BUILD_DIR)/$(TARGET).bin
	openocd -f debug.cfg -c "program $(BUILD_DIR)/$(TARGET).bin 0x8000000 reset exit"
