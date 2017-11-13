##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [2.23.0] date: [Wed Jul 05 20:44:53 EEST 2017]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = FATFS


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# source path
SOURCES_DIR =  \
Middlewares/FatFs \
Application/User \
Application/MAKEFILE \
Middlewares \
Application \
dev_driver

# JSON action file path
DEVICE_ACTION_FILE_NAME=\"device_action.json\"
# virtual device memory file path
DEVICE_FILE_NAME_XXD ="device.dat"
DEVICE_FILE_NAME=\"device.dat\"
# virtual device memory hex file path
DEVICE_HEX_FILE_NAME="device.hex"

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Middlewares/Third_Party/FatFs/src/option/syscall.c \
Src/fatfs.c \
Src/user_diskio.c \
Middlewares/Third_Party/FatFs/src/ff.c \
Middlewares/Third_Party/FatFs/src/diskio.c \
Src/main.c \
Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
dev_driver/dev_io.c \
dev_driver/dev_action.c

# ASM sources
# ASM_SOURCES =  \
# startup_stm32f407xx.s


######################################
# firmware library
######################################
PERIFLIB_SOURCES = 


#######################################
# binaries
#######################################
BINPATH = 
PREFIX =
CC = $(BINPATH)$(PREFIX)gcc
AS = $(BINPATH)$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)$(PREFIX)objcopy
AR = $(BINPATH)$(PREFIX)ar
SZ = $(BINPATH)$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-IMiddlewares/Third_Party/FatFs/src \
-Idev_driver \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

# CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -rdynamic

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

#Defines
DEFINES_FLAGS = -DDEVICE_ACTION_FILE_NAME=$(DEVICE_ACTION_FILE_NAME) -DDEVICE_FILE_NAME=$(DEVICE_FILE_NAME)

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" $(DEFINES_FLAGS)



#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407VGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys -lunwind
LIBDIR =
# -specs=nano.specs
# LDFLAGS = $(MCU)  -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cefref -Wl,--gc-sections
LDFLAGS =  -L/usr/local/lib -lunwind
# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
# OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
# vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
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
# clean up
#######################################
clean: clean_data
	-rm -fR .dep $(BUILD_DIR)

clean_data:
	rm *.json *.dat *.hex

#######################################
# execute
#######################################
run: $(BUILD_DIR)/$(TARGET).elf
	./$(BUILD_DIR)/$(TARGET).elf
	xxd -b $(DEVICE_FILE_NAME_XXD) > $(DEVICE_HEX_FILE_NAME)

#######################################
# debug
#######################################
debug: $(BUILD_DIR)/$(TARGET).elf
	ddd $(BUILD_DIR)/$(TARGET).elf &
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***