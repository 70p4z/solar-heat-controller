
# ARMGNU ?= arm-none-eabi

# CFLAGS = -Wall -nostdlib -nostartfiles -ffreestanding 

DEFINES += VERSION=132
DEFINES += DEBUG

# Cortex-M4
#CPUFLAGS += -mcpu=cortex-m4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
# ARM7TDMI (LPC213x)
CPUFLAGS += -mthumb -mcpu=arm7tdmi-s -mtune=arm7tdmi-s 

all: default

ifeq ($(BUILD_DIR),)
BUILD_DIR = build
endif


INCLUDE_PATH += include/
SOURCE_PATH += src/ 

SOURCE_FILES = $(foreach dir,$(SOURCE_PATH),$(shell find $(dir) -name "*.[cs]") )
OBJECT_FILES = $(foreach file, $(SOURCE_FILES), $(BUILD_DIR)/$(basename $(notdir $(file))).o) 
#$(foreach dir,$(SOURCE_PATH),$(shell find $(dir) -name "*.[cs]" | sed -e "s/[cs]$/o/") )
VPATH = $(SOURCE_PATH)
DEPEND_FILES = $(foreach file, $(SOURCE_FILES), $(BUILD_DIR)/$(basename $(notdir $(file))).d)

TARGET := arm-none-eabi-


CFLAGS += $(addprefix -D,$(DEFINES)) $(addprefix -I,$(INCLUDE_PATH)) -std=gnu11 -g3
CFLAGS += -Os -ffunction-sections -fdata-sections -Wall -fomit-frame-pointer -fstack-usage -MMD -MP
CFLAGS += $(CPUFLAGS)

LDFLAGS += -Tlpc2138.ld
LDFLAGS += -Wl,-Map="$(BUILD_DIR)/map" 
LDFLAGS += -Wl,--gc-sections -static 
LDFLAGS += -nostartfiles
LDFLAGS += $(CPUFLAGS)

LIBS += -lm -lc

-include $(DEPEND_FILES)

default: $(BUILD_DIR)/elf

prepare: include/font_16segs.h include/font_7segs.h 
	mkdir -p $(BUILD_DIR)/ 

include/font_16segs.h: font/font_seg_remap.py font/base_font_16segs.txt
	cat font/base_font_16segs.txt |env python3 font/font_seg_remap.py bcdemnprkutsahgf > include/font_16segs.h

include/font_7segs.h: font/font_seg_remap.py font/base_font_7segs.txt
	cat font/base_font_7segs.txt |env python3 font/font_seg_remap.py cbadegf > include/font_7segs.h

clean:
	rm -fr $(BUILD_DIR)/

$(BUILD_DIR)/elf $(BUILD_DIR)/map $(BUILD_DIR)/asm $(BUILD_DIR)/bin: $(OBJECT_FILES)
	$(TARGET)gcc -o "$@" $(LDFLAGS) $(OBJECT_FILES) $(LIBS) 
	$(TARGET)objdump -h -S $(BUILD_DIR)/elf > "$(BUILD_DIR)/asm"
	$(TARGET)objcopy -O ihex -S $(BUILD_DIR)/elf $(BUILD_DIR)/hex
	srec_cat -o -binary $(BUILD_DIR)/hex -intel -offset - 0x08000000 > $(BUILD_DIR)/bin

$(BUILD_DIR)/%.o: %.c Makefile | prepare
	$(TARGET)gcc -o "$@" "$<" -c $(CFLAGS) -MT"$@" -MF"$(@:%.o=%.d)" 

$(BUILD_DIR)/%.o: %.s Makefile | prepare
	$(TARGET)gcc -o "$@" "$<" -c $(CFLAGS) -MT"$@" -MF"$(@:%.o=%.d)" 

load: all
	../lpc21isp/lpc21isp -wipe -verify $(BUILD_DIR)/hex  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A600JLJ7-if00-port0 115200 14745
	