#
#  Wirekite - MCU code 
#  Copyright (c) 2017 Manuel Bleichenbacher
#  Licensed under MIT License
#  https://opensource.org/licenses/MIT
#

#  Project Name
PROJECT = wirekite

#  Set your MCU type here
#  - TeensyLC: MKL26Z64 / cortex-m0plus / 48000000
MCU = MKL26Z64
CPU = cortex-m0plus
F_CPU = 48000000

#  Tool path (GNU ARM Embedded Toolchain base directory)
TOOLDIR = ~/Documents/Software/gcc-arm-none-eabi

#  Project directory structure
SRCDIR = src
OUTPUTDIR = bin
OBJDIR = obj

INCDIRS = -Iinclude
LIBS = 
STD_LIBS = -lm

#  Project C & C++ files which are to be compiled
CPP_FILES = $(wildcard $(SRCDIR)/*.cpp)
C_FILES = $(wildcard $(SRCDIR)/*.c)
AS_FILES = $(wildcard $(SRCDIR)/*.s)
LDSCRIPT = src/$(MCU).ld

#  Change project C & C++ files into object files
OBJ_FILES := $(addprefix $(OBJDIR)/,$(notdir $(CPP_FILES:.cpp=.o)))
OBJ_FILES += $(addprefix $(OBJDIR)/,$(notdir $(C_FILES:.c=.o)))
OBJ_FILES += $(addprefix $(OBJDIR)/,$(notdir $(AS_FILES:.s=.o)))

#  Compiler options for C and C++
CPPFLAGS = -Wall -mthumb -Os -MMD -D__$(MCU)__ -DF_CPU=$(F_CPU) $(INCDIRS) -mcpu=$(CPU) -fsingle-precision-constant -fno-common

#  Compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections -fno-common

ASFLAGS = -x assembler-with-cpp 

#  Compiler options for C only
CFLAGS = 

#  Linker options
LDFLAGS  = -Wl,--gc-sections --specs=nano.specs -mcpu=$(CPU) -mthumb -T$(LDSCRIPT)

TOOLBINDIR = $(TOOLDIR)/bin

CC = $(TOOLBINDIR)/arm-none-eabi-gcc
AS = $(TOOLBINDIR)/arm-none-eabi-as
AR = $(TOOLBINDIR)/arm-none-eabi-ar
LD = $(TOOLBINDIR)/arm-none-eabi-ld
OBJCOPY = $(TOOLBINDIR)/arm-none-eabi-objcopy
SIZE = $(TOOLBINDIR)/arm-none-eabi-size
OBJDUMP = $(TOOLBINDIR)/arm-none-eabi-objdump
REMOVE = rm -rf


#  Main rules

all:: $(OUTPUTDIR)/$(PROJECT).hex stats

$(OUTPUTDIR)/$(PROJECT).bin: $(OUTPUTDIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary -j .text -j .data $(OUTPUTDIR)/$(PROJECT).elf $(OUTPUTDIR)/$(PROJECT).bin

$(OUTPUTDIR)/$(PROJECT).hex: $(OUTPUTDIR)/$(PROJECT).elf
	$(OBJCOPY) -R .stack -O ihex $(OUTPUTDIR)/$(PROJECT).elf $(OUTPUTDIR)/$(PROJECT).hex

#  Linker invocation
$(OUTPUTDIR)/$(PROJECT).elf: $(OBJ_FILES) $(LIBS)
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) $(OBJ_FILES) $(LIBS) $(STD_LIBS) -o $(OUTPUTDIR)/$(PROJECT).elf

#  Additional targets
stats: $(OUTPUTDIR)/$(PROJECT).elf
	$(SIZE) $(OUTPUTDIR)/$(PROJECT).elf
	
dump: $(OUTPUTDIR)/$(PROJECT).elf
	$(OBJDUMP) -h $(OUTPUTDIR)/$(PROJECT).elf    

burn: $(OUTPUTDIR)/$(PROJECT).hex
	teensy_loader_cli -mmcu=$(MCU) -w -v $<

teensylc: $(OUTPUTDIR)/$(PROJECT).hex
	cp $(OUTPUTDIR)/wirekite.hex $(OUTPUTDIR)/wirekite_teensylc.hex

clean:
	$(REMOVE) $(OBJDIR)
	$(REMOVE) $(OUTPUTDIR)/*.elf
	$(REMOVE) $(OUTPUTDIR)/wirekite.hex

toolvers:
	$(CC) --version | sed q
	$(AS) --version | sed q
	$(LD) --version | sed q
	$(AR) --version | sed q
	$(OBJCOPY) --version | sed q
	$(SIZE) --version | sed q
	$(OBJDUMP) --version | sed q


#  Rules

$(OBJDIR)/%.o : $(SRCDIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS)  $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/%.o : $(SRCDIR)/%.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -o $@ $<


#  Compiler generated dependency info
-include $(OBJ_FILES:.o=.d)


.PHONY: all
