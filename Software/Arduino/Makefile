# Hey Emacs, this is a -*- makefile -*-

# AVR-GCC Makefile template, derived from the WinAVR template (which
# is public domain), believed to be neutral to any flavor of "make"
# (GNU make, BSD make, SysV make)


MCU = atmega328p
ARDUINO_DIR = /usr/share/arduino
ARDUINO_CORE_PATH= $(ARDUINO_DIR)/hardware/arduino/cores/arduino
ARDUINO_VARIANT= $(ARDUINO_DIR)/hardware/arduino/variants/standard
ARDUINO_LIB_PATH= $(ARDUINO_DIR)/libraries
USER_LIB_PATH := /home/jgeisler/sketchbook/libraries
ARDUINO_VERSION= 105

F_CPU = 16000000

FORMAT = ihex
TARGET = SlaveArduino
SRC= 
CPPSRC= 	timer_Arduino.cpp\
	dcf.cpp\
	timer.cpp\
	emcy.cpp\
	lifegrd.cpp\
	lss.cpp\
	nmtMaster.cpp\
	nmtSlave.cpp\
	objacces.cpp\
	pdo.cpp\
	sdo.cpp\
	states.cpp\
	sync.cpp\
	ObjDict.cpp\
	ds401.cpp\
	main.cpp

USER_LIB_CPP_SRCS= $(USER_LIB_PATH)/mcp_can/mcp_can.cpp

LIB_CPP_SRCS= $(ARDUINO_LIB_PATH)/SPI/SPI.cpp

CORE_C_SRCS = $(wildcard $(ARDUINO_CORE_PATH)/*.c)
CORE_C_SRCS += $(wildcard $(ARDUINO_CORE_PATH)/avr-libc/*.c)
CORE_CPP_SRCS = $(wildcard $(ARDUINO_CORE_PATH)/*.cpp)
CORE_CPP_SRCS := $(filter-out %main.cpp, $(CORE_CPP_SRCS))


OBJDIR=obj
EXTRAINCDIRS = $(USER_LIB_PATH)/mcp_can \
	$(ARDUINO_CORE_PATH) \
	$(ARDUINO_VARIANT) \
	$(ARDUINO_LIB_PATH)/SPI
	
#	$(ARDUINO_VAR_PATH)/$(VARIANT) \
#	$(SYS_INCLUDES) \
#	$(PLATFORM_INCLUDES)

ASRC = 
OPT = s

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu++11

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)UL -DARDUINO=$(ARDUINO_VERSION) -D__PROG_TYPES_COMPAT__

# Place -I options here
CINCS = 


CDEBUG = -g$(DEBUG)
CWARN = -Wall
# -Wstrict-prototypes
CTUNING = -fsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)
CEXTRA = -ffunction-sections -fdata-sections
CFLAGS = $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CSTANDARD) $(CEXTRA)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

 


#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 


#Additional libraries.

# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

PRINTF_LIB = 

# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

SCANF_LIB = 

MATH_LIB = -lm

# External memory options

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,--section-start,.data=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS = -Wl,--gc-sections -O$(OPT)

#LDMAP = $(LDFLAGS) -Wl,-Map=$(TARGET).map,--cref
LDFLAGS = $(EXTMEMOPTS) $(LDMAP) $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)


# Programming support using avrdude. Settings and variables.

AVRDUDE_PROGRAMMER = arduino
AVRDUDE_PORT = /dev/ttyUSB0

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep


# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_BASIC = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS = $(AVRDUDE_BASIC) $(AVRDUDE_NO_VERIFY) $(AVRDUDE_VERBOSE) $(AVRDUDE_ERASE_COUNTER)


CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AR= avr-ar
AVRDUDE = avrdude
REMOVE = rm -f
REMOVEDIR = rm -rf
MV = mv -f
MKDIR= mkdir -p

# Define all object files.
# OBJ = $(SRC:.c=.o) $(ASRC:.S=.o) 
LOCAL_OBJS = $(SRC:%.c=$(OBJDIR)/%.o) $(CPPSRC:%.cpp=$(OBJDIR)/%.o) $(ASRC:%.S=$(OBJDIR)/%.o) 
#OBJ_FILES = $(SRC: .c=.o) $(CPPSRC:.cpp=.o) $(ASRC:.S=.o) 
#OBJS = $(patsubst %,$(OBJDIR)/%,$(OBJ_FILES))
LIB_OBJS = $(patsubst $(ARDUINO_LIB_PATH)/%.cpp,$(OBJDIR)/libs/%.o,$(LIB_CPP_SRCS)) \
	$(patsubst $(ARDUINO_LIB_PATH)/%.c,$(OBJDIR)/libs/%.o,$(LIB_C_SRCS)) \
	$(patsubst $(ARDUINO_LIB_PATH)/%.S,$(OBJDIR)/libs/%.o,$(LIB_AS_SRCS))

USER_LIB_OBJS = $(patsubst $(USER_LIB_PATH)/%.cpp,$(OBJDIR)/userlibs/%.o,$(USER_LIB_CPP_SRCS)) \
	$(patsubst $(USER_LIB_PATH)/%.c,$(OBJDIR)/userlibs/%.o,$(USER_LIB_C_SRCS)) \
	$(patsubst $(USER_LIB_PATH)/%.S,$(OBJDIR)/userlibs/%.o,$(USER_LIB_AS_SRCS))

CORE_OBJ_FILES = $(CORE_C_SRCS:.c=.o) $(CORE_CPP_SRCS:.cpp=.o) $(CORE_AS_SRCS:.S=.o)
CORE_OBJS = $(patsubst $(ARDUINO_CORE_PATH)/%, \
	$(OBJDIR)/core/%,$(CORE_OBJ_FILES))

OBJ= $(LOCAL_OBJS) $(USER_LIB_OBJS) $(LIB_OBJS) $(CORE_OBJS)

CORE_LIB = $(OBJDIR)/libcore.a

# Define all listing files.
# LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)
LST = $(SRC:%.c=$(OBJDIR)/%.lst) $(CPPSRC:%.cpp=$(OBJDIR)/%.lst) $(ASRC:%.S=$(OBJDIR)/%.lst) 

# Compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)  $(GENDEPFLAGS)
ALL_CPPFLAGS = -mmcu=$(MCU) -I. -x c++ $(CFLAGS)  $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)


# Default target.
all: build

build: elf hex eep size

elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss 
sym: $(TARGET).sym


# Program the device.  
program: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)




# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 


coff: $(TARGET).elf
	$(COFFCONVERT) -O coff-avr $(TARGET).elf $(TARGET).cof


extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr $(TARGET).elf $(TARGET).cof


.SUFFIXES: .elf .hex .eep .lss .sym

.elf.hex:
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@



# Link: create ELF output file from object files.
$(TARGET).elf: $(OBJ) 
	$(CC) $(ALL_CFLAGS) $(OBJ) --output $@ $(LDFLAGS)

$(CORE_LIB): $(CORE_OBJS) $(LIB_OBJS) $(PLATFORM_LIB_OBJS) $(USER_LIB_OBJS)
	$(AR) rcs $@ $(CORE_OBJS) $(LIB_OBJS) $(PLATFORM_LIB_OBJS) $(USER_LIB_OBJS)

# Compile: create object files from C source files.
# .c.o:
$(OBJDIR)/%.o : %.c
	$(CC) -c $(ALL_CFLAGS) $< -o $@
#	gcc -MM $(CFLAGS) $*.c > $*.d
#	@mv -f $*.d $*.d.tmp
#	@sed -e 's|.*:|$*.o:|' < $*.d.tmp > $*.d
#	@cp -f $*.d $*.d.tmp
#	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
#	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
#	@rm -f $*.d.tmp

$(OBJDIR)/%.o : %.cpp
	$(CC) -c $(ALL_CPPFLAGS) $< -o $@

$(OBJDIR)/libs/%.o : $(ARDUINO_LIB_PATH)/%.cpp
	@$(MKDIR) $(dir $@)
	$(CC) -c $(ALL_CPPFLAGS) $< -o $@

$(OBJDIR)/platformlibs/%.o : $(ARDUINO_PLATFORM_LIB_PATH)/%.cpp
	@$(MKDIR) $(dir $@)
	$(CC) -c $(ALL_CPPFLAGS) $< -o $@

$(OBJDIR)/userlibs/%.o : $(USER_LIB_PATH)/%.cpp
	@$(MKDIR) $(dir $@)
	$(CC) -c $(ALL_CPPFLAGS) $< -o $@

$(OBJDIR)/core/%.o : $(ARDUINO_CORE_PATH)/%.cpp
	@$(MKDIR) $(dir $@)
	$(CC) -c $(ALL_CPPFLAGS) $< -o $@

$(OBJDIR)/core/%.o : $(ARDUINO_CORE_PATH)/%.c
	@$(MKDIR) $(dir $@)
	$(CC) -c $(ALL_CFLAGS) $< -o $@

# -include $(OBJ:.o=.d)

# Compile: create assembler files from C source files.
.c.s:
	$(CC) -S $(ALL_CFLAGS) $< -o $@

.cpp.s:
	$(CC) -S $(ALL_CPPFLAGS) $< -o $@

# Assemble: create object files from assembler source files.
$(OBJDIR)/%.o : %.S
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

size: $(TARGET).elf
	@$(SIZE) -C --mcu=${MCU} $(TARGET).elf

# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@


# Target: clean project.
clean:
	$(REMOVE) $(TARGET).hex $(TARGET).eep $(TARGET).cof $(TARGET).elf \
	$(TARGET).map $(TARGET).sym $(TARGET).lss \
	$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d)
	$(REMOVEDIR) .dep
	$(REMOVEDIR) $(OBJDIR)

# Create object files directory
$(shell mkdir $(OBJDIR) 2>/dev/null)

# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

.PHONY:	all build elf hex eep lss sym program coff extcoff clean


