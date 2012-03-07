TARGET=clocky
SOURCES=clocky.c

MCU=msp430g2452
#MCU=msp430g2553
#MCU=msp430g2231

CFLAGS   = -mmcu=$(MCU) -g -Os -Wall -Wunused $(INCLUDES)   
ASFLAGS  = -mmcu=$(MCU) -x assembler-with-cpp -Wa,-gstabs
LDFLAGS  = -mmcu=$(MCU) -Wl,-Map=$(TARGET).map

CC       = msp430-gcc
LD       = msp430-ld
AR       = msp430-ar
AS       = msp430-gcc
GASP     = msp430-gasp
NM       = msp430-nm
OBJCOPY  = msp430-objcopy
RANLIB   = msp430-ranlib
STRIP    = msp430-strip
SIZE     = msp430-size
READELF  = msp430-readelf
MAKETXT  = srec_cat
CP       = cp -p
RM       = rm -f
MV       = mv

DEPEND   = $(SOURCES:.c=.d)
OBJECTS  = $(SOURCES:.c=.o)

all: $(TARGET).elf $(TARGET).hex $(TARGET).txt

$(TARGET).elf: $(OBJECTS)
	echo "Linking $@"
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	echo
	echo ">>>>> Size of firmware <<<<<"
	$(SIZE) $(TARGET).elf
	echo

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@
%.txt: %.hex
	$(MAKETXT) -O $@ -TITXT $< -I
%.o: %.c
	echo "Compiling $<"
	$(CC) -c $(CFLAGS) -o $@ $<
%.lst: %.c
	$(CC) -c $(CFLAGS) -Wa,-anlhd $< > $@

ifneq ($(MAKECMDGOALS), clean)
-include $(DEPEND)
endif

%.d: %.c
	echo "Generating dependencies $@ from $<"
	$(CC) -M ${CFLAGS} $< >$@

#.SILENT:
.PHONY:	clean

clean:
	$(RM) $(OBJECTS)
	$(RM) $(TARGET).{elf,hex,map,txt}
	$(RM) $(SOURCES:.c=.lst)
	$(RM) $(DEPEND)

