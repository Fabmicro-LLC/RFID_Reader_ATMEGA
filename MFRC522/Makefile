CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
CP=$(CROSS_COMPILE)objcopy
AS=$(CROSS_COMPILE)as
STRIP=$(CROSS_COMPILE)strip
OBJDUMP=$(CROSS_COMPILE)objdump

BUILD_NUMBER=`cat build_number`

#MCFLAGS = -s -Wl,-u,vfprintf -lprintf_flt -Os -std=c99 -mmcu=atmega16
#MCFLAGS = -Os -std=c99 -mmcu=atmega16 
#MCFLAGS = -Os -std=c99 -mmcu=avr5 -mmcu=atmega16 -Wl,-u,vfprintf -lprintf_min -Wl,--gc-sections -Wl,-Map,main.map,--cref 
#MCFLAGS = -Os -std=c99 -Wl,-u,vfprintf -lprintf_min -Wl,--gc-sections -Wl,-Map,main.map,--cref 
MCFLAGS +=  -Os -std=c99 -Wl,-u,vfprintf -lprintf_flt  

STATIC_LIBS =

CFLAGS	+= $(MCFLAGS) $(INCLUDES) -mmcu=atmega16 -DBUILD_NUMBER=$(BUILD_NUMBER)+1  -fno-move-loop-invariants
CFLAGS  := $(CFLAGS) -DDEBUG=1
LDFLAGS += -mmcu=atmega64  -Wl,-u,vfprintf -lprintf_flt 


all: mfrc522.hex

mfrc522.hex: mfrc522.elf
	$(CP) -j .text -j .data -O ihex $^ $@

%.o: %.c 
	$(CC) -c $(CFLAGS) $^ $(LDFLAGS) $(STATIC_LIBS) -o $@

mfrc522.elf: main.o mfrc522.o 
	@expr $(BUILD_NUMBER) + 1 > build_number 
	$(CC) $^ $(LDFLAGS) $(STATIC_LIBS) -o $@
	$(STRIP) $@

disass:	mfrc522.elf
	$(OBJDUMP) --disassemble-all $^

install:
	#avrdude -p m64 -c avr910 -e -U flash:w:mfrc522.hex -P /dev/ttyACM0 -B 115200
	avrdude -p m64 -c avrispmkII -U  flash:w:mfrc522.hex -P usb


fuse8M:
	# 8 MHz internal RC 
	#avrdude -p m64 -c avr910 -U lfuse:w:0xE4:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m
	avrdude -p m64 -c avrispmkII -U lfuse:w:0xE4:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m -P usb

fuse14M:
	# 14 MHz external crystal
	avrdude -p m64 -c avrispmkII -P usb -U lfuse:w:0xFF:m -U hfuse:w:0xC9:m -U efuse:w:0xFF:m 

clean:
	rm *.hex *.elf *.o *.map


