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
#MCFLAGS =  -Os -std=c99 -Wl,-u,vfprintf -lprintf_flt  
MCFLAGS =  -Os -std=c99 
INCLUDES = -I/home/rz/work/AVR/avr/include

STATIC_LIBS =

CFLAGS	= $(MCFLAGS) $(INCLUDES) -mmcu=atmega8 -DBUILD_NUMBER=$(BUILD_NUMBER)+1  -fno-move-loop-invariants #-DDEBUG=1

LDFLAGS = -mmcu=atmega8  


all: main.hex

main.hex: main.elf
	$(CP) -j .text -j .data -O ihex $^ $@

main.o: main.c 
	$(CC) -c $(CFLAGS) $^ $(LDFLAGS) $(STATIC_LIBS) -o $@

main.elf: main.o  
	@expr $(BUILD_NUMBER) + 1 > build_number 
	$(CC) $^ $(LDFLAGS) $(STATIC_LIBS) -o $@
	$(STRIP) $@

disass:	main.elf
	$(OBJDUMP) --disassemble-all $^

install:
	avrdude -p m8 -c avr910 -e -U flash:w:main.hex -P /dev/ttyACM0 -B 115200


# fuses: CKSEL = 0100 - 8MHz, CKOPT = 1 (unprogrammed), JTAGEN = 1 (disabled)
fuse:
	avrdude -p m8 -c avr910 -U lfuse:w:0xE4:m -U hfuse:w:0xD9:m -P $(PORT) -B $(BAUD)


clean:
	rm main.hex main.o

