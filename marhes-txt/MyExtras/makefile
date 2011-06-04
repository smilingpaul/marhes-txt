NAME   = lpc_p2378

CC      = arm-none-eabi-gcc-4.5.0
LD      = arm-none-eabi-ld
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump

IDIR = Includes
ODIR = Output
SDIR = Source

OBJECTS = $(ODIR)/crt0.o $(ODIR)/main.o $(ODIR)/pwm.o $(ODIR)/uart0.o 
OBJECTS += $(ODIR)/uart0ISR.o $(ODIR)/lcd.o $(ODIR)/armVIC.o

CFLAGS  = -I $(IDIR) -c -fcommon -O0 -g
AFLAGS  = -ahls -mapcs-32 -o
LFLAGS  =  -Map $(ODIR)/main.map -T LPC2378-ROM.ld
CPFLAGS = -O binary
HEXFLAGS = -O ihex
ODFLAGS	= -x --syms

all: test

clean:
	-rm -f $(ODIR)/*.lst $(ODIR)/main.* $(ODIR)/*.o 

test: $(ODIR)/main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) $(ODIR)/main.out $(ODIR)/main.bin
	$(OD) $(ODFLAGS) $(ODIR)/main.out > $(ODIR)/main.dmp
	@echo "...building hex"
	$(CP) $(HEXFLAGS) $(ODIR)/main.out $(ODIR)/main.hex

$(ODIR)/main.out: $(OBJECTS) LPC2378-ROM.ld
	@ echo "..linking"
	$(LD) $(LFLAGS) -o $(ODIR)/main.out $(OBJECTS)

$(ODIR)/%.o : $(SDIR)/%.s
	$(AS) $(AFLAGS) $@ $<

$(ODIR)/%.o : $(SDIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@