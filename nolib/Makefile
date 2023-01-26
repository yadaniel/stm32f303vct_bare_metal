CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
NM = arm-none-eabi-nm
OD = arm-none-eabi-objdump
CP = arm-none-eabi-objcopy

# CFLAGS = -Wall -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections
# LDFLAGS = -Wall -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Wl,--gc-sections
CFLAGS = -Wall -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -ffunction-sections -fdata-sections
LDFLAGS = -Wall -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,--gc-sections

# pass parameters to make with
# make FLASH_START=0 FLASH_COUNT=255 dd
FLASH_START = 0x08000000
FLASH_COUNT = 0x400
FLASH_START_DEC = $(shell echo $$(($(FLASH_START))))
FLASH_COUNT_DEC = $(shell echo $$(($(FLASH_COUNT))))

# download ST-LINK_CLI utility from STM website
# command line options are described in UM0892
# -c connection mode with parameters
#   swd | jtag
#   freq = 4000k | 1800k | 900k | 480k
#   ur = connect under reset
# -me full flash erase
# -rst reset system | -hardrst
#
# NOTE: when loading binary, the load address is required
# ST-LINK_CLI.exe -c swd freq=1800k ur -me -p *.bin 0x08000000 -v -hardrst

# xxd -g4 -e -s0 -l32 main.bin
# -g group 4 bytes
# -a autoskip zeros and replace them compact
# -l read length
# -c bytes per line 
# -e little endian dump
# -s skip

.PHONY: clean, dd, debug

main.elf: main.o user.o map.ld
	$(CC) $(LDFLAGS) -nostdlib $< -Tmap.ld -Wl,-Map=main.map -o $@
	# $(CC) $(LDFLAGS) -specs=nano.specs -lc $< -Tmap.ld -Wl,-Map=main.map -o $@
	$(OD) -h $@

main.o: main.c
	$(CC) -c $< $(CFLAGS) -o $@

user.o: user.c
	$(CC) -c $< $(CFLAGS) -o $@

hex: main.elf
	$(CP) -O ihex main.elf main.hex

srec: main.elf
	$(CP) -O srec main.elf main.srec

bin: main.elf
	$(CP) -O binary main.elf main.bin

flash: hex
	ST-LINK_CLI.exe -c swd freq=1800k ur -me -p main.hex -v -hardrst
	# ST-LINK_CLI.exe -c swd -p main.hex -v -rst

read-dev:
	ST-LINK_CLI.exe -c swd freq=1800k -r8 $(FLASH_START) $(FLASH_COUNT)

read-bin: bin
	# dd if=main.bin bs=1 skip=134217728 count=1024 | hexdump -C
	# dd if=main.bin bs=1 skip=$(FLASH_START_DEC) count=$(FLASH_COUNT_DEC) | hexdump -C
	xxd -s $(FLASH_START) -l $(FLASH_COUNT) main.bin

dump: main.elf
	$(OD) -D $< > dump

list: main.elf
	$(NM) $< > list

debug:
	@echo "debug"
	@echo $(FLASH_START)
	@echo $(FLASH_COUNT)
	@echo $(FLASH_START_DEC)
	@echo $(FLASH_COUNT_DEC)

check-compiler:
	$(CC) -print-multi-lib
	@echo "----------------------------------------------------------------------"
	$(CC) $(CFLAGS) --print-multi-dir

clean:
	rm -f main.{o,elf,hex,srec,bin,map} user.o

