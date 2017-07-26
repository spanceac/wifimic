SER_PORT=/dev/ttyUSB0
CC = xtensa-lx106-elf-gcc
SDK_PATH = /home/spanceac/sebu/esp-devel/new_sdk/ESP8266_NONOS_SDK
CFLAGS = -I. -mlongcalls -I$(SDK_PATH)/include
CFLAGS += -L$(SDK_PATH)/include
CFLAGS += -I$(SDK_PATH)/driver_lib/include/driver
CFLAGS += -I$(SDK_PATH)/driver_lib/include/

LDLIBS = -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -lc -ldriver -lwpa2 -Wl,--end-group -lgcc -lcrypto
LDFLAGS = -T$(SDK_PATH)/ld/eagle.app.v6.ld -L$(SDK_PATH)/ld -L$(SDK_PATH)/lib

wifimic-0x00000.bin: wifimic
	esptool.py elf2image $^

wifimic: wifimic.o

wifimic.o: wifimic.c

flash: wifimic-0x00000.bin
	esptool.py --port $(SER_PORT) write_flash --flash_mode dio 0 wifimic-0x00000.bin 0x10000 wifimic-0x10000.bin

clean:
	rm -f wifimic wifimic.o wifimic-0x00000.bin wifimic-0x10000.bin
