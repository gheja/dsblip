#!/bin/bash

program="dsblip"
programmer="usbasp"
# programmer="usbtiny"
# profile="attiny13"
profile="attiny85"
# profile="trinket"

if [ "$profile" == "attiny13" ]; then
	mcu1="attiny13"
	mcu2="t13"
	f_cpu="8000000"
elif [ "$profile" == "attiny85" ]; then
	mcu1="attiny85"
	mcu2="t85"
	f_cpu="8000000"
	# # 6.4 MHz / 8 = 800 kHz
	# f_cpu="800000"
elif [ "$profile" == "atmega328p" ]; then
	mcu1="atmega328p"
	mcu2="m328p"
	f_cpu="8000000"
elif [ "$profile" == "trinket" ]; then
	programmer="usbtiny"
	mcu1="attiny85"
	mcu2="t85t"
	f_cpu="8000000"
else
	echo "Unknown MCU, exiting."
	exit 1
fi

cd src

avr-gcc -std=c99 -g -Os -mmcu=${mcu1} -o ${program}.o -DF_CPU=${f_cpu} \
	ds18b20.c \
	onewire.c \
	radio.c \
	utils.c \
	main.c || exit 1

# 	serial.c \


avr-objcopy -j .text -j .data -O ihex ${program}.o ${program}.hex || exit 1
avr-size -d --mcu=${mcu1} ${program}.o
avr-objdump -d ${program}.o > ${program}.asm

final_size=`avr-size -d --mcu=${mcu1} ${program}.o | tail -n 1 | awk '{ print $4; }'`

# E:FF, H:DF, L:E2
# sudo avrdude -c ${programmer} -p ${mcu2} -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

# sudo avrdude -c ${programmer} -p ${mcu2} -U flash:w:${program}.hex

# E:07, H:D9, L:E2
# sudo avrdude -c ${programmer} -p ${mcu2} -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m

sudo avrdude -c ${programmer} -p ${mcu2} -U flash:w:${program}.hex

# now=`date +%Y%m%d_%H%M%S`
# zip -r9 ../${now}_size_${final_size}.zip `ls -1 | grep -vE '\.jpg$'` | grep -vE '^  adding:'

