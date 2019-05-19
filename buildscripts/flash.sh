#!/bin/bash
if [ "$1" == "bootloader" ]; then
    $offset = 0x1000
    echo "Offset set for bootloader ($offset)"
elif [ "$1" == "partitiontable" ]; then
    $offset = 0x8000
    echo "Offset set for partitiontable ($offset)"
elif [ "$1" == "updater" ]; then
    $offset = 0x10000
    echo "Offset set for updater ($offset)"
elif [ "$1" == "firmware" ]; then
    $offset = 0x130000
    echo "Offset set for firmware ($offset)"
else
    echo "Invalid offset parameter, try flash.sh [bootloader, partitiontable, updater, firmware] [path_to_bootloader.bin]"
fi

python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect $offset $2