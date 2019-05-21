#!/bin/bash
if [ "$1" == "updater" ]; then
    pio run
    python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x130000 $2
elif [ "$1" == "firmware" ]; then
    pio run
    python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x130000 $2
else
    echo "Invalid parameter(s), try flash.sh [\"bootloader\", \"partitiontable\", \"updater\", \"firmware\"] <path_to_file.bin>"
fi