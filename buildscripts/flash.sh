#!/bin/bash
if [ "$1" == "bootloader" ]; then
    if [[ $2 -eq 0 ]]; then
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 bootloader.bin
    else
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 $2
    fi
elif [ "$1" == "partitiontable" ]; then
    if [[ $2 -eq 0 ]]; then
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x8000 binary_partitions.bin
    else
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x8000 $2
    fi
elif [ "$1" == "updater" ]; then
    if [[ $2 -eq 0 ]]; then
        pio run
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x10000 .pioenvs/esp32doit-devkit-v1/firmware.bin
    else
        pio run
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x10000 $2
    fi
elif [ "$1" == "firmware" ]; then
    if [[ $2 -eq 0 ]]; then
        pio run
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x130000 .pioenvs/esp32doit-devkit-v1/firmware.bin
    else
        pio run
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x130000 $2
    fi
else
    echo "Invalid parameter(s), try flash.sh [\"bootloader\", \"partitiontable\", \"updater\", \"firmware\"] <path_to_file.bin>"
fi
