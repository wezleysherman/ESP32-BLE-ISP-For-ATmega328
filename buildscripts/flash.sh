#!/bin/bash
if [ "$1" == "bootloader" ]; then
    if [[ $2 -eq 0 ]]; then
        rm -rf Trynkit-Firmware-Updater
        git clone https://$(cat /root/.GITHUBTOKEN)@github.com/wezleysherman/Trynkit-Firmware-Updater
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 /root/Trynkit-Firmware-Updater/bootloader.bin
    else
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 $2
    fi
elif [ "$1" == "partitiontable" ]; then
    if [[ $2 -eq 0 ]]; then
        rm -rf TrynkitEsp32ISP
        git clone https://$(cat /root/.GITHUBTOKEN)E@github.com/wezleysherman/TrynkitEsp32ISP
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x8000 /root/TrynkitEsp32ISP/binary_partitions.bin
    else
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x8000 $2
    fi
elif [ "$1" == "updater" ]; then
    if [[ $2 -eq 0 ]]; then
        rm -rf Trynkit-Firmware-Updater
        git clone https://$(cat /root/.GITHUBTOKEN)@github.com/wezleysherman/Trynkit-Firmware-Updater
        cd TrynkitEsp32ISP
        pio run
        cd ../
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x130000 /root/Trynkit-Firmware-Updater/.pioenvs/esp32doit-devkit-v1/firmware.bin
    else
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x130000 $2
    fi
elif [ "$1" == "firmware" ]; then
    if [[ $2 -eq 0 ]]; then
        rm -rf TrynkitEsp32ISP
        git clone https://$(cat /root/.GITHUBTOKEN)@github.com/wezleysherman/TrynkitEsp32ISP
        cd TrynkitEsp32ISP
        pio run
        cd ../
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x130000 /root/TrynkitEsp32ISP/.pioenvs/esp32doit-devkit-v1/firmware.bin
    else
        python /root/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x130000 $2
    fi
else
    echo "Invalid parameter(s), try flash.sh [\"bootloader\", \"partitiontable\", \"updater\", \"firmware\"] <path_to_file.bin>"
fi
