1. Create CentOS VM with internet access and no user (just root).
2. Run setup.sh to install dependencies.
3. Pass through serial USB device to the VM.
4. Run flash.sh with appropriate option to clone, compile, and flash the partition.

From fresh CentOS:  
`cd /buildscripts`  
`/bin/bash setup.sh`  

To flash bootloader:  
`cd /buildscripts`  
`/bin/bash flash.sh [bootloader, partitiontable, updater, firmware] [path_to_bootloader.bin]`  