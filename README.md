1. Create CentOS 7 VM with internet access and no user (just root).
2. In the Setup, make sure ethernet is set to ON
3. Get the buildscripts onto the VM in a way of your choosing (SFTP, Filezilla, etc)
4. Setup a Git token  
    * Go to Github
    * Go to Settings > Developer Settings > Personal Access Tokens
    * Click "Generate a New Token" and enter your password again
    * Set a description/name for it, check the "repo" permission and hit the "Generate token" button at the bottom of the page.
    * __*Copy your new token before you leave the page*__
    * Create a new file on your VM: `/root/.GITHUBTOKEN` 
    * Paste the token and save
4. Run setup.sh to install dependencies.
5. Pass through serial USB device to the VM.
6. Run flash.sh with appropriate parameters. This will clone, compile, then flash the partition.

From fresh CentOS:  
`cd /buildscripts`  
`/bin/bash setup.sh`  

To flash bootloader:  
`cd /buildscripts`  
`/bin/bash flash.sh [bootloader, partitiontable, updater, firmware] [path_to_bootloader.bin]`  