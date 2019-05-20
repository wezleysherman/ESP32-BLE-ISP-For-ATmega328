#!/bin/bash
yum update -y
yum install epel-release -y
yum install python-pip -y
yum install git -y
pip install --upgrade pip
pip install -U platformio

cd /root/
git clone -b v3.2 --recursive https://github.com/espressif/esp-idf.git
python -m pip install --user -r esp-idf/requirements.txt
