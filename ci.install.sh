#!/bin/bash
set -e

dnf install git make glibc-devel gcc g++ patch tar diffutils jq -y

curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py

PIO_PATH=$HOME/.platformio/penv/bin
PATH=$PIO_PATH:$PATH 

git clone --recursive https://github.com/igrr/mkspiffs.git
cd mkspiffs 
make clean 
make dist BUILD_CONFIG_NAME="-arduino-esp32" CPPFLAGS="-DSPIFFS_OBJ_META_LEN=4"
cp mkspiffs $HOME/.platformio/penv/bin/mkspiffs_espressif32_arduino
cd ../
rm -rf mkspiffs


cd firmware  
platformio pkg install --environment esp32-c3-devkitm-1
cd ../