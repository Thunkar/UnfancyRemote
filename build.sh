#!/bin/bash
set -e

# Root 
yarn install

# Captive portals

cd ./software/app/frontend
yarn build --mode rx
mkdir -p dist/rx
mv dist/index.html dist/rx/index.html
yarn build --mode tx
mkdir -p dist/tx
mv dist/index.html dist/tx/index.html

# ESP32 firmwares

PATH=$PATH:$HOME/.platformio/penv/bin
cd ../../rx
rm -rf data
mkdir -p data
cp ../app/frontend/dist/rx/index.html ./data/index.html

platformio run --target buildfs --environment esp32-c3-devkitm-1

MERGED_BIN_PATH=merged.bin pio run -t mergebin

cd ../tx
rm -rf data
mkdir -p data
cp ../app/frontend/dist/tx/index.html ./data/index.html

platformio run --target buildfs --environment esp32-c3-devkitm-1

MERGED_BIN_PATH=merged.bin pio run -t mergebin

# Landing page

cd ../../landing
mv ../software/tx/merged.bin ./public/merged.bin
mv ../software/rx/merged.bin ./public/merged.bin