#!/bin/sh

set -e
set -x

echo "Removing old build artifacts"
rm -rf ./Logger/build/
rm -rf ./NanoLogger/build/

echo "Configuring Arduino"
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli core update-index

echo "Installing the cores"
arduino-cli core install Seeeduino:nrf52@1.1.1
arduino-cli core install arduino:avr@1.8.6

echo "Installing libraries"
arduino-cli lib install SdFat@2.2.0
arduino-cli lib install U8g2@2.33.15
arduino-cli lib install ezButton@1.0.4
arduino-cli lib install RTClib@2.1.1

echo "Compiling the Logger sketch"
arduino-cli compile --verbose --export-binaries \
    --build-property compiler.c.extra_flags=-DUSE_SPI_ARRAY_TRANSFER=1 \
    --build-property compiler.cpp.extra_flags=-DUSE_SPI_ARRAY_TRANSFER=1 \
    ./Logger/

echo "Compiling the NanoLogger sketch"
arduino-cli compile --verbose --export-binaries ./NanoLogger/

echo "Generating the UF2 image"
./utils/uf2/utils/uf2conv.py \
    --convert \
    --base 0x26000 \
    --family 0xADA52840 \
    --output ./Logger/build/Seeeduino.nrf52.xiaonRF52840Sense/Logger.ino.uf2 \
    ./Logger/build/Seeeduino.nrf52.xiaonRF52840Sense/Logger.ino.hex
