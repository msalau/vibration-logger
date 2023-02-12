#!/bin/sh

set -e
set -x

echo "Removing old build artifacts"
rm -rf ./Logger/build/

echo "Configuring Arduino"
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli core update-index

echo "Installing the core"
arduino-cli core install Seeeduino:nrf52@1.1.1

echo "Installing libraries"
arduino-cli lib install SD@1.2.4
arduino-cli lib install U8g2@2.33.15
arduino-cli lib install ezButton@1.0.4
arduino-cli lib install RTClib@2.1.0

echo "Compiling the sketch"
arduino-cli compile --export-binaries --fqbn Seeeduino:nrf52:xiaonRF52840Sense:softdevice=s140v6,debug=l0 ./Logger/

echo "Generating the UF2 image"
./utils/uf2/utils/uf2conv.py \
    --convert \
    --base 0x26000 \
    --family 0xADA52840 \
    --output ./Logger/build/Seeeduino.nrf52.xiaonRF52840Sense/Logger.ino.uf2 \
    ./Logger/build/Seeeduino.nrf52.xiaonRF52840Sense/Logger.ino.hex
