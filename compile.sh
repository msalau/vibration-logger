#!/bin/sh

set -e
set -x

COMPILE_ARGS="
    --export-binaries
    --verbose
    --build-property compiler.c.extra_flags=-DUSE_SPI_ARRAY_TRANSFER=1
    --build-property compiler.cpp.extra_flags=-DUSE_SPI_ARRAY_TRANSFER=1
"

arduino-cli compile $COMPILE_ARGS --profile xiao ./XiaoLogger
arduino-cli compile $COMPILE_ARGS --profile nano ./NanoLogger
arduino-cli compile $COMPILE_ARGS --profile pico ./PicoLogger
