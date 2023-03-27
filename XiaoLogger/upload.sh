#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Usage $0 <file> <port>"
    exit 1
fi

FILE=$1
PORT=$2

set -e
set -x

adafruit-nrfutil \
    --verbose dfu serial \
    --package $FILE \
    --port $PORT \
    --baudrate 115200 \
    --touch 1200 \
    --singlebank
