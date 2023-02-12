#!/bin/sh

if [ $# -ne 1 ]; then
    echo "Usage $0 <port>"
    exit 1
fi

PORT=$1

stty -F $PORT 115200 raw -echo

cat $PORT &
PID=$!
printf "time $(date '+%d.%m.%Y %H:%M:%S')\r\n" > $PORT
sleep 0.2s
kill $PID 
wait $PID 2>/dev/null
