#!/bin/bash
for device in /dev/video*; do
    echo "Device: $device"
    v4l2-ctl -d $device --all
    echo
done