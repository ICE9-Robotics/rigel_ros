#!/bin/bash
export SUDO_ASKPASS="${HOME}/.ssh/.secrets/.supwd.sh"

if [ -n "$(ifconfig -a | grep can0)" ]
then
    if [ -z "$(ifconfig -a | grep can0 | grep UP)" ]
    then
        sudo -A ip link set can0 up type can bitrate 500000
    fi
    echo "CAN setup complete."
    exit 0
else
    echo "[ERROR] CAN0 not found."
    exit 1
fi