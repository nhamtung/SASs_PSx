#!/bin/bash
echo Autoconnect Bluetooth
MAC="84:30:95:91:E5:59"
powered() {
    echo "show" | bluetoothctl | grep "Powered" | cut -d " " -f 2
}
connected() {
    echo "info ${MAC}" | bluetoothctl | grep "Connected" | cut -d " " -f 2
}
while true
do
    sleep 1
    if [ $(powered) = yes ] && [ $(connected) = no ]; then
        echo "connect ${MAC}" | bluetoothctl
        sleep 5
    fi
done