# Reference link: 
- ps4-ros: https://github.com/solbach/ps4-ros
- joy: http://wiki.ros.org/joy

# Require
- Ubuntu 18.04 Bionic
- ROS Melodic
- Hardware: PS4 - Bluetooth
- Install ds4dr: $sudo pip install ds4drv
- Install joy package: $sudo apt-get install ros-melodic-joy

# Check connect
- Check port: $ls /dev/input/js*
- Expected: /dev/input/js0
- Display raw data: $sudo jstest /dev/input/js0

# Run
- Run PS4: $roslaunch sass_psx psx_control_manual.launch
- Control PS4: 
    + Press and hold the L1 or L2 to select max_linear_speed
    + Use Vertical of left joystick to control Linear and Horizontal of right joystick to control Angular

# Autoconnect Bluetooth with systemctl (Autostart when boot)
- Create bash file:
    + Direct to folder: /SASs_PSx_ws/src/SASs_PSx/sass_psx
    + Create and edit bash file: $sudo nano autoconnect_bluetooth.sh
    + Add to bash file: 
```
#!/bin/bash
echo Autoconnect Bluetooth
MAC="xx:xx:xx:xx:xx:xx"
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
```
- Permit bash file:
    + $sudo chmod +x autoconnect_bluetooth.sh
- Create service:
    + Direct to system folder: $cd /etc/systemd/system/
    + Create service: $sudo nano autoconnect_bluetooth.service
    + Add to dervice file:
```
[Unit]
After=network.target
StartLimitIntervalSec=0
[Service]
User=nhamtung
Type=simple
Restart=always
RestartSec=1
ExecStart=/home/nhamtung/TungNV/SASs_PSx_ws/src/SASs_PSx/sass_psx/autoconnect_bluetooth.sh
RemainAfterExit=no
[Install]
WantedBy=multi-user.target
```
- Enable service: $sudo systemctl enable autoconnect_bluetooth.service
- Start service: $sudo systemctl start autoconnect_bluetooth.service
- Stop service: $sudo systemctl stop autoconnect_bluetooth.service
- Disable service: $sudo systemctl disable autoconnect_bluetooth.service
- Restart service: $sudo systemctl restart autoconnect_bluetooth.service
- Check status service: $sudo systemctl status autoconnect_bluetooth.service
- Reload daemon: $sudo systemctl daemon-reload