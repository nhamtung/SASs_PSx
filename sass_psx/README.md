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
- Enable psx_mannual: Publish to topic /enable_psx_manual with mode=1
- Control PS4: 
    + Press and hold the L1 or L2 to select max_linear_speed and max_angular_speed
      + L1: max_linear_speed = 1m/s, max_angular_speed = 0.6 rad/s
      + L2: max_linear_speed = 0.3m/s, max_angular_speed = 0.25 rad/s
    + Use Vertical of left joystick to control Linear and Horizontal of right joystick to control Angular
- Check status: $rostopic echo /psx_manual_status

# Install package
- Add command to install in CMakeLists.txt:
```
#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
## Mark executables for installation
install(TARGETS psx_manual_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/ps4.launch
  launch/psx_control_manual.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES
  config/ps4.config.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
```
- Build install package: $catkin_make install
- Source: $source install/setup.bash

# Autoconnect Bluetooth with systemctl (Autostart when boot)
- Create bash file:
    + Direct to folder: /SASs_AMR_ws/src/SASs_AMR/base_controller/sass_startup
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
    sleep 5
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
User=robotics
Type=simple
Restart=always
RestartSec=1
ExecStart=/home/robotics/SASs_AMR_ws/src/SASs_AMR/base_controller/amr_startup/sass_startup/sass_startup/autoconnect_bluetooth.sh
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