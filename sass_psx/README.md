# Reference link: 
- ps4-ros: https://github.com/solbach/ps4-ros
- joy: http://wiki.ros.org/joy

# Require
- Ubuntu 18.04 Bionic
- ROS Melodic
- Hardware: PS4
- Install ds4dr: $sudo pip install ds4drv
- Install joy package: $sudo apt-get install ros-melodic-joy

# Check connect
- Check port: $ls /dev/input/js*
- Expected: /dev/input/js0
- Display raw data: $sudo jstest /dev/input/js0

# Run
- Run PS4: $roslaunch sass_psx psx_control_manual.launch
- Control PS4: 
    + Press and hold the L1 (max_linear_speed: 1m/s) or L2 (max_linear_speed: 0.3m/s)
    + Use joystick on the Left: Vertical is Linear and Horizontal is angular