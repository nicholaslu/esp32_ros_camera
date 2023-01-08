# esp32_ros_camera
Add ROS-based camera image transfer support to ESP32.

## Hardware
ESP32-CAM

## Environment
PlatformIO on VS Code

# Network setting
Under `\src`, add new file `secret.c` with the content below:
```
const char* ssid     = "ssid";
const char* password = "password";
```
Modify IP address and server port in `main.cpp`.


## Dependence
### rosserial
Under `\lib`, run: 
```
git clone -b noetic https://github.com/nicholaslu/ros_lib.git
```
