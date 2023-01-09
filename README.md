# esp32_ros_camera
Add ROS-based camera image transfer support to ESP32.

## Hardware
[ESP32-CAM](https://docs.platformio.org/en/latest/boards/espressif32/esp32cam.html)

## Environment
PlatformIO on VS Code

## Network setting
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
Modify `\lib\ros_lib\ros\node_handle.h` to increase buffer size:

```
/* Node Handle */
template<class Hardware,
         int MAX_SUBSCRIBERS = 25,
         int MAX_PUBLISHERS = 25,
         int INPUT_SIZE = 1024,
         int OUTPUT_SIZE = 65536>
```