# teleop_pkg

## DESCRIPTION
This ROS package works with teleop_app (https://github.com/aitronik/teleop_app). It manages different Android devices on which app is installed, receiving data from device via bluetooth and publishing on ROS topics.

__If different devices want to connect to the same platform on which thi node is running, remember to assign them an increasing bluetooth channel number on app, starting from channel 1__

For each connected device this pkg runs an istance of deviceManager class to handle its data. The collected data will be publish on following topics:

1. `/status`, that report the actual device status depending on wich mode has been selected on app

2. `/joystickCmd`, for manual control. Normalized axis values from app joystick widget or device accelerometers are published [-1, 1]

3. `/gps`, where current device latitude and longitude are published

4. `/orientation`, where current device roll, pitch and yaw angles are published

Since each deviceManager instance publishes on these topics, each message has the device id stored on `msg.header.frame_id` in order to identify "who publish what". Only status message has id encoded in an uint8_t bitfield structured as follow:

 * bit 0:   joystick/Accelerometers on/off
 * bit 1:   gps/orientation         on/off
 * bit 2-5: 4-bit device ID (1->15)
 * bit 6-7  TBD

## REQUIREMENTS

1. ROS
2. bluetooth library. It can be installed running: `sudo apt install libbluetooth-dev`

