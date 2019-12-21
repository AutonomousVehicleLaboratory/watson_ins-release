# watson_ins

This is a ROS node for the Watson DMS-SGP02.

## Building

This package follows the standard ROS build conventions.

Create a new workspace
```bash
mkdir -p catkin_ws/src
```

Clone the source
```bash
cd catkin_ws/src
git clone https://github.com/AutonomousVehicleLaboratory/watson_ins.git
cd ..
```

Build the source
```bash
catkin init
catkin build
```

## Running

```
rosrun watson_ins watson_ins_node
```

## Node Information

### Default Parameters

| Parameter | Value | Description |
| --------- | ------ | ----------- |
| SerialPath | /dev/ttyUSB0 | Path of serial port |
| SerialBaud | 9600 | Serial baud rate |
| SerialTimeout | 1000 | Serial port timeout, in milliseconds |

### Subscribers

None

### Publishers

| Publisher | Function | Message Type |
| --------- | -------- | ------------ |
| /imu_raw | Publishes IMU data | sensor_msgs::Imu |
| /fix | Publishes GPS data | sensor_msgs::NavSatFix |

### Supported Watson DMS-SGP02 Driver Configuration

| Field                 | Field Width (Bytes) |
|-----------------------|---------------------|
| Data Type             | 2                   |
| Time Stamp            | 9                   |
| Roll                  | 7                   |
| Pitch                 | 6                   |
| Yaw                   | 6                   |
| X Acceleration        | 6                   |
| Y Acceleration        | 6                   |
| Z Acceleration        | 6                   |
| Forward Acceleration  | 6                   |
| Lateral Acceleration  | 6                   |
| Vertical Acceleration | 6                   |
| X Angle Rate          | 6                   |
| Y Angle Rate          | 6                   |
| Z Angle Rate          | 6                   |
| Heading Rate          | 6                   |
| Forward Velocity      | 7                   |
| Latitude              | 10                  |
| Longitude             | 11                  |
| Altitude              | 5                   |

### Supporting other Watson output configuration

By default, this ROS node supports the fields mentioned in the previous
section. If support for different driver configurations is desired, please see
`include/watson_ins_driver.h`.
