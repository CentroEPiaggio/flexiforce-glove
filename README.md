sensor-gloves
=============

Several types of sensorized gloves developed for the Pisa/IIT Soft hand (eventually with human hands).

You need the Pisa/IIT soft hand packages available here [pisa-iit-soft-hand](https://github.com/CentroEPiaggio/pisa-iit-soft-hand).

1. Flexiforce-based glove
-------------------------

It is set up to work with Arduino micro board.

Build/upload the firmware with standard `catkin_make`, or specifically with:

`catkin_make sensor_gloves_firmware_flexiforce`

And upload the firmware with:

`catkin_make sensor_gloves_firmware_flexiforce-upload`

Note: Check that the arduino board connects to `/dev/ttyACM0`, otherwise you must to change the `firmware/CMakeLists.txt` file, you will se the PORT label.

Note: add your user to the `dialout` group to avoid sudoing by running:

`sudo adduser YOUR_USER dialout`

Finally, run in your PC:

`roslaunch sensor_gloves flexiforce_glove.launch`

Current topics:

 - `/flexiforce/flexiforce_raw_values`

 - `/flexiforce/flexiforce_calibrated_values`

 - `/flexiforce/joint_states`

Test emulating an arduino message (i.e. without an arduino board connect, you need to comment the usb connection in the launch file) by publishing on `/flexiforce/flexiforce_raw_values` as:

```
rostopic pub /flexiforce/flexiforce_raw_values std_msgs/Int16MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
- 0
- 0
- 0
- 0
- 0
- 0
- 0" 
```

ToDO: calibration method to obtain gain and offset for each sensor automatically.