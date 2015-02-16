Flexiforce glove
================

This is a glove based on [FlexiForce](http://www.tekscan.com/store/flexiforce-sensors.html) by Tekscan and Arduino, in this case micro. 

It is meant ot be used for the Pisa/IIT Soft hand (and eventually for human hands). You might find useful to use the Pisa/IIT SoftHand package available here [pisa-iit-soft-hand](https://github.com/CentroEPiaggio/pisa-iit-soft-hand) to visualize the hand.

1. Flexiforce-based glove
-------------------------

It is set up to work with Arduino micro board.

Build the firmware with standard `catkin_make`, or specifically with:

`catkin_make flexiforce_glove_firmware_flexiforce`

And upload the firmware with (for Arduino Micro you should do it by pushing the reset button and upload before the arduino enters in run mode, look at the leds):

`catkin_make flexiforce_glove_firmware_flexiforce-upload`

Note: Check that the arduino board connects to `/dev/ttyACM0`, otherwise you need to change the `firmware/CMakeLists.txt` file as well as `launch/flexiforce_glove.launch`, you will se the PORT label, and provide privileges with `sudo chmod 777 /dev/ttyACM0`.

Tip: add your user to the `dialout` group to avoid sudoing by running:

`sudo adduser YOUR_USER dialout`

Finally, set properly the `flexiforce_setup.yaml` (instructions within), and run:

`roslaunch flexiforce_glove flexiforce_glove.launch`

Current topics:

 - `/flexiforce/raw_values`

 - `/flexiforce/calibrated_values`

 - `/flexiforce/joint_states`

Test with a message as if it comes from the board (you need to tell that the board is not connect in the launch file), and then publish on `/flexiforce/flexiforce_raw_values` with:

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

Check the result with `rostopic echo /flexiforce/joint_states`, or any other topic.

ToDO: calibration method to obtain gain and offset for each sensor automatically.
ToDO: check that the number of sensor in yaml file matches the size of the raw data.
