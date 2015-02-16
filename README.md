# Flexiforce Glove for the Pisa/IIT SoftHand

This is a [FlexiForce](http://www.tekscan.com/store/flexiforce-sensors.html)/[Arduino]()-based glove designed for the Pisa/IIT SoftHand (and eventually for human hands). You might find useful the Pisa/IIT SoftHand package available here [pisa-iit-soft-hand](https://github.com/CentroEPiaggio/pisa-iit-soft-hand) to visualize the hand.

## How to upload the firmware

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

Current topics are:

 - `/flexiforce/raw_values`

 - `/flexiforce/calibrated_values`

 - `/flexiforce/joint_states`

 - `/flexiforce/

## Test

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

## ToDo's

- [ ] Calibration method to obtain gain and offset for each sensor automatically
- [ ] Validate that the number of sensor in yaml file matches the size of the raw data
- [ ] Generalize for n-sensors in the firmware

