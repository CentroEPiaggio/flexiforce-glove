# Flexiforce Glove for the Pisa/IIT SoftHand

This is a [FlexiForce](http://www.tekscan.com/store/flexiforce-sensors.html)/[Arduino]()-based glove designed for the Pisa/IIT SoftHand (and eventually for human hands). You might find useful the Pisa/IIT SoftHand package available here [pisa-iit-soft-hand](https://github.com/CentroEPiaggio/pisa-iit-soft-hand) to visualize the hand. You need a microUSB cable to connect the arduino to the PC

## How to upload the firmware

It is set up to work with Arduino micro board.

Build the firmware with standard `catkin_make`, or specifically with:

`roscd && cd .. && catkin_make flexiforce_glove_firmware_flexiforce`

And upload it (for Arduino Micro)
 1. Push the reset button, and the led start blinking
 2. Type `catkin_make flexiforce_glove_firmware_flexiforce-upload` before the led stops blinking, otherwise repeat the operation

Note: Check that the arduino board connects to `/dev/ttyACM0`, you can use `cd /dev && ls | grep ttyACM`. If not, you need to change the `firmware/CMakeLists.txt` file as well as `launch/flexiforce_glove.launch` and compile again to account for the new port. Provide privileges with `sudo chmod 777 /dev/ttyACM0`.

Tip: if you have `sudo` credentials on you PC, add your user to the `dialout` group to avoid sudoing by running:
`sudo adduser YOUR_USER dialout`

Calibrate the glove by modifying the values on the [`flexiforce_setup.yaml`](config/`flexiforce_setup.yaml`), and run:

`roslaunch flexiforce_glove flexiforce_glove.launch`

Current topics are:

 - `/flexiforce/raw_values`

 - `/flexiforce/calibrated_values`

 - `/flexiforce/joint_states`

 - `/flexiforce/

Check the result with `rostopic echo /flexiforce/joint_states`, or any other topic.

## ToDo's

- [ ] Calibration method to obtain gain and offset for each sensor automatically
- [ ] Validate that the number of sensor in yaml file matches the size of the raw data
- [ ] Generalize for n-sensors in the firmware
- [ ] Add visualization to the [`flexiforce_glove.launch`](launch/flexiforce_glove.launch`) file

