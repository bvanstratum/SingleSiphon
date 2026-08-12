# Session Notes - 2026-04-27

## What's working
- colcon build succeeds
- Launch file brings up micro-ROS agent + teleop node in xterm
- ESP32 publishes dummy sine wave current data at 500ms
- Teleop node publishes to actuator_1/freq and actuator_2/freq
- set up a separate esp32 to read debug statements on Serial1 since microROS seems to cause instability with that
- battery has been tested 
- 
## Next steps
- [ ] Add Float32 subscriber to ESP32 for actuator_1/freq and actuator_2/freq
      - Remember to bump executor handle count to 3
- [ ] Test full loop: keypress → ROS topic → ESP32 callback
- [ ] Replace dummy sine wave with real actuator PWM output
- [ ] Set up PlotJuggler layout and save to ~/singleSiphon/config/demo.xml
- [ ] Consider 1kHz logging with time sync (we discussed the pattern)
- [ ] set up a joystick for controlling the motor
- [x] fix frequency shifting issue
- [ ] add debug terminal to the lauch file as an option
- [ ] test encoder
- [ ] test the motor pwm pin
- [ ] test the current logging


## Key commands
- Build:   cd ~/singleSiphon && colcon build --packages-select singleSiphon
- Source:  source ~/singleSiphon/install/setup.bash
- Launch:  ros2 launch singleSiphon frequencyControlDemo.py
- Monitor: ros2 topic echo /actuator_1/freq
           ros2 topic echo /micro_ros/motor_current
