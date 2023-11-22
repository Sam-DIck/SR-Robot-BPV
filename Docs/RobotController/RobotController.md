# RobotController
## Constructor
### [RobotController](#RobotController-Ctor)
&emsp;Creates a RobotController
## Properties
### RobotController._robot 
&emsp;Direct access to the [sr.robot3.Robot](https://studentrobotics.org/docs/programming/robot_api/)
### RobotController.[waiting_for_start](#RobotController-waiting_for_start)
&emsp;Whether the robot has been started
### RobotController.[using_derived](#RobotController-using_derived)
&emsp;Whether the robot is using derived position and rotation
### RobotController.[speed](#RobotController-speed)
&emsp;The signed speed of the robot
### RobotController.[markers](#RobotController-markers)
&emsp;A list of markers visible to the robot's camera
### RobotController.wall_markers
&emsp;A list of markers designated as wall markers visible to the camera
### RobotController.asteroid_markers
&emsp;A list of all markers designated as asteroid markers visible to the camera
## Methods
### RobotController.wait_start
&emsp;Pause code execution until the start button has been pressed
### RobotController.step
&emsp;Perform all calculations related to vision and movement
### RobotController.set_power
&emsp;Directly set motor power for the left and right drive motors
### RobotController.set_relative
&emsp;Set target speed and angular velocity which the robot tries to achieve
### RobotController.set_absolute
&emsp;Set target position and rotation which the robot tries to reach
### RobotController.stop
&emsp;Set motor power to 0
### RobotController.sleep
&emsp;Pause code execution for a specified amount of time
### RobotController.get_marker_position
&emsp;Get the position of the specified marker 
### RobotController.speed_to_power
&emsp;Convert from wheel speed to motor power
### RobotController.power_to_speed
&emsp;Convert from motor power to wheel speed