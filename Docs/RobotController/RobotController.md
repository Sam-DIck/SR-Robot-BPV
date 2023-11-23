# RobotController
## Constructor
### [RobotController](../RobotController/RobotController-Ctor.md)
&emsp;Creates a RobotController
## Properties
### RobotController._robot 
&emsp;Direct access to the [sr.robot3.Robot](https://studentrobotics.org/docs/programming/robot_api/)
### RobotController.[waiting_for_start](../RobotController/RobotController-waiting_for_start.md)
&emsp;Whether the robot has been started
### RobotController.[using_derived](../RobotController/RobotController-using_derived.md)
&emsp;Whether the robot is using derived position and rotation
### RobotController.[speed](../RobotController/RobotController-speed.md)
&emsp;The signed speed of the robot
### RobotController.[markers](../RobotController/RobotController-markers.md)
&emsp;A list of markers visible to the robot's camera
### RobotController.[wall_markers](../RobotController/RobotController-wall_markers.md)
&emsp;A list of markers designated as wall markers visible to the camera
### RobotController.[asteroid_markers](../RobotController/RobotController-asteroid_markers.md)
&emsp;A list of all markers designated as asteroid markers visible to the camera
## Methods
### RobotController.[wait_start](../RobotController/RobotController-wait_start.md)
&emsp;Pause code execution until the start button has been pressed
### RobotController.[step](../RobotController/RobotController-step.md)
&emsp;Perform all calculations related to vision and movement
### RobotController.[set_power](../RobotController/RobotController-set_power.md)
&emsp;Directly set motor power for the left and right drive motors
### RobotController.[set_relative](../RobotController/RobotController-set_relative.md)
&emsp;Set target speed and angular velocity which the robot tries to achieve
### RobotController.[set_absolute](../RobotController/RobotController-set_absolute.md)
&emsp;Set target position and rotation which the robot tries to reach
### RobotController.[stop](../RobotController/RobotController-stop.md)
&emsp;Set motor power to 0
### RobotController.[sleep](../RobotController/RobotController-sleep.md)
&emsp;Pause code execution for a specified amount of time
### RobotController.[get_marker_position](../RobotController/RobotController-get_marker_position.md)
&emsp;Get the position of the specified marker 
### RobotController.[speed_to_power](../RobotController/RobotController-speed_to_power.md)
&emsp;Convert from wheel speed to motor power
### RobotController.[power_to_speed](../RobotController/RobotController-power_to_speed.md)
&emsp;Convert from motor power to wheel speed