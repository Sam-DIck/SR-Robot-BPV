# [RobotController](./RobotController.md) Constructor

## Declaration
RobotController(<br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;left_motor:[MotorMapping](../MotorMapping/MotorMapping.md), right_motor:[MotorMapping](../MotorMapping/MotorMapping.md), <br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;wheel_base:float, camera_displacement:[Vec3](../Vec3/Vec3.md), speed_power_ratio:float,<br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;vel_PID:[PID](../PID/PID.md), ang_PID:[PID](../PID/PID.md))<br>
)
## Description
Creates a RobotController
```
RobotController(
    left_motor = MotorMapping(motor_board = "srABC1", motor_index = 0),
    right_motor = MotorMapping(motor_board = "srABC1",motor_index = 1),
    wheel_base = 0.5,
    camera_displacement = Vec3(0.0, 0.07, 0.0),
    vel_PID = PID(kd=1,ki=1,kp=1),
    ang_PID = PID(kd=1,ki=1,kp=1)
)
 ```