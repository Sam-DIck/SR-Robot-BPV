from RobotControl import *
import time

R=RobotController(
    left_motor  = MotorMapping(motor_board="srABC1",motor_index=0),
    right_motor = MotorMapping(motor_board="srABC1",motor_index=1),

    wheel_base=0.5,
    camera_displacement=Vec3(0,0.07,0),
    speed_power_ratio=1.25,

    vel_PID=PID(0,kp=-0.01,ki=0,kd=0),
    ang_PID=PID(0,kp=-0.05,ki=0,kd=0),
    
    target_dt=0.1
)


def Program():
    from math import sin,pi
    yield R.sleep(1)
    R.set_relative(speed=0.1)
    for i in range(100):
        s = sin(i*pi/2)
        print(s)
        R.set_power(left_motor=0.3*s,right_motor=0.3*s)
        yield R.sleep(1)
    yield None

R.run(Program)
print("complete")