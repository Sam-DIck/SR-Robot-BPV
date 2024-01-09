from RobotControl import *
from math import pi

R=RobotController(
    left_motor  = MotorMapping(motor_board="srABC1",motor_index=0),
    right_motor = MotorMapping(motor_board="srABC1",motor_index=1),

    wheel_base=0.5,
    camera_displacement=Vec3(0,0.07,0),
    speed_power_ratio=1.25,

    vel_PID=PID(0,kp=-0.1,ki=0,kd=0),
    ang_PID=PID(0,kp=-0.05,ki=0,kd=0),
    
    target_dt=0.1
)


def Program():
    st = R.time()
    R.set_relative(ang_vel=pi/10)
    yield R.sleep(20)

R.run(Program)
print("complete")