from RobotControl import *


R=RobotController(
    left_motor  = MotorMapping(motor_board="srABC1",motor_index=0),
    right_motor = MotorMapping(motor_board="srABC1",motor_index=1),

    wheel_base=0.5,
    camera_displacement=Vec3(0,0.07,0),
    speed_power_ratio=1.25,

    vel_PID=PID(0,kp=-0.1,ki=0,kd=0),
    ang_PID=PID(0,kp=0.05,ki=0,kd=0)
)

R.wait_start()
st = R._robot.time()
while R._robot.time()-st<5:
    R.step()
print("started")
R.set_relative(
    speed=0.1,
)

st = R._robot.time()
R.sleep(0.1)
with open('data.csv','w+') as f:
    string = 'time,derived,tar ang vel,tar speed,ang vel,velX,velY,rot,posX,posY,motorL,motorR\n'
    f.write(string)
while R._robot.time()-st<200:
    R.step()
    with open('data.csv','a') as f:
        string = f'{R._robot.time()-st},{1 if R.using_derived else 0},{R._tar_ang_vel},{R._tar_speed},{R._ang_vel},{R._vel.x},{R._vel.y},{R._rot},{R._pos.x},{R._pos.y},{R.motorL.power},{R.motorR.power}'
        for m in R.markers:
            string+=f',{m.id},{R.get_marker_position(m).x},{R.get_marker_position(m).y}'
        f.write(
            string+'\n'
        )
        
    R.sleep(0.1)
R.stop()
R.sleep(1)

