from sr.robot3 import *
import sr.robot3 as sr
from math import pi,sqrt,atan2,sin,cos
from modules.PID import PID
from modules.Util import angle_between,display_power,calc_distance,calc_signed_distance

R = Robot()

wall_markers = [
    (2153,-2875),
    (1438,-2850),
    (718,-2850),
    (0,-2850),
    (-718,-2850),
    (-1438,-2850),
    (-2153,-2850),

    (-2875, 2153),
    (-2875, 1438),
    (-2875, 718),
    (-2875, 0),
    (-2875, -718),
    (-2875, -1438),
    (-2875, -2153),

    (-2153,2875),
    (-1438,2850),
    (-718,2850),
    (0,2850),
    (718,2850),
    (1438,2850),
    (2153,2850),

    (2875, 2153),
    (2875, 1438),
    (2875, 718),
    (2875, 0),
    (2875, -718),
    (2875, -1438),
    (2875, -2153)
]


leftMotor = R.motor_boards["srABC1"].motors[0]
rightMotor = R.motor_boards["srABC1"].motors[1]

print("start")

def get_position_old(_markers:list):
    markers = [m for m in _markers if m.id<28]
    if len(markers)>=2:
        pair = (None,None)
        best = 1e8
        for mA in markers:
            for mB in [m for m in markers if m != mA]:
                angle_dif = mA.position.horizontal_angle - mB.position.horizontal_angle
                if abs(abs(angle_dif)-pi/2)<=best-pi/2:
                    pair = (mA,mB)
                    best = abs(angle_dif)

        pA,pB = wall_markers[pair[0].id],wall_markers[pair[1].id]
        dA = pair[0].position.distance
        dB = pair[1].position.distance

        def get_intersections(x0, y0, r0, x1, y1, r1):
            
            
            # circle 1: (x0, y0), radius r0
            # circle 2: (x1, y1), radius r1

            d=sqrt((x1-x0)**2 + (y1-y0)**2)
            
            # non intersecting
            if d > r0 + r1 :
                return None
            # One circle within other
            if d < abs(r0-r1):
                return None
            # coincident circles
            if d == 0 and r0 == r1:
                return None
            else:
                a=(r0**2-r1**2+d**2)/(2*d)
                h=sqrt(r0**2-a**2)
                x2=x0+a*(x1-x0)/d   
                y2=y0+a*(y1-y0)/d   
                x3=x2+h*(y1-y0)/d     
                y3=y2-h*(x1-x0)/d 

                x4=x2-h*(y1-y0)/d
                y4=y2+h*(x1-x0)/d
                
                return ((x3, y3), (x4, y4))

        intersections = get_intersections(pA[0],pA[1],dA,pB[0],pB[1],dB)
        
        if intersections is None:
            return None
        if intersections[0]==intersections[1]:
            return (round(intersections[0][0]),round(intersections[0][1]))
        else:
            
            intA = (pA[0]-intersections[0][0])*(pB[0]-intersections[0][0]) + (pA[1]-intersections[0][1])*(pB[1]-intersections[0][1])
            intB = (pA[0]-intersections[0][0])*(pB[0]-intersections[1][0]) + (pA[1]-intersections[0][1])*(pB[1]-intersections[1][1])

            if -2875<intersections[0][0]<2875 and -2875<intersections[0][1]<2875:
                return (round(intersections[0][0]),round(intersections[0][1]))
            else:
                return (round(intersections[1][0]),round(intersections[1][1]))
    else:
        return None

def get_rotation(_markers:list):
    markers = [m for m in _markers if m.id<28]
    if len(markers)==0: return None
    total = 0
    for m in markers:
        ang:float
        if 0<=m.id<7:
            ang=m.orientation.yaw-pi/2
        elif 7<=m.id<14:
            ang=m.orientation.yaw+pi
        elif 14<=m.id<21:
            ang=m.orientation.yaw+pi/2
        elif 21<=m.id<28:
            ang=m.orientation.yaw
        total += (ang + pi) % (2 * pi) - pi
    
    return round((total / len(markers))%(2*pi)-pi,4)

def get_position_new(_markers:list):
    markers = [m for m in _markers if m.id<28]
    if len(markers)==0: return None
    tx,ty = 0,0
    for m in markers:
        ang:float
        if 0<=m.id<7:
            ang=m.orientation.yaw-pi/2
        elif 7<=m.id<14:
            ang=m.orientation.yaw+pi
        elif 14<=m.id<21:
            ang=m.orientation.yaw+pi/2
        elif 21<=m.id<28:
            ang=m.orientation.yaw
        ang = (ang + pi) % (2 * pi) - pi
        mpos = wall_markers[m.id]
        px=mpos[0] - m.position.distance*cos(ang)
        py=mpos[1] - m.position.distance*sin(ang)
        tx+=px
        ty+=py
    return tx/len(markers),ty/len(markers)

def get_position_combined(_markers:list)->tuple[int,int]|None:
    markers = [m for m in _markers if m.id<28]
    if len(markers)==0:
        return None
    elif len(markers) == 1:
        m = markers[0]
        ang:float
        if 0<=m.id<7:
            ang=m.orientation.yaw-pi/2
        elif 7<=m.id<14:
            ang=m.orientation.yaw+pi
        elif 14<=m.id<21:
            ang=m.orientation.yaw+pi/2
        elif 21<=m.id<28:
            ang=m.orientation.yaw
        ang = (ang + pi) % (2 * pi) - pi
        mpos = wall_markers[m.id]
        px=mpos[0] - m.position.distance*cos(ang)
        py=mpos[1] - m.position.distance*sin(ang)
        return round(px),round(py)
    elif len(markers)>=2:
        pair = (None,None)
        best = 1e8
        for mA in markers:
            for mB in [m for m in markers if m != mA]:
                angle_dif = mA.position.horizontal_angle - mB.position.horizontal_angle
                if abs(abs(angle_dif)-pi/2)<=best-pi/2:
                    pair = (mA,mB)
                    best = abs(angle_dif)

        pA,pB = wall_markers[pair[0].id],wall_markers[pair[1].id]
        dA = pair[0].position.distance
        dB = pair[1].position.distance

        def get_intersections(x0, y0, r0, x1, y1, r1):
            
            
            # circle 1: (x0, y0), radius r0
            # circle 2: (x1, y1), radius r1

            d=sqrt((x1-x0)**2 + (y1-y0)**2)
            
            # non intersecting
            if d > r0 + r1 :
                return None
            # One circle within other
            if d < abs(r0-r1):
                return None
            # coincident circles
            if d == 0 and r0 == r1:
                return None
            else:
                a=(r0**2-r1**2+d**2)/(2*d)
                h=sqrt(r0**2-a**2)
                x2=x0+a*(x1-x0)/d   
                y2=y0+a*(y1-y0)/d   
                x3=x2+h*(y1-y0)/d     
                y3=y2-h*(x1-x0)/d 

                x4=x2-h*(y1-y0)/d
                y4=y2+h*(x1-x0)/d
                
                return ((x3, y3), (x4, y4))

        intersections = get_intersections(pA[0],pA[1],dA,pB[0],pB[1],dB)
        
        if intersections is None:
            return None
        if intersections[0]==intersections[1]:
            return (round(intersections[0][0]),round(intersections[0][1]))
        else:
            
            intA = (pA[0]-intersections[0][0])*(pB[0]-intersections[0][0]) + (pA[1]-intersections[0][1])*(pB[1]-intersections[0][1])
            intB = (pA[0]-intersections[0][0])*(pB[0]-intersections[1][0]) + (pA[1]-intersections[0][1])*(pB[1]-intersections[1][1])

            if -2875<intersections[0][0]<2875 and -2875<intersections[0][1]<2875:
                return (round(intersections[0][0]),round(intersections[0][1]))
            else:
                return (round(intersections[1][0]),round(intersections[1][1]))

def get_distance_sensor():
    return R.arduino.pins[A4].analog_read()

def set_motor(left:float,right:float)->None:
    if abs(left)<0.01: left=0
    if abs(right)<0.01: right=0
    leftMotor.power = min(max(left,-1),1)
    rightMotor.power = min(max(right,-1),1)

leftMotor.power = 0
rightMotor.power = 0

lost = True

loc = None
last_loc = loc
rot = None
last_rot=rot
first_lost_time = -1
tar_rot = 0
tar_pos = None#(0,0)


speed = 0.1
angle_pid = PID(initial_value=0,kp=5,ki=0,kd=0)
distance_pid = PID(initial_value=0,kp=0.0005,ki=0.0001,kd=0)
last_time = R.time()
delta_time=1/100

COLLECT_ASTEROID=0
RETURN_TO_PLANET=1

state=COLLECT_ASTEROID

def Mainloop():
    if tar_pos is None:
        set_motor(0,0)
        return False
    angle = atan2(loc[1]-tar_pos[1],loc[0]-tar_pos[0])
    distance = calc_signed_distance(loc[0],loc[1],tar_pos[0],tar_pos[1],rot)
    if distance<0:
        angle *= -1
    angle_pid.set_target(angle_between(angle,rot))
    turn = angle_pid.calc_strength(0,dt=delta_time)
    speed = distance_pid.calc_strength(distance,dt=delta_time)
    speed = min(max(speed,-0.6),0.6)
    # if -1000<distance<100:
        # turn=0
    set_motor(-turn/(8*pi)+speed,
              turn/(8*pi)+speed)
    
    return calc_distance(loc[0],loc[1],tar_pos[0],tar_pos[1]) < 100


while True: 
    time = R.time()
    dt=last_time-time
    last_time=time

    markers=R.camera.see()
    walls = [m for m in markers if 0<=m.id<=27]
    asteroids = [m for m in markers if 150<=m.id<=199]

    result = False


    if lost :
        if first_lost_time==-1 or first_lost_time is None:
            first_lost_time=R.time()
        elif R.time()-first_lost_time>1:
            set_motor(-0.2,-0.1)
    else:
        first_lost_time=-1
    if loc is None or rot is None:
        first_lost_time-=1
    else:
        
        result = Mainloop()

        if state==COLLECT_ASTEROID and len(asteroids)>0:
            closest=sorted(asteroids,key=lambda m:m.position.distance)[0]
            px=loc[0]+cos(closest.position.horizontal_angle+rot)*closest.position.distance
            py=loc[1]-sin(closest.position.horizontal_angle+rot)*closest.position.distance
            tar_pos = (round(px),round(py))
    
    if state == COLLECT_ASTEROID:
        if get_distance_sensor()<0.2:
            R.servo_board.servos[0].position = 0.9
            R.servo_board.servos[1].position = 0.9
            R.sleep(0.3)
    elif state==RETURN_TO_PLANET:
        tar_pos=(0,-2200)


    last_loc = loc
    loc = get_position_combined(markers)
    last_rot = rot
    rot = get_rotation(markers)
    lost = loc is None or rot is None
    #print(f'arrived={result}\t{lost=}\t{loc=}\t{last_loc=}\t{rot=}\t{last_rot=}\tlost_time={R.time()-first_lost_time}')
    print(f'l={display_power(leftMotor.power)}\t\tr={display_power(rightMotor.power)}',end='\t\t')
    print(f'{loc=}\t\t{rot=}',end='\t\t')
    print(f'')
    R.sleep(0.01)
