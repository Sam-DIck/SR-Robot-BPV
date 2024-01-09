from sr.robot3 import Robot
from sr.robot3.motor_devices import Motor # type: ignore
from sr.robot3.camera import Marker
from test import start
from .vector import Vec3
from .PID import PID
from math import sin,cos,tan,pi

FILTERING_ALPHA=1


WAITING_FOR_START=1
STATIONARY=2
DRIVING=4
USING_DERIVED=8


DRIVING_MODE_NONE=0
DRIVING_MODE_RELATIVE=1
DRIVING_MODE_ABSOLUTE=2
DRIVING_MODE_POWER=3

WALL_MARKER_POSIIONS = [
    (2.154 ,-2.875),
    (1.436 ,-2.875),
    (.718  ,-2.875),
    (0     ,-2.875),
    (-.718 ,-2.875),
    (-1.436,-2.875),
    (-2.154,-2.875),

    (-2.875, 2.154),
    (-2.875, 1.436),
    (-2.875, 0.718),
    (-2.875, 0    ),
    (-2.875,-0.718),
    (-2.875,-1.436),
    (-2.875,-2.154),

    (-2.154, 2.875),
    (-1.435, 2.875),
    (-.718 , 2.875),
    (0     , 2.875),
    (.718  , 2.875),
    (1.436 , 2.875),
    (2.154 , 2.875),

    (2.875 ,  2.152),
    (2.875 ,  1.436),
    (2.875 ,  0.718),
    (2.875 ,  0    ),
    (2.875 , -0.718),
    (2.875 , -1.436),
    (2.875 , -2.154)
]

# DEBUG: ############################################################
from time import time
__debug_start_time = time()
def Print(*args, **kwargs):
    '''for Debug purposes, adds time infront of message and prints'''
    t= time()-__debug_start_time
    sec = round(t%60,3)
    min = round((t/60))
    # print(f"{min}:{sec}\t| ",*args, **kwargs)

class InvalidMethodCallException(Exception):
    def __init__(self,method_name:str,/,before:bool=False):
        super().__init__(f"'{method_name}' can only be called {'before' if before  else 'after'} the process is started.")

class MotorMapping:
    '''Maps a motor reference to its hardware connections'''
    motor_board:str
    motor_index:int
    def __init__(self,/,motor_board:str,motor_index:int)->None:
        if motor_index not in (0,1):
            raise ValueError("'motor_index' must have a value of either 0 or 1")
        self.motor_index=motor_index
        self.motor_board=motor_board

class RobotController:
    # Configuration
    _robot:Robot=Robot()
    _motorL:MotorMapping = MotorMapping(motor_board="",motor_index=0)
    _motorR:MotorMapping = MotorMapping(motor_board="",motor_index=1)

    _wheel_base:float=0.5
    _camera_displacement:Vec3=Vec3()
    _speed_power_ratio:float=1.25

    _target_dt:float=0

    #PID
    _PID_speed:PID=PID(0,kp=0,ki=0,kd=0)
    _PID_angvel:PID=PID(0,kp=0,ki=0,kd=0)

    # kinematics
    _pos:Vec3=None # type: ignore
    _vel:Vec3=None # type: ignore
    _rot:float=None # type: ignore
    _ang_vel:float=None # type: ignore
    _lt:float=_robot.time()
    
    # status
    _waiting_for_start:bool=True
    _using_derived:bool=False
    _driving_mode:int=DRIVING_MODE_NONE

    # generated
    
    _markers:list[Marker]|None=None

    # control targets
    _tar_powerL:float=0
    _tar_powerR:float=0

    _tar_speed:float=0
    _tar_ang_vel:float=0


    def __init__(self,/,left_motor:MotorMapping,
                 right_motor:MotorMapping,
                 wheel_base:float,
                 camera_displacement:Vec3,
                 speed_power_ratio:float,
                 vel_PID:PID,
                 ang_PID:PID,
                 target_dt:float=0.01)->None:
        self._motorL=left_motor
        self._motorR=right_motor
        self._wheel_base=wheel_base
        self._camera_displacement=camera_displacement
        self._speed_power_ratio=speed_power_ratio
        self._PID_speed=vel_PID
        self._PID_angvel=ang_PID
        self.motorL.power=0
        self.motorR.power=0
        self._target_dt=target_dt

    @property
    def waiting_for_start(self)->bool:
        return self._waiting_for_start
    @property
    def using_derived(self)->bool:
        return self._using_derived
    @property
    def stationary(self)->bool:
        return self.speed<1e-3
    @property
    def driving(self)->bool:
        return self._driving_mode!=DRIVING_MODE_NONE
    @property
    def status_flags(self)->int:
        flag = WAITING_FOR_START&self._waiting_for_start
        flag += STATIONARY&self.stationary
        flag += DRIVING&self.driving
        flag += USING_DERIVED&self._using_derived
        return flag
    @property
    def speed(self)->float:
        return self._vel.magnitude
    @property
    def signed_speed(self)->float:
        return self._vel.dot(Vec3.from_angle(self._rot))
    @property
    def motorL(self)->Motor:
        return self._robot.motor_boards[self._motorL.motor_board].motors[self._motorL.motor_index]
    @property
    def motorR(self)->Motor:
        return self._robot.motor_boards[self._motorR.motor_board].motors[self._motorR.motor_index]
    @property
    def markers(self)->list[Marker]:
        '''Partially Functional: returns a list of markers visible in the camera
        view and caches the result (this is reset when the robot moves)'''
        if self._markers is None:
            m = stopwatch(self._robot.camera.see)
            self._markers = sorted(m,key=lambda x:x.id)
        return self._markers
    @property
    def wall_markers(self)->list[Marker]:
        '''returns markers filtered to only include the wall markers'''
        return [m for m in self.markers if 0<=m.id<28]
    @property
    def asteroid_markers(self)->list[Marker]:
        '''returns markers filtered to only include the asteroids'''
        return [m for m in self.markers if 150<=m.id<=199]
    
    def _step(self,dt)->None:
        '''private method: performs internal calculations relating to motors, vision...'''
        if dt <=0:
            print(f"Invalid timestep({dt}s)")
            dt = max(0.01,dt)
        if self.waiting_for_start:
            raise InvalidMethodCallException('step')
        self._markers=None

        rot = stopwatch(self.get_rotation)
        if self._rot is None:
            if rot is not None:
                self._rot = rot
            else:
                self._rot=0
        
        if rot is None:
            rot = self.get_rot_prediction(dt)
        self._ang_vel = (rot-self._rot)/dt


        pos = stopwatch(self.get_position)
        
        self._using_derived=False

        if self._pos is None:
            self._vel = Vec3()
            if pos is not None:
                self._pos=pos
            else:
                self._pos=Vec3()
        if pos is None:
            pos = self.get_pos_prediction(dt)
            self._using_derived=True
        else:
            pos = FILTERING_ALPHA*pos + (1-FILTERING_ALPHA) * self.get_pos_prediction(dt,use_true=True)
        
        self._vel = (pos-self._pos)/dt
        self._pos=pos
        self._rot=rot
        powerL = self.motorL.power
        powerR = self.motorR.power

        if self._driving_mode==DRIVING_MODE_NONE:
            self.motorL.power = 0
            self.motorR.power = 0
        elif self._driving_mode==DRIVING_MODE_RELATIVE:
            self._PID_speed.set_target(self._tar_speed)
            print(f'{self._rot}')
            print(f'{self._vel=}')
            print(f'forward={Vec3.from_angle(self._rot)}')
            print(f'{self.speed=}')
            print(f'{self.signed_speed=}')
            v = self._PID_speed.calc_strength(self.signed_speed,dt=dt)
            
            self._PID_angvel.set_target(self._tar_ang_vel)
            a = self._PID_angvel.calc_strength(self._ang_vel,dt=dt)
            if self._tar_speed > 0:
                v = min(1,max(0,v))
            elif self._tar_speed < 0:
                v = min(0,max(-1,v))
            powerL = v-a
            powerR = v+a
            
            
        elif self._driving_mode==DRIVING_MODE_ABSOLUTE:
            pass
        elif self._driving_mode==DRIVING_MODE_POWER:
            powerL = self._tar_powerL
            powerR = self._tar_powerR
        
        
        self.motorL.power = min(1,max(-1,powerL))
        self.motorR.power = min(1,max(-1,powerR))
        Print("_step: complete")

    def set_power(self,/,left_motor:float|None=None,right_motor:float|None=None)->None:
        ''' Program command: diectly set the motor power'''
        if left_motor is not None:
            self._tar_powerL = left_motor
            self._driving_mode=DRIVING_MODE_POWER
        if right_motor is not None:
            self._tar_powerR=right_motor
            self._driving_mode=DRIVING_MODE_POWER
    
    def set_relative(self,/,speed:float=0,ang_vel:float|None=None)->None:
        ''' Program Command: set the target speed and angular velocity for the robot'''
        self._driving_mode=DRIVING_MODE_RELATIVE
        self._tar_speed = speed
        if ang_vel is not None:
            self._tar_ang_vel=ang_vel
        else:
            self._tar_ang_vel=0
        ang_vel=self._tar_ang_vel
        if ang_vel!=0:
            radius=speed/ang_vel
            self.motorL.power = self.speed_to_power(ang_vel*(radius - self._wheel_base/2))
            self.motorR.power = self.speed_to_power(ang_vel*(radius + self._wheel_base/2))
        else:
            self.motorL.power = self.speed_to_power(speed)
            self.motorR.power = self.speed_to_power(speed)
        
    def get_marker_position(self,marker:Marker)->Vec3:
        '''gets the Arena space position of the provided Marker'''
        return self._pos - Vec3(
                                marker.position.distance*cos(self._rot-marker.position.horizontal_angle)/1000,
                                marker.position.distance*sin(self._rot-marker.position.horizontal_angle)/1000,
                                0
                          )

    def stop(self)->None:
        ''' Program command: stop the robot from moving'''
        self.motorL.power=0
        self.motorR.power=0
        self._driving_mode=DRIVING_MODE_NONE
    
    def get_position(self,/)->Vec3|None:
        ''' uses wall markers visible to the robot's camera to determine its location adjucsts for the camera offset'''
        pos = self._get_position()
        if pos is None:
            return None
        else:
            co_X=self._camera_displacement.x
            co_Y=self._camera_displacement.y
            offset = Vec3(co_X*cos(self._rot)-co_Y*sin(self._rot),co_X*sin(self._rot)+co_Y*cos(self._rot),0)
            return pos+offset

    def _get_position(self)->Vec3|None:
        '''private method: uses wall markers visible to the robot's camera to determine its location'''
        ang = self._rot
        best = 0
        pair:tuple[Marker,Marker]|None = None
        markers = self.wall_markers
        for mA in markers:
            for mB in [m for m in markers if m.id < mA.id]:
                angle_dif = mA.position.horizontal_angle - mB.position.horizontal_angle
                if abs(abs(angle_dif)-pi/2)<=abs(best-pi/2):
                    pair = (mA,mB)
                    best = abs(angle_dif)
        if pair != None:
            theta1 = pair[0].orientation.yaw - pair[0].position.horizontal_angle
            if 0<=pair[0].id<7:
                theta1+=-pi/2
            elif 7<=pair[0].id<14:
                theta1+=pi
            elif 14<=pair[0].id<21:
                theta1+=pi/2
            elif 21<=pair[0].id<28:
                theta1+=0
            theta2 = pair[1].orientation.yaw - pair[1].position.horizontal_angle
            if 0<=pair[1].id<7:
                theta2+=-pi/2
            elif 7<=pair[1].id<14:
                theta2+=pi
            elif 14<=pair[1].id<21:
                theta2+=pi/2
            elif 21<=pair[1].id<28:
                theta2+=0


            m1x,m1y=WALL_MARKER_POSIIONS[pair[0].id]
            m2x,m2y=WALL_MARKER_POSIIONS[pair[1].id]

            x = (tan(theta2)*m2x - tan(theta1)*m1x+m1y-m2y)/(tan(theta2)-tan(theta1))
            y = m1y-tan(theta1)*x-tan(theta1)*m1x
            return Vec3(x=x,y=y,z=0)

    def get_rotation(self)->float|None:
        ''' uses wall markers visible to the robot's camera to determine its rotation'''
        markers = self.wall_markers
        if len(markers)==0: return None
        total = 0
        for m in markers:
            ang:float = 0
            if 0<=m.id<7:
                ang=m.orientation.yaw+pi/2
            elif 7<=m.id<14:
                ang=m.orientation.yaw+pi
            elif 14<=m.id<21:
                ang=m.orientation.yaw-pi/2
            elif 21<=m.id<28:
                ang=m.orientation.yaw
            total += (ang + pi) % (2 * pi) - pi
        return round((total / len(markers))%(2*pi)-pi,4)

    def get_speed_prediction(self)->float:
        '''uses the motor speed to predict the robot speed'''
        return self.power_to_speed((self.motorL.power + self.motorL.power)/2)

    def get_ang_vel_prediction(self)->float:
        ''' uses the motor speed to predict the robot angular velocity '''
        L=self.motorL.power
        R=self.motorR.power
        if R==L: return 0
        radius = self._wheel_base*(L + R) / (2*(L-R))
        if radius==0: return 0
        return self.get_speed_prediction() / radius
    
    def get_pos_prediction(self,dt,/,use_true:bool=False)->Vec3:
        av = self.get_ang_vel_prediction()
        vel = self.get_speed_prediction()
        if use_true:
            if self._ang_vel is not None:
                av = self._ang_vel
            if self._vel is not None:
                vel = self._vel.magnitude
        rot = self._rot + av * dt
        if av != 0:
            radius = vel / av
            cx=self._pos.x + radius * cos(self._rot+pi/2)
            cy=self._pos.y + radius * sin(self._rot+pi/2)
            px=cx + radius * sin(rot)
            py=cy - radius * cos(rot)
            return Vec3(px,py,0)
        
        else:
            return self._pos + Vec3.from_angle(self._rot) * vel
    
    def get_rot_prediction(self,dt,/,use_true:bool=False)->float:
        av=self.get_ang_vel_prediction()
        if use_true:
            if self._ang_vel is not None:
                av=self._ang_vel
        return self._rot + av*dt
    
    def speed_to_power(self,speed:float)->float:
        return speed / self._speed_power_ratio
    
    def power_to_speed(self,power:float)->float:
        return power * self._speed_power_ratio
    
    ################
    # Flow Control #

    def time(self)->float:
        return self._robot.time()

    def _processing_control_loop(self, program):
        '''private method: the motor control loop'''
        start_time=self.time()
        lt=start_time
        programIter=program()
        pause_time=0
        running = True
        while self.time()-start_time<150 and running:
            t = self.time()
            dt=t-lt
            lt=t
            pause_time -= dt
            try:
                if pause_time<0:
                    Print("_processing_control_loop : executing user code")
                    pause_time=stopwatch(next,programIter)
                if pause_time is None:
                    running = False
            except StopIteration:
                running = False

            
            stopwatch(self._step,dt)
            Print("beging wait for DT")
            with open('data.csv','a') as f:
                string = f'{self._robot.time()-start_time},{1 if self.using_derived else 0},{self._tar_ang_vel},{self._tar_speed},{self._ang_vel},{self._vel.x},{self._vel.y},{self._rot},{self._pos.x},{self._pos.y},{self.motorL.power},{self.motorR.power},'
                for m in self.markers:
                    string+=f',{m.id},{self.get_marker_position(m).x},{self.get_marker_position(m).y}'
                f.write(
                    string+'\n'
                )
            elapsed = self.time()-t
            self._robot.sleep(self._target_dt+t-self.time())
            Print("wait for DT complete")
        self.motorL.power=0
        self.motorR.power=0
        
    def run(self, Program)->None:
        '''starts the execution of the motor control loop and the program logic
        Program is a generator that yields a delay interval'''
        if self.waiting_for_start:
            self._robot.wait_start()
            self._waiting_for_start=False
            with open('data.csv','w+') as f:
                string = 'time,derived,tar ang vel,tar speed,ang vel,velX,velY,rot,posX,posY,motorL,motorR\n'
                f.write(string)
            self._processing_control_loop(program=Program)
        else:
            raise InvalidMethodCallException('run',before=True)
        
    def sleep(self,pause):
        '''Program Command: halts program for <pause> seconds'''
        return pause

    def step(self):
        '''Program Command: halts program until the next execution step'''
        return 0


def stopwatch(function, *args, **kwargs):
    from time import time
    sw = time()
    result = function(*args, **kwargs)
    elapsed = time()-sw
    fullName = str(function)
    fullName=fullName.strip('<>')
    if ' of <' in fullName:
        name = fullName.split(' of <')[0]
    else: name = fullName
    name = name.split(' ')[2].ljust(30)
    # print(f'{name}: {elapsed=}')
    return result