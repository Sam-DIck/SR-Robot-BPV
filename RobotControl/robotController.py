from sr.robot3 import Robot
from sr.robot3.motor_devices import Motor # type: ignore
from sr.robot3.camera import Marker
from .vector import Vec3
from .PID import PID
from .Util import display_power
from math import sin,cos,tan,sqrt,pi

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



class InvalidMethodCallException(Exception):
    def __init__(self,method_name:str,/,before:bool=False):

        super().__init__(f"'{method_name}' can only be called {'before' if before  else 'after'} the process is started.")

class MotorMapping:
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
    _status:int=WAITING_FOR_START
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
                 ang_PID:PID)->None:
        self._motorL=left_motor
        self._motorR=right_motor
        self._wheel_base=wheel_base
        self._camera_displacement=camera_displacement
        self._speed_power_ratio=speed_power_ratio
        self._PID_speed=vel_PID
        self._PID_angvel=ang_PID
        self.motorL.power=0
        self.motorR.power=0

    @property
    def waiting_for_start(self)->bool:
        return bool(WAITING_FOR_START&self._status)
    @property
    def using_derived(self)->bool:
        return bool(USING_DERIVED&self._status)
    @property
    def speed(self)->float:
        return self._vel.dot(Vec3.from_angle(self._rot))
    @property
    def motorL(self)->Motor:
        return self._robot.motor_boards[self._motorL.motor_board].motors[self._motorL.motor_index]
    @property
    def motorR(self)->Motor:
        return self._robot.motor_boards[self._motorR.motor_board].motors[self._motorR.motor_index]
    @property
    def markers(self)->list[Marker]:
        if self._markers is None:
            self._markers = sorted(self._robot.camera.see(),key=lambda x:x.id)
        return self._markers
    @property
    def wall_markers(self)->list[Marker]:
        return [m for m in self.markers if 0<=m.id<28]
    @property
    def asteroid_markers(self)->list[Marker]:
        return [m for m in self.markers if 150<=m.id<=199]
    
    def wait_start(self)->None:
        if self.waiting_for_start:
            self._robot.wait_start()
            self._status^=WAITING_FOR_START

        else:
            raise InvalidMethodCallException('wait_start',before=True)
    
    def step(self)->None:
        if self.waiting_for_start:
            raise InvalidMethodCallException('step')
        self._markers=None
        t = self._robot.time()
        dt=t-self._lt
        self._lt=t

        rot = self.get_rotation()
        if self._rot is None:
            if rot is not None:
                self._rot = rot
            else:
                self._rot=0
        if rot is None:
            rot = self.get_rot_prediction(dt)
        self._ang_vel = (rot-self._rot)/dt

        pos = self.get_position()
        if pos is None:
            pos = self.get_position()
        
        self._status&= ~USING_DERIVED

        if self._pos is None:
            self._vel = Vec3()
            if pos is not None:
                self._pos=pos
            else:
                self._pos=Vec3()
        if pos is None:
            pos = self.get_pos_prediction(dt)
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
            v = self._PID_speed.calc_strength(self.speed,dt=dt)
            
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

    def set_power(self,/,left_motor:float|None=None,right_motor:float|None=None)->None:
        if left_motor is not None:
            self._tar_powerL = left_motor
            self._driving_mode=DRIVING_MODE_POWER
        if right_motor is not None:
            self._tar_powerR=right_motor
            self._driving_mode=DRIVING_MODE_POWER
    
    def set_relative(self,/,speed:float=0,ang_vel:float|None=None)->None:
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
        return self._pos - Vec3(
                                marker.position.distance*cos(self._rot-marker.position.horizontal_angle)/1000,
                                marker.position.distance*sin(self._rot-marker.position.horizontal_angle)/1000,
                                0
                          )

    def stop(self)->None:
        self.set_power(
            left_motor=0,
            right_motor=0
        )
        self._driving_mode=DRIVING_MODE_NONE
    
    def sleep(self,time:float)->None:
        self._robot.sleep(time)

    def get_position(self,/)->Vec3|None:
        pos = self._get_position()
        if pos is None:
            return None
        else:
            co_X=self._camera_displacement.x
            co_Y=self._camera_displacement.y
            offset = Vec3(co_X*cos(self._rot)-co_Y*sin(self._rot),co_X*sin(self._rot)+co_Y*cos(self._rot),0)
            return pos+offset

    def _get_position(self)->Vec3|None:
        ang = self._rot
        best = 0
        pair:tuple[Marker,Marker]|None = None
        for mA in self.wall_markers:
            for mB in [m for m in self.wall_markers if m != mA]:
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
        markers = self.wall_markers
        if len(markers)==0: return None
        total = 0
        for m in markers:
            ang:float = 0
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

    def get_speed_prediction(self)->float:
        return self.power_to_speed((self.motorL.power + self.motorL.power)/2)

    def get_ang_vel_prediction(self)->float:
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