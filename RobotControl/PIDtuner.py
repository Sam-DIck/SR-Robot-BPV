from PID import PID
from random import random,uniform
import matplotlib.pyplot as plt 
from time import sleep

WHEEL_BASE=0.5
SPEED_POWER_RATIO=1.25
TARGET_V=1
TARGET_R=1

def get_speed_prediction(l,r)->float:
    return power_to_speed((l+r)/2)

def get_ang_vel_prediction(l,r)->float:
    if l==r: return 0
    radius = WHEEL_BASE*(l + r) / (2*(l-r))
    if radius==0: return 0
    return get_speed_prediction(l,r) / radius

def speed_to_power(speed:float)->float:
    return speed / SPEED_POWER_RATIO

def power_to_speed(power:float)->float:
    return power * SPEED_POWER_RATIO


class member:
    kpV:float
    kiV:float
    kdV:float
    pidV:PID
    pidR:PID
    mean_squared:float=0
    def __init__(self,/,kpV=None,kiV=None,kdV=None,kpR=None,kiR=None,kdR=None):
        if kpV is None:
            self.kpV=random()
        else:
            self.kpV=kpV
        if kiV is None:
            self.kiV=random()
        else:
            self.kiV=kiV
        if kdV is None:
            self.kdV=random()
        else:
            self.kdV=kdV
        
        if kpR is None:
            self.kpR=random()
        else:
            self.kpR=kpR
        if kiR is None:
            self.kiR=random()
        else:
            self.kiR=kiR
        if kdR is None:
            self.kdR=random()
        else:
            self.kdR=kdR
        
        self.pidV=PID(0,kp=self.kpV,ki=self.kiV,kd=self.kdV)
        self.pidV.set_target(TARGET_V)
        self.pidR=PID(0,kp=self.kpR,ki=self.kiR,kd=self.kdR)
        self.pidR.set_target(TARGET_R)
        
    
    lm:float=0
    rm:float=0

def mixMember(a:member,b:member)->member:
    z=uniform(-0.5,1.5)
    kpV=z*a.kpV+(1-z)*b.kpV
    z=uniform(-0.5,1.5)
    kiV=z*a.kiV+(1-z)*b.kiV
    z=uniform(-0.5,1.5)
    kdV=z*a.kdV+(1-z)*b.kdV

    z=uniform(-0.5,1.5)
    kpR=z*a.kpR+(1-z)*b.kpR
    z=uniform(-0.5,1.5)
    kiR=z*a.kiR+(1-z)*b.kiR
    z=uniform(-0.5,1.5)
    kdR=z*a.kdR+(1-z)*b.kdR

    return member(
        kpV=kpV,
        kiV=kiV,
        kdV=kdV,

        kpR=kpR,
        kiR=kiR,
        kdR=kdR
    )


population:list[member] = [member() for _ in range(900)]

data:list[float]=[]


DT=0.1
# plt.ion()
# figure, ax = plt.subplots(figsize=(10000, 8))
# line1, = ax.plot(range(len(data)), data)
best:member=None
while True:
    try:
        for i in range(1000):
            for m in population:
                vel=get_speed_prediction(m.lm,m.rm)*(random()*0.1+0.95)
                v=m.pidV.calc_strength(vel,dt=DT)
                m.lm+=v
                m.rm+=v
                m.lm=min(1,max(-1,m.lm))
                m.rm=min(1,max(-1,m.rm))
                err=TARGET_V-vel
                m.mean_squared+=err**2
                # print(m.mean_squared)
        
        Max = sorted(population,key=lambda x: x.mean_squared)[0].mean_squared
        survivors=sorted(population,key=lambda x: x.mean_squared)[:30]

        population=[]
        for a in survivors:
            for b in survivors:
                population.append(mixMember(a,b))
        
        best=survivors[0]
        print(best.mean_squared**0.5)
    except KeyboardInterrupt:
        break
print(f'{m.kdV=}\t{m.kiV=}\t{m.kpV=}')
print(f'{m.kdR=}\t{m.kiR=}\t{m.kpR=}')

