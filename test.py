# test.py

#from threading import Thread
from threading import Thread
from time import time
from typing import Iterator
from numpy import iterable



q = 0
def Update(dt):
    global q
    q += 1

def update_loop(program,target_dt:float=0):
    pause_time=0
    s = time()
    lt = s
    programIter = program()
    while time()-s<150:
        t = time()
        dt = t-lt
        lt=t
        pause_time -= dt
        if pause_time<0:
            pause_time = next(programIter)
            print(pause_time)
        if pause_time is None:
            break

        Update(dt)
        while time()-t<target_dt:
            _ = sum([i**0.5 for i in range(1000)])

    print("update_loop complete")





def start(*args,**kwargs):
    thread =  Thread(None,update_loop,args=args,kwargs=kwargs)
    thread.start()
    return thread

def sleep(seconds):
    return seconds

def step():
    return 0


def Program():
    from math import sin,pi
    yield sleep(5)
    lt = time()
    for i in range(20):
        yield sleep(1)
    yield None

start(Program)