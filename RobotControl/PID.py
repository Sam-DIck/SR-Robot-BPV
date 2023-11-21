class PID:
    _error:float=0
    _integral:float=0
    _derivative:float=0

    kp:float
    ki:float
    kd:float

    _target:float

    def __init__(self,/,initial_value:float,*,kp:float=0,ki:float=0,kd:float=0)->None:
        '''kp: the coefficient of the proportional term
        ki: the coefficient of the integral term
        kd: the coefficient of the derivative term'''
        self._error=0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._target = initial_value

    def set_target(self,/, value:float)->None:
        self._target=value

    def calc_strength(self,/,value:float,*,dt:float=1)->float:
        '''value: the current value of the tracked variable'''
        
        last_err = self._error 
        self._error = value-self._target
        self._integral += self._error*dt
        self._derivative = (self._error - last_err)/dt
        

        sp = self._error*self.kp
        si = self._integral*self.ki
        sd = self._derivative*self.kd

        return sp+si+sd






if __name__ =="__main__":
    from random import uniform
    from sys import stdout
    import matplotlib.pyplot as plt
    dt=1/60
    data:list[tuple[float,float]] = [(0,0)]
    
    t=0
    p=0
    v=0
    pid = PID(initial_value=p,
              kp=0,ki=0,kd=-1)
    pid.set_target(1)
    commands=[0 for _ in range(0)]
    for _ in range(1000):
        commands.append(pid.calc_strength(p,dt=dt))
        a = commands[0]
        commands=commands[1:]
        for _ in range(10):
            ra=a*uniform(0.99,1.01)
            v += ra*(dt/10)
            p += 1/2*ra*(dt/10)**2*uniform(0.99,1.01)
            t+=(dt/10)
            data.append((p,t))
        #stdout.write(f'\r{p=:.4f}\t{v=:.4f}\t{a=:.4f}')

    P=[dp[0] for dp in data]
    T=[dp[1] for dp in data]
    plt.plot(T,P)
    plt.show()
