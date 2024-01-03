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