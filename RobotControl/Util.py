from math import pi,floor,cos,sin
def angle_between(angle1:float, angle2:float)->float:
    angle1 = (angle1 + pi) % (2 * pi) - pi
    angle2 = (angle2 + pi) % (2 * pi) - pi
    signed_angle = angle2 - angle1
    if signed_angle > pi:
        signed_angle -= 2 * pi
    elif signed_angle <= -pi:
        signed_angle += 2 * pi
    return signed_angle

def calc_distance(x1:float,y1:float,x2:float,y2:float)->float:
    return ((x1-x2)**2+(y1-y2)**2)**0.5

def calc_signed_distance(x1:float,y1:float,x2:float,y2:float,R:float)->float:
    return (x1-x2)*cos(R)+(y1-y2)*sin(R)

def power_to_str(power:float,width:int=20)->str:
    p = abs(power)
    w = round(p*width)
    string = '#'*w + '-'*(width-w)
    if power<0:
        string = string[::-1]
    return '|'+ string + f'| {power:.2%}'