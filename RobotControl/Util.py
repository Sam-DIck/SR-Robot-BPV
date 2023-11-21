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

def display_power(power:float)->str:
    res = 10
    gradient='$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\|()1\{\}[]?-_+~<>i!lI;:,"^`\'.'[-1:0:-1]
    p = abs(power)
    string=''
    for i in range(res):
        v = floor(min(max(p*res-i,0),1)*(len(gradient)-1))
        #print(v)
        string+=gradient[v]

    if power < 0:
        string = string[-1:0:-1]
    return string + f'  {power:.0%}'

def calc_distance(x1:float,y1:float,x2:float,y2:float)->float:
    return ((x1-x2)**2+(y1-y2)**2)**0.5

def calc_signed_distance(x1:float,y1:float,x2:float,y2:float,R:float)->float:
    return (x1-x2)*cos(R)+(y1-y2)*sin(R)

if __name__ == '__main__':
    for i in range(-100,101):
        print(display_power(i/100))