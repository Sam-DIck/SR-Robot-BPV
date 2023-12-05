import pygame
from math import sin,cos,pi
from time import sleep

pygame.init()

height=575
width=575
surface = pygame.display.set_mode((width,height))
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24) # type: ignore
running=True
# Data:list[tuple[float,float,float,float,tuple[float,float],float,tuple[float,float],float,float]] = []

def pos_to_screen(x,y):
    return (int(x*100+287.5),int(287.5-y*100))


keys:list=[]

paused=False
frame = 1
Data = []
while running:
    ### vvv user Input vvv
    for event in pygame.event.get():  
        if event.type == pygame.QUIT:  
           running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            paused = not paused
        
        if event.type==pygame.KEYDOWN:
            if event.key not in keys:
                keys.append(event.key)
        if event.type==pygame.KEYUP:
            if event.key in keys:
                keys.remove(event.key)

    if pygame.K_LEFT in keys:
        if pygame.K_LSHIFT in keys:
            frame = 0
        else:
            frame -=1
        paused = True
    if pygame.K_RIGHT in keys:
        if pygame.K_LSHIFT in keys:
            frame = len(Data)
        else:
            frame += 1
        if frame == len(Data):
            paused=False

    mouse_pos = pygame.mouse.get_pos()
    ### ^^^ User Input ^^^



    try:
        with open('data.csv','r') as f:
            lines = f.read().split('\n')
            if len(lines)<=2:
                continue
            Data = [[float(dataPoint) for dataPoint in dataEntry.split(',')] for dataEntry in lines[1:-1]]
    except IOError:
        pass
    if len(Data)==0:
        continue
    if not paused:
        frame +=1
    frame = max(0,min(frame,len(Data)-1))
    data = Data[frame]
    time = data[0]
    derived = bool(data[1])
    tar_ang_vel = data[2]
    tar_speed = data[3]
    ang_vel = data[4]
    velX = data[5]
    velY = data[6]
    rot = data[7]
    posX = data[8]
    posY = data[9]
    motorL = data[10]
    motorR = data[11]
    markers = [(data[i],data[i+1],data[i+2]) for i in range(12,len(data),3)]
    surface.fill((0,0,0))
    pos = pos_to_screen(posX,posY)
    p1 = pos[0] + 10*cos(pi-rot),    pos[1] + 10*sin(pi-rot)
    p2 = pos[0] + 10*cos(pi-rot+2.5),pos[1] + 10*sin(pi-rot+2.5)
    p3 = pos[0] + 10*cos(pi-rot-2.5),pos[1] + 10*sin(pi-rot-2.5)
    col = (255,0,0) if not derived else (100,0,0)
    pygame.draw.polygon(surface,col,[p1,p2,p3])
    # pygame.draw.circle(surface,col,pos,10)
    pygame.draw.line(surface,(0,255,0),pos,pos_to_screen(posX+velX,posY+velY))
    if len(Data)>=2:
        pygame.draw.lines(surface,(255,255,0),False,[pos_to_screen(d[8],d[9]) for d in Data])
    shown = None
    for id,x,y in markers:
        m_pos = pos_to_screen(x,y)
        pygame.draw.circle(surface,(0,0,255),m_pos,10)
        dist = (m_pos[0]-mouse_pos[0])**2 + (m_pos[1]-mouse_pos[1])**2
        if dist < 100:
            shown = (id,x,y)
    T = font.render(f'Robot| px={data[8]:.3f} py={data[9]:.3f} vx={velX:.3f} vy={velY:.3f}',True,(255,255,255))
    surface.blit(T,(20,535))
    if shown is not None:
        id,x,y = shown
        T = font.render(f'{id=}| {x=:.3f} {y=:.3f}',True,(255,255,255))
        surface.blit(T,(20,555))
        
    pygame.display.flip()

    if frame>0:
        pygame.time.wait(int(1000*(Data[frame][0]-Data[frame-1][0])))