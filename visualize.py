import pygame

pygame.init()

height=575
width=575
surface = pygame.display.set_mode((width,height))
running=True
# Data:list[tuple[float,float,float,float,tuple[float,float],float,tuple[float,float],float,float]] = []

def pos_to_screen(x,y):
    return (int(x*100+287.5),int(287.5-y*100))

while running:
    for event in pygame.event.get():  
        if event.type == pygame.QUIT:  
           running = False

    try:
        with open('data.csv','r') as f:
            lines = f.read().split('\n')
            if len(lines)<=2:
                continue
            Data = [[float(dataPoint) for dataPoint in dataEntry.split(',')] for dataEntry in lines[1:-1]]
            data = Data[-1]
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
            col = (255,0,0) if not derived else (100,0,0)
            pygame.draw.circle(surface,col,pos,10)
            pygame.draw.line(surface,(0,255,0),pos,pos_to_screen(posX+velX,posY+velY))
            if len(Data)>=2:
                pygame.draw.lines(surface,(255,255,0),False,[pos_to_screen(d[8],d[9]) for d in Data])
            for id,x,y in markers:
                pygame.draw.circle(surface,(0,0,255),pos_to_screen(x,y),10)
            pygame.display.flip()
    except IOError:
        pass