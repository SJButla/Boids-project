import numpy as np
import pygame
import random

width, height = 900,650
pygame.init()
screen= pygame.display.set_mode((width,height)) #initialising pygame display
clock = pygame.time.Clock()

class Agent:
    def __init__(self,x,y, speed):
        self.position=np.array([x,y], dtype=np.float64) # x is in x direction, y is in y direction but down (inverted)
        self.velocity=np.array([random.uniform(-2,2), random.uniform(-2,2)], dtype=np.float64) #initialising random velocity between -2 and 2
        self.speed=speed #speed constant thats just multiplied to the velocity

    def update(self):
        self.position+=self.velocity*self.speed #move current direction heading

        if self.position[0]<=0:  # checks if agent is on y boundary
            self.position[0]=0
            self.velocity[0]*=-1 #reverses velocity
        elif self.position[0]>=width: #checks if agent is on x boundary
            self.position[0]=width
            self.velocity[0]*=-1
        elif self.position[1]<=0: #checks if agent is on y boundary
            self.position[1]=0
            self.velocity[1]*=-1 #reverses velocity
        elif self.position[1]>height:  #checks if agent is on y boundary
            self.position[1]=height
            self.velocity[1]*=-1 #reverses velocity

    def draw(self, screen):
            # Draw the agent as a white circle with a direction line
            pos = self.position.astype(int)
            pygame.draw.circle(screen, (255, 255, 255), pos, 4)
            # Draw line showing direction
            end_pos = pos + (self.velocity * 5)
            pygame.draw.line(screen, (255, 0, 0), pos, end_pos, 2)

agents=[]
for x in range(100): #creating agents through the class and adding them to array
    x=random.randint(0,width)
    y=random.randint(0,height)
    speed=4
    new_agent=Agent(x,y,speed)
    agents.append(new_agent)

running = True
while running: #while loop to just run pygame environment and add objects to screen
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    screen.fill((0, 0, 0))
    
    for agent in agents:
        agent.update()
        agent.draw(screen)
    
    pygame.display.flip()
    clock.tick(60) #fps

pygame.quit()