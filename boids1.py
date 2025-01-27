import numpy as np
import pygame
import random
import math

width, height = 900,700
pygame.init()
screen= pygame.display.set_mode((width,height)) #initialising pygame display
clock = pygame.time.Clock()

class Agent:
    def __init__(self,x,y, speed):
        self.position=np.array([x,y], dtype=np.float64) # x is in x direction, y is in y direction but down (inverted)
        self.velocity=np.array([random.uniform(-2,2), random.uniform(-2,2)], dtype=np.float64) #initialising random velocity between -2 and 2
        self.speed=speed #speed constant thats just multiplied to the velocity
    
        
    def get_heading_angle(self):
        heading_angle=math.degrees(math.atan2(self.velocity[1], self.velocity[0])) #calculates heading angle through velocity coordinates
        normalised_angle = heading_angle %360
        return normalised_angle
    
              
    def get_nearest_neighbour(self, agents):
        def euclidean_distance(agent1, agent2): #calculates direct euclidean distance
            x1= agent1.position[0]-agent2.position[0]  
            y1= agent1.position[1]-agent2.position[1]
            return math.sqrt((x1*x1)+(y1*y1))
        nearest_neighbour=None
        smallest_distance=float(100000000000000)
        for i in agents: #loops through all agents and compares current to every other checking distance
            if i!=self:
                current_distance = euclidean_distance(self, i)
                if current_distance < smallest_distance:
                        nearest_neighbour = i #if distance is smaller than current euclidean distance, its set to the smallest
                        smallest_distance = current_distance
        return nearest_neighbour, smallest_distance
        
    def viewable(self):
        nearest_neighbour, euclidean_distance= self.get_nearest_neighbour(agents)
        if euclidean_distance<100:
            diff_x = nearest_neighbour.position[0] - self.position[0] #calculate difference between x position
            diff_y = nearest_neighbour.position[1] - self.position[1] #calculates angle between y positions
            angle = math.degrees(math.atan2(diff_y, diff_x)) #uses atan2 function to calculate the angle between the two points
            relative_angle = (angle - self.get_heading_angle()) % 360 #modulates angle to 360 degrees and subtracts heading angle 
            if relative_angle>=135 and angle<=225: #as facing up is 90 degrees in the function, the blindspot is between angles 135 and 225
                return True #returns true if in blindpot
            else:
                return False
                
    def alignment(self):
        '''self is your current boid
        need to check for boids, if viewable, if in radius, add to list
        then you take all the boids within list, calculate avg velocity
        apply to boids
        '''  
            
    def separation(self, agents):
        if self.viewable():
            neighbour, euclidian_distance = self.get_nearest_neighbour(agents)
            separation_threshold = 100  # distance separaiton constant
            if neighbour and euclidian_distance < separation_threshold: 
                repulsion = self.position - neighbour.position   # create a repulsion vector pointing away from the nearest neighbor
                if euclidian_distance > 0:
                    max_repulsion = 1.0 # limits the maximum magnitude of repulsion
                    repulsion_magnitude = min(max_repulsion, (separation_threshold - euclidian_distance) / separation_threshold) #calculates how strongly to repel based off proximity
                    repulsion_direction = repulsion / euclidian_distance #converts to unit vector
                    self.velocity += repulsion_direction * repulsion_magnitude # appliesrepulsion
                
            # Limis overall velocity
        max_velocity = 5.0  
        velocity_magnitude = np.linalg.norm(self.velocity) #prevents agents from accelerating too harshly
        if velocity_magnitude > max_velocity:
            self.velocity = (self.velocity / velocity_magnitude) * max_velocity #scales velocity down if too high but preserves direction
            
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
for x in range(30): #creating agents through the class and adding them to array
    x=random.randint(0,width)
    y=random.randint(0,height)
    speed=0.5
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
        agent.separation(agents)
        agent.draw(screen)
    
    pygame.display.flip()
    clock.tick(60) #fps

pygame.quit()