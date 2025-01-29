import numpy as np
import pygame
import random
import math

width, height = 1500,900
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
   
    def euclidean_distance(agent1, agent2): #calculates direct euclidean distance
        x1= agent1.position[0]-agent2.position[0]  
        y1= agent1.position[1]-agent2.position[1]
        return math.sqrt((x1*x1)+(y1*y1))
    
        
    def get_near_neighbours(self): #gets all neighbours in a predefined radius
        neighbours=[]
        radius=50 #sets radius constant to 50
        for agent in agents: #loops through all agents in simulation
            if agent!=self: #checks theres no comparison to self
                euclidean_dist= self.euclidean_distance(agent) #calculates the euclidean distance between two agents
                if euclidean_dist< radius: #checks if the distance lies wihthin predefined radius
                    neighbours.append(agent) #adds to neighbours list
        return neighbours    

    def viewable(self): #returns a list of all near neighbours that are viewable
        neighbours=self.get_near_neighbours()
        viewable=[]
        for nearest in neighbours:
            diff_x = nearest.position[0] - self.position[0] #calculate difference between x position
            diff_y = nearest.position[1] - self.position[1] #calculates angle between y positions
            angle = math.degrees(math.atan2(diff_y, diff_x)) #uses atan2 function to calculate the angle between the two points
            relative_angle = (angle - self.get_heading_angle()) % 360 #modulates angle to 360 degrees and subtracts heading angle 
            if not(relative_angle>=135 and angle<=225): #as facing up is 90 degrees in the function, the blindspot is between angles 135 and 225
                viewable.append(nearest)
        return viewable
                
    def alignment(self):
        alignment_list=self.viewable() #gets near neighbours and then returns which ones are viewable
        if not alignment_list: #if no neighbours in list, return
            return
        x_total=0
        y_total=0
        for i in alignment_list: 
            x_total+=i.velocity[0]
            y_total+=i.velocity[1]
      
        average_velocity=np.array([x_total/len(alignment_list),y_total/len(alignment_list)], dtype=np.float64) #creates an average velocity of all viewable neighbours      
        difference_force=average_velocity-self.velocity #calculates the difference between current velocity and average       
        max_force = 0.5
        steering_magnitude = np.linalg.norm(difference_force) #computes magnitude of difference vector
        if steering_magnitude > max_force:
            difference_force = (difference_force / steering_magnitude) * max_force #normalises difference force
        
        self.velocity += difference_force # apply the steering force to current velocity
        speed = np.linalg.norm(self.velocity) # normalise velocity to maintain speed
        if speed > 0:
            desired_speed = 2.0 
            self.velocity = (self.velocity / speed) * desired_speed #applies steering force to velocity  
                
    def separation(self, agents):
        viewable=self.viewable()
        separation_threshold = 25  # distance separaiton constant
        original_velocity = self.velocity.copy()
        for neighbour in viewable:
            euclidean_dist= self.euclidean_distance(neighbour)
            if euclidean_dist < separation_threshold:
                repulsion = self.position - neighbour.position   # create a repulsion vector pointing away from the nearest neighbour
                if euclidean_dist > 0:
                    max_repulsion = 1.0 # limits the maximum magnitude of repulsion
                    repulsion_magnitude = min(max_repulsion, (separation_threshold - euclidean_dist) / separation_threshold) #calculates how strongly to repel based off proximity
                    repulsion_direction = repulsion / euclidean_dist #converts to unit vector
                    self.velocity += repulsion_direction * repulsion_magnitude # appliesrepulsion
                
        # limis overall velocity
        current_speed = np.linalg.norm(self.velocity)
        if current_speed > 0:  # prevent division by zero
            original_speed = np.linalg.norm(original_velocity)
            self.velocity = (self.velocity / current_speed) * original_speed    
        
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
        # Draw the agent as a white circle
        pos = self.position.astype(int)
        pygame.draw.circle(screen, (255, 255, 255), pos, 4)
        
        # Draw line showing direction - increase the scaling factor
        direction_length = 15  # Increased from 5 to 15
        normalized_velocity = self.velocity / np.linalg.norm(self.velocity) if np.linalg.norm(self.velocity) > 0 else self.velocity
        end_pos = pos + (normalized_velocity * direction_length)
        pygame.draw.line(screen, (255, 0, 0), pos, end_pos.astype(int), 2)

agents=[]
for x in range(100): #creating agents through the class and adding them to array
    x=random.randint(0,width)
    y=random.randint(0,height)
    speed=2
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
        agent.alignment()
        agent.draw(screen)
    
    pygame.display.flip()
    clock.tick(60) #fps

pygame.quit()