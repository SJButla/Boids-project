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
        self.time_offset = random.uniform(0, 2 * math.pi)  
        
    def get_heading_angle(self):
        heading_angle=math.degrees(math.atan2(self.velocity[1], self.velocity[0])) #calculates heading angle through velocity coordinates
        normalised_angle = heading_angle %360
        return normalised_angle      
   
    def euclidean_distance(self, agent2): #calculates direct euclidean distance
        x1= self.position[0]-agent2.position[0]  
        y1= self.position[1]-agent2.position[1]
        return math.sqrt((x1*x1)+(y1*y1))
    
        
    def get_near_neighbours(self): #gets all neighbours in a predefined radius
        neighbours=[]
        radius=40 #sets radius constant 
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
                
        # limits overall velocity
        current_speed = np.linalg.norm(self.velocity)
        if current_speed > 0:  # prevent division by zero
            original_speed = np.linalg.norm(original_velocity)
            self.velocity = (self.velocity / current_speed) * original_speed    
        
        max_velocity = 5.0  
        velocity_magnitude = np.linalg.norm(self.velocity) #prevents agents from accelerating too harshly
        if velocity_magnitude > max_velocity:
            self.velocity = (self.velocity / velocity_magnitude) * max_velocity #scales velocity down if too high but preserves direction
    
    
    def cohesion(self):
        cohesion_list=self.viewable() #gets viewable neighbours in specified radius
        if not cohesion_list:
            return
        x_total=0
        y_total=0
        for neighbour in cohesion_list:
            x_total+=neighbour.position[0]
            y_total+=neighbour.position[1]
        
        average_position=np.array([x_total/len(cohesion_list),y_total/len(cohesion_list)], dtype=np.float64) 
        position_difference = average_position - self.position       
        distance = np.linalg.norm(position_difference)
       
        if distance > 0:
            desired_velocity = (position_difference / distance) 
        else:
            desired_velocity = np.array([0.0, 0.0])  # No movement if we're at the target

        # Calculate steering force (desired change in velocity)
        difference_force = desired_velocity - self.velocity
        steering_magnitude = np.linalg.norm(difference_force)
        max_force = 0.03

        # Limit the steering force
        if steering_magnitude > max_force:
            difference_force = (difference_force / steering_magnitude) * max_force

        # Apply steering force to current velocity
        self.velocity += difference_force

        # Maintain consistent speed
        speed = np.linalg.norm(self.velocity)
        if speed > 0:
            desired_speed = 2.0
            self.velocity = (self.velocity / speed) * desired_speed
            
       
    def apply_random_movement(self):
        current_time = pygame.time.get_ticks() * 0.001  #gets time in milliseconds
        oscillation_x = math.sin(current_time + self.time_offset) * 0.01
        oscillation_y = math.cos(current_time * 1.3 + self.time_offset) * 0.01
        random_force = np.array([oscillation_x, oscillation_y])
        self.velocity += random_force
        
    
    def obstacle_avoidance(self, obstacles):
        perception_radius = 50  
        max_avoidance_force = 3.0
        for obstacle in obstacles:
            distance_obstacle = obstacle.position - self.position #calculates distance from agent to obstacle
            distance = np.linalg.norm(distance_obstacle) #calculates absolute value of this distance
            actual_distance = distance - obstacle.radius   #calculates actual absolute distance considering obstacle radius
            if actual_distance < perception_radius:
                if distance > 0:
                    direction = distance_obstacle / distance #caclulates unit vecotr of velocity
                strength = max_avoidance_force * (1.0 - actual_distance / perception_radius)**2 #calculates a quadratic scaling avoidance force based on how close the agent is to obstacle
                self.velocity -= direction * strength #applies strength to velocity to steer away from obstacle 
                if actual_distance < 10:
                    self.velocity = -direction * 5.0 #increases force of maneuver if agent gets dangerously close

    def update(self):
        self.position += self.velocity * self.speed  # move current direction heading
        # Wrap around screen edges
        self.position[0] = self.position[0] % width  # wrap horizontally
        self.position[1] = self.position[1] % height
        
        
    def draw(self, screen):
        # draw the agent as a white circle
        pos = self.position.astype(int)
        pygame.draw.circle(screen, (255, 255, 255), pos, 4)
        # draw line showing direction
        direction_length = 15  # Increased from 5 to 15
        normalized_velocity = self.velocity / np.linalg.norm(self.velocity) if np.linalg.norm(self.velocity) > 0 else self.velocity
        end_pos = pos + (normalized_velocity * direction_length)
        pygame.draw.line(screen, (255, 0, 0), pos, end_pos.astype(int), 2)
    

class Obstacles:
    def __init__(self, x, y, radius): #intialisation function for obstacle class
         self.position=np.array([x,y], dtype=np.float64)
         self.radius=radius
    
    def draw(self, screen):
        pos = self.position.astype(int)
        pygame.draw.circle(screen, (0, 0, 255), pos, self.radius)
        
agents=[]
obstacles=[]

for i in range(7): #loop to initialise objects on pygame simulation
    x = random.randint(0, width)
    y = random.randint(0, height)
    radius = random.randint(10, 30)
    obstacles.append(Obstacles(x, y, radius))
    
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
        agent.cohesion()
        agent.apply_random_movement()
        agent.obstacle_avoidance(obstacles)
        agent.draw(screen)
        
    for obs in obstacles:
        obs.draw(screen)
        
    pygame.display.flip()
    clock.tick(60) #fps

pygame.quit()