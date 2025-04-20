import numpy as np
import random
import pandas as pd
import sys
from datetime import datetime
from sklearn.cluster import DBSCAN

from boids1copy import Agent

def calculate_nearest_neighbours(agents): # method that calculates the average distance of nearest neighbour distance in the simulation
    distances = []
    for agent in agents:
        if len(agents)>1:
           min_dist=10000000
           for a in agents:
               if a!=agent:
                   temp_dist= np.linalg.norm(agent.position - a.position)
                   if temp_dist<min_dist:
                       min_dist=temp_dist
           if min_dist!= 10000000:
               distances.append(min_dist) 
    if len(distances)>0:
        total=0
        for d in distances:
            total+=d        
        return total/len(distances)
    else:
        return 0

def calculate_polarisation(agents): #calculates average alignment (polarisation)
    if len(agents)==0:
        return 0
    velocities = []
    for agent in agents:
        velocity_magnitude = np.linalg.norm(agent.velocity)
        if velocity_magnitude > 0:
            normalised_velocity = agent.velocity / velocity_magnitude
            velocities.append(normalised_velocity)
        else:
            velocities.append(np.array([0, 0]))
            
    sum_x = 0
    sum_y = 0
    for vel in velocities:
        sum_x += vel[0]
        sum_y += vel[1]
    
    mean_direction = np.array([sum_x / len(velocities), sum_y / len(velocities)])
    return np.linalg.norm(mean_direction)

def number_of_schools(agents): #Uses DBSCAN clustering algorithm to create labels for individuals which are then uniquley counted
    if len(agents) < 2:
        return 1
        
    positions = []
    for agent in agents:
        positions.append([agent.position[0], agent.position[1]])
    positions = np.array(positions)
    
    clustering = DBSCAN(eps=100, min_samples=2).fit(positions)
    labels = clustering.labels_
    unique_labels = set(labels)
    num_clusters = len(unique_labels)
    if -1 in unique_labels:
        num_clusters -= 1
    
    return max(1, num_clusters)  

def run_simulation_headless(visual_range, alignment_force, cohesion_force, field_of_view_angle, max_frames=300):
    width, height = 1500, 900
    agents = []
    for i in range(100): #initialises 100 agents
        x = random.randint(0, width)
        y = random.randint(0, height)
        speed = 2
        agent = Agent(x, y, speed) #initialises indivudal from main code file
    
        agent.visual_range = visual_range #asigns parameters to variable names
        agent.max_alignment_force = alignment_force
        agent.max_cohesion_force = cohesion_force
        
        blind_spot_size = 360 - field_of_view_angle 
        blind_spot_start = 180 - (blind_spot_size / 2) #calculates lower end of blindspot
        blind_spot_end = 180 + (blind_spot_size / 2) #calculates upper end of blindspot
        agent.blind_spot_start = blind_spot_start #assigns the parameters in the simulation to these variables
        agent.blind_spot_end = blind_spot_end
 
        agents.append(agent)
        
    num_schools_values = []
    nearest_neighbour_values = []
    polarisation_values = []

    for frame_count in range(max_frames): 
        for agent in agents:  #runs simulation for frame count
            agent.separation(agents)
            agent.alignment()
            agent.cohesion()
            agent.apply_random_movement()
            agent.update()
        
        if frame_count % 20 == 0: #runs every 20 frames
            current_schools = number_of_schools(agents) #runs metrics on agent simulation
            current_nnd = calculate_nearest_neighbours(agents)
            current_polarisation = calculate_polarisation(agents)

            num_schools_values.append(current_schools) #adds metrics returned from functions to list
            nearest_neighbour_values.append(current_nnd)
            polarisation_values.append(current_polarisation)
    
   
    if len(num_schools_values) > 5: #checks if there are enough values in metric list
        avg_schools = sum(num_schools_values[-5:]) / 5 #takes last 5 values and calculates average number of schools
        avg_nnd = sum(nearest_neighbour_values[-5:]) / 5 #takes last 5 values and calculates average nearest neighbour distance
        avg_polarisation = sum(polarisation_values[-5:]) / 5 #takes last 5 values and calculates average alginment
    else:
        avg_schools = sum(num_schools_values) / len(num_schools_values) if num_schools_values else 0 #if there are less than 5 values just use all available
        avg_nnd = sum(nearest_neighbour_values) / len(nearest_neighbour_values) if nearest_neighbour_values else 0
        avg_polarisation = sum(polarisation_values) / len(polarisation_values) if polarisation_values else 0
    
    return { #returns dictionary of metrics
        'num_schools': float(avg_schools),
        'nnd': float(avg_nnd),
        'polarisation': float(avg_polarisation)
    }

def run_parameter_sweep(): #parameter sweep that runs run_simulation_headless for multiple parameter combination

    visual_ranges = [30, 50, 70] #intialises list of paramters to run
    alignment_forces = [0.3, 0.5, 0.7]
    cohesion_forces = [0.03, 0.05, 0.07]
    field_of_view_angles = [180, 270, 360]
    total_combinations = len(visual_ranges) * len(alignment_forces) * len(cohesion_forces) * len(field_of_view_angles) #calculates total possible combination
    results = []
    combination_count = 0
    try: #incase run fails
        for vr in visual_ranges:
            for af in alignment_forces:
                for cf in cohesion_forces:
                    for fov in field_of_view_angles: #4 nested loops to loop through all possible combination
                        combination_count += 1
                        print(f"Parameters: VR={vr}, AF={af}, CF={cf}, FOV={fov}") #outputs combination at the start of each run
             
                        sim_result = run_simulation_headless(vr, af, cf, fov) #passes in parameters to utilise and dicitonary is assgined to sim_result
                        
                        sim_result['visual_range'] = vr #adds new key value pairs to dictionary for parameters used
                        sim_result['alignment_force'] = af
                        sim_result['cohesion_force'] = cf
                        sim_result['field_of_view'] = fov
                        results.append(sim_result) #adds metric values to dicitonary
                        
                        print(f"Result: Schools={sim_result['num_schools']:.2f}, "
                              f"NND={sim_result['nnd']:.2f}, "
                              f"Polarisation={sim_result['polarisation']:.2f}") #outputs metric values after simulation run
                        
                
                        df = pd.DataFrame(results) #adds results to panda dataframe
                        df.to_csv('parameter_sweep_results.csv', index=False) #adds dataframe to csv file- one line=one pass of simulation
                
                              
    except KeyboardInterrupt: #fail cases
        print("\nInterrupt") 
    except Exception as e:
        print(f"\nError during parameter sweep: {e}")
 
    if results: #if parameter sweep successful create csv
        df = pd.DataFrame(results)
        filename = f'parameter_sweep_results.csv'
        df.to_csv(filename, index=False)
 
        print("\nSummary")
        print(f"Total combinations tested: {len(results)}/{total_combinations}")

    else:
        print("No results to save.")
        return None

if __name__ == "__main__":
    input("Enter to start") #enter to run parameter sweep
    results_df = run_parameter_sweep()
   
    sys.exit(0)