import numpy as np
import heapq as hq
import cv2
import time
import math

from typing import Dict, Tuple, List, Union


# visualization of map with robot movements
def visualize_environment(obstacles, clearances, start, goal, path, trajectory_map, trajectory_list, RR, map_size = (5400,3000)):
    map_x, map_y = map_size 
    scale = 0.2
    # Create a blank 5400x3000 white frame
    frame = np.ones((map_y, map_x, 3), dtype=np.uint8) * 255

    # Generate meshgrid of all (x, y) coordinates
    x_grid, y_grid = np.meshgrid(np.arange(map_x), np.arange(map_y))

    # Compute clearance area and display as gray
    for conditions in clearances.values():
        mask = np.ones_like(x_grid, dtype=bool)
        for cond in conditions:
            mask &= cond(x_grid, y_grid)
        frame[mask] = (150, 150, 150)

    # Compute obstacle area and display as black
    obstacle_mask = np.zeros_like(x_grid, dtype=bool)
    for conditions in obstacles.values():
        temp_mask = np.ones_like(x_grid, dtype=bool)
        for cond in conditions:
            temp_mask &= cond(x_grid, y_grid)
        obstacle_mask |= temp_mask
    frame[np.where(obstacle_mask)] = (0, 0, 0)

    # Flip to match coordinate system
    frame = cv2.flip(frame, 0)

    # Draw final start and goal points
    cv2.circle(frame, (int(start[0]), int(map_y - start[1])), 50, (0, 0, 255), -1)  # Red (start)
    cv2.circle(frame, (int(goal[0]), int(map_y - goal[1])), RR, (0, 255, 0), -1)  # Green (goal)
    cv2.circle(frame,(int(goal[0]), int(map_y - goal[1])), 15, (0, 0, 0), -1)  # Green (goal)

    print(('Plotting Explored Nodes...'))
    # Draw explored node trajectories
    for i, trajectory in enumerate(trajectory_list):
        for j in range(len(trajectory) - 1):
            x1, y1, _ = trajectory[j]
            x2, y2, _ = trajectory[j + 1]

            y1_flipped = map_y - y1
            y2_flipped = map_y - y2

            cv2.line(frame, (int(x1), int(y1_flipped)), (int(x2), int(y2_flipped)),
                        (0, 200, 200), 5)
        #else:
            #print("not there")

        # Update display every 100 steps
        if i % 2 == 0:
            scale_frame = cv2.resize(frame, (int(map_x * scale), int(map_y * scale)), interpolation=cv2.INTER_LINEAR)
            cv2.imshow("A* Path Visualization", scale_frame)
            cv2.waitKey(1)

    print('Finished')
    # Draw path node trajectories
    if path is not None:

        print('Plotting Path...')
        for node in path:
            if node in trajectory_map:
                trajectory = trajectory_map[node]
                for j in range(len(trajectory) - 1):
                    x1, y1, _ = trajectory[j]
                    x2, y2, _ = trajectory[j + 1]

                    y1_flipped = map_y - y1
                    y2_flipped = map_y - y2

                    cv2.line(frame, (int(x1), int(y1_flipped)), (int(x2), int(y2_flipped)),
                             (255, 0, 0), 5)  # Blue path

        # Draw final start and goal points
        cv2.circle(frame, (int(start[0]), int(map_y - start[1])), 50, (0, 0, 255), -1)  # Red (start)
        cv2.circle(frame, (int(goal[0]), int(map_y - goal[1])), RR, (0, 255, 0), -1)  # Green (goal)
        cv2.circle(frame,(int(goal[0]), int(map_y - goal[1])), 15, (0, 0, 0), -1)  # Green (goal)


        scale_frame = cv2.resize(frame, (int(map_x * scale), int(map_y * scale)), interpolation=cv2.INTER_LINEAR)

        # Final visualization
        print('Finished')
        cv2.imshow("A* Path Visualization", scale_frame)

    scale_frame = cv2.resize(frame, (int(map_x * scale), int(map_y * scale)), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("A* Path Visualization", scale_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# visualize falcon path only
def visualize_falcon_path(obstacles, clearances, start, goal, path, RR, map_size = (5400,3000)):
    map_x, map_y = map_size 
    scale = 0.2
    # Create a blank 5400x3000 white frame
    frame = np.ones((map_y, map_x, 3), dtype=np.uint8) * 255

    # Generate meshgrid of all (x, y) coordinates
    x_grid, y_grid = np.meshgrid(np.arange(map_x), np.arange(map_y))

    # Compute clearance area and display as gray
    for conditions in clearances.values():
        mask = np.ones_like(x_grid, dtype=bool)
        for cond in conditions:
            mask &= cond(x_grid, y_grid)
        frame[mask] = (150, 150, 150)

    # Compute obstacle area and display as black
    obstacle_mask = np.zeros_like(x_grid, dtype=bool)
    for conditions in obstacles.values():
        temp_mask = np.ones_like(x_grid, dtype=bool)
        for cond in conditions:
            temp_mask &= cond(x_grid, y_grid)
        obstacle_mask |= temp_mask
    frame[np.where(obstacle_mask)] = (0, 0, 0)

    # Flip to match coordinate system
    frame = cv2.flip(frame, 0)

    # Draw final start and goal points
    cv2.circle(frame, (int(start[0]), int(map_y - start[1])), 50, (0, 0, 255), -1)  # Red (start)
    cv2.circle(frame, (int(goal[0]), int(map_y - goal[1])), RR, (0, 255, 0), -1)  # Green (goal)
    cv2.circle(frame,(int(goal[0]), int(map_y - goal[1])), 15, (0, 0, 0), -1)  # Green (goal)

    if path is not None:

        print('Plotting Path...')
        # print(len(path))
        # print(path)
        for node in path:
            # print(node)
            # wait = input('waiting')
            x,y = node[0], node[1]
            cv2.circle(frame, (int(x),int(map_y - y)), 20, (0,0,0), -1)
            
        # Draw final start and goal points
        cv2.circle(frame, (int(start[0]), int(map_y - start[1])), 50, (0, 0, 255), -1)  # Red (start)
        cv2.circle(frame, (int(goal[0]), int(map_y - goal[1])), RR, (0, 255, 0), -1)  # Green (goal)
        cv2.circle(frame,(int(goal[0]), int(map_y - goal[1])), 15, (0, 0, 0), -1)  # Green (goal)


        scale_frame = cv2.resize(frame, (int(map_x * scale), int(map_y * scale)), interpolation=cv2.INTER_LINEAR)

        # Final visualization
        print('Finished')
        cv2.imshow("A* Path Visualization", scale_frame)

    scale_frame = cv2.resize(frame, (int(map_x * scale), int(map_y * scale)), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("A* Path Visualization", scale_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# get map obstacle defintions
def get_obstacles():

    # Define obstacles
    obstacles = {

            "Obstacle 1": [
                lambda x, y: x >= 1000,
                lambda x, y: x <= 1100,
                lambda x, y: y >= 0,
                lambda x, y: y <= 2000
            ],

            "Obstacle 2": [
                lambda x, y: x >= 2100,
                lambda x, y: x <= 2200,
                lambda x, y: y >= 1000,
                lambda x, y: y <= 3000
            ],

            "Obstacle 3": [
                lambda x, y: x >= 3200,
                lambda x, y: x <= 3300,
                lambda x, y: y >= 0,
                lambda x, y: y <= 1000
            ],

            "Obstacle 4": [
                lambda x, y: x >= 3200,
                lambda x, y: x <= 3300,
                lambda x, y: y >= 2000,
                lambda x, y: y <= 3000
            ],

            "Obstacle 5": [
                lambda x, y: x >= 4300,
                lambda x, y: x <= 4400,
                lambda x, y: y >= 0,
                lambda x, y: y <= 2000
            ],

    }

    return obstacles

# prompts user for desired clearance, generates clearances dict
def get_clearance(RR, user_clearance, map_size = (5400, 3000)):

    map_x, map_y = map_size

    clearance = RR + user_clearance

    # Define clearances
    clearances = {

            "Clearance 1": [
                lambda x, y: x >= 1000-clearance,
                lambda x, y: x <= 1100+clearance,
                lambda x, y: y >= 0,
                lambda x, y: y <= 2000+clearance
            ],

            "Clearance 2": [
                lambda x, y: x >= 2100-clearance,
                lambda x, y: x <= 2200+clearance,
                lambda x, y: y >= 1000-clearance,
                lambda x, y: y <= 3000
            ],

            "Clearance 3": [
                lambda x, y: x >= 3200-clearance,
                lambda x, y: x <= 3300+clearance,
                lambda x, y: y >= 0,
                lambda x, y: y <= 1000+clearance
            ],

            "Clearance 4": [
                lambda x, y: x >= 3200-clearance,
                lambda x, y: x <= 3300+clearance,
                lambda x, y: y >= 2000-clearance,
                lambda x, y: y <= 3000
            ],

            "Clearance 5": [
                lambda x, y: x >= 4300-clearance,
                lambda x, y: x <= 4400+clearance,
                lambda x, y: y >= 0,
                lambda x, y: y <= 2000+clearance
            ],

            "Clearance 6": [
                lambda x, y: x >= 0,
                lambda x, y: x <= 10+clearance,
                lambda x, y: y >= 0,
                lambda x, y: y <= map_y
            ],

            "Clearance 7": [
                lambda x, y: x >= map_x-10-clearance,
                lambda x, y: x <= map_x,
                lambda x, y: y >= 0,
                lambda x, y: y <= map_y
            ],

            "Clearance 8": [
                lambda x, y: x >= 0,
                lambda x, y: x <= map_x,
                lambda x, y: y >= 0,
                lambda x, y: y <= 10+clearance
            ],

            "Clearance 9": [
                lambda x, y: x >= 0,
                lambda x, y: x <= map_x,
                lambda x, y: y >= map_y-10-clearance,
                lambda x, y: y <= map_y
            ],

    }

    return clearances

# define action set given rpms
def get_action_set(rpms):

    actions = [
        (0,rpms[0]),
        (rpms[0],0),
        (rpms[0],rpms[0]),
        (0,rpms[1]),
        (rpms[1],0),
        (rpms[1],rpms[0]),
        (rpms[0],rpms[1]),
        (rpms[1],rpms[1])
    ]
        
    return actions

# checks point for validity: Not within obstacles or clearance regions
def is_valid(x: Union[float,int], y: Union[float,int], clearances: Dict) -> bool:

    # If location is within obstacle constraints
    if any(all(constraint(x, y) for constraint in constraints) for constraints in clearances.values()):

        # Return invalid
        return False
    
    # If location is not within obstacle constraints
    else:
        
        # Return valid
        return True


# get pose after action
def get_pose(x,y,theta_deg,u_l,u_r, WR, WB, DT):


    # Convert RPM to rad/s
    ul_rad = (u_l * 2 * np.pi) / 60
    ur_rad = (u_r * 2 * np.pi) / 60

    # Compute linear and angular velocity
    v = WR/2 * (ul_rad + ur_rad) #mm/s
    omega = WR/WB * (ur_rad - ul_rad) #rad/s

    
    # list for discrete trajectory
    trajectory = []
    trajectory.append((x,y,theta_deg))

    # time step such that 1 mm moved at every step
    time_step = 0.1
    n_steps = max(1,int(DT / time_step))
    theta = np.radians(theta_deg)

    for _ in range(n_steps):
        dx = v * np.cos(theta) * time_step
        dy = v * np.sin(theta) * time_step
        dtheta = omega * time_step
        x += dx
        y += dy
        theta += dtheta
        trajectory.append((x,y,np.degrees(theta)%360))
    # print('trajectory creation time: ', traj_time.stop())
    # print('vel: ', v, ' time step: ', time_step, ' number steps: ', n_steps)
    

    return trajectory
    
# A* algorithm definition
def a_star(start: Tuple[float, float, int], goal: Tuple[float, float], threshold, clearances: Dict, actions: List, WR, WB, DT, map_size: Tuple[int, int] = (5400, 3000)) -> Union[List, None]:
    trajectory_map = {}
    # Mark start time
    start_time = time.time()
    early_stop_on = False
    early_stop = 100      # number of nodes to explore before quitting algorithm
    duplicate_distance_threshold = 100 # if within 5mm of other config, consider as duplicate
    map_x, map_y = map_size

    # convert start and goal to tuples
    start = (start[0], start[1], start[2])
    goal = (goal[0], goal[1])

    # Define function for computing heuristic
    def heuristic(node: Tuple[float, float, int], goal: Tuple[float, float]) -> float:

        euclidean_dist = np.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) 

        # dx = goal[0] - start[0]
        # dy = goal[1] - start[1]
        # direct_to_goal_heading = np.degrees(np.arctan2(dy,dx))
        # heading_difference = abs((direct_to_goal_heading - node[2] +180) % 360 - 180)
        # print(node)
        # print(goal)
        # print(euclidean_dist, direct_to_goal_heading, heading_difference)
        return euclidean_dist #+ heading_difference

    # Define function for backtracking
    def backtrack(goal: Tuple[float, float, int], parent_map: Dict) -> List:
        path_og = []
        path_cm = []
        while goal in parent_map:
            # convert to list format in cm needed for Falcon Sim
            # points = [x cm, y cm, theta rad]
            path_og.append(goal)

            path_cm.append([goal[0]/10.0, goal[1]/10.0, np.deg2rad(goal[2])])
            goal = parent_map[goal]
        path_og.reverse()
        path_cm.reverse()
        return path_og, path_cm

    # Define function for getting node neighbors
    def get_neighbors(node: Tuple[float, float, float], visited: np.ndarray, clearances: Dict, actions: List, WR=WR, WB=WB, DT=DT, map_size: Tuple[int, int] = (map_x, map_y)) -> List:

        x, y, theta = node 
        neighbors = []

        # for every action set generate new node
        #action_i = 0
        for ul, ur in actions:
            
            # get changes
            trajectory = get_pose(x, y, theta, ul, ur, WR, WB, DT)
            final_x, final_y, final_theta = trajectory[-1] 

            new_theta_30_index = int(round(final_theta / 30)) % 12
            int_x, int_y = int(round(final_x)/duplicate_distance_threshold), int(round(final_y)/duplicate_distance_threshold)
            
            flag = 0
            for point in trajectory:
                xi, yi, _ = point
                if not 0 <= xi < map_size[0] or not 0 <= yi < map_size[1] or not is_valid(xi,yi, clearances):
                    flag = 1 
                    break


            if not flag and visited[int_y, int_x, new_theta_30_index] == 0:
                visited[int_y, int_x, new_theta_30_index] = 1
                neighbors.append((final_x, final_y, final_theta))
                trajectory_map[(final_x, final_y, final_theta)] = trajectory
                trajectory_list.append(trajectory)


            #action_i += 1

        return neighbors

    # Create configuration map for visited nodes
    discretized_height = int(map_size[1]/duplicate_distance_threshold)
    discretized_width = int(map_size[0]/duplicate_distance_threshold)
    #print('Discretized Width, Height: ', discretized_width, discretized_height)
    visited = np.zeros((discretized_height,discretized_width, 12), dtype=np.uint8)

    # Initialize open list
    open_list = []
    hq.heappush(open_list, (0, start))

    # Initialize dictionary for storing parent information
    parent_map = {}

    # Initialize dictionary for storing cost information
    cost_map = {start: 0}

    # Initialize list for storing closed nodes and explored nodes
    closed_nodes = []
    explored_nodes = []
    trajectory_list = []

    # Loop until queue is empty
    try:
        while open_list:

            current_node_info = hq.heappop(open_list)
            current_node: Tuple[float, float, int] = current_node_info[1]

            # Add node to closed list
            closed_nodes.append(current_node)

            # Record explored node for visualization
            explored_nodes.append(current_node)
            ## print(current_node)

            # Determine if solution is found
            if np.sqrt((current_node[0] - goal[0]) ** 2 + (current_node[1] - goal[1]) ** 2) <= threshold:

                # Mark end time
                end_time = time.time()

                # print(f"Time to search: {end_time - start_time:.4f} seconds")

                # Backtrack to find path from goal
                path_og, path_cm = backtrack(current_node, parent_map)
                return path_og, path_cm, explored_nodes, trajectory_map, trajectory_list
            
            # Loop through neighbors
            for neighbor in get_neighbors(current_node, visited, clearances, actions):

                # cost for action taken -- all actions valued at 1
                new_cost = cost_map[current_node] + 1  

                # checks if node cost can be reduce and updates it
                if neighbor not in cost_map or new_cost < cost_map[neighbor]:
                    cost_map[neighbor] = new_cost
                    total_cost = new_cost + heuristic(neighbor, goal)
                    hq.heappush(open_list, (total_cost, neighbor))
                    parent_map[neighbor] = current_node

            # logging


            # early stop to give up -- for testing
            if early_stop_on:
                if len(explored_nodes) >= early_stop:
                    break

    except KeyboardInterrupt:
        # print('Force Quit')
        pass



    return None, explored_nodes, trajectory_map, trajectory_list  # Return None if no path is found

# break up path to relative path - assumes 
def get_relatvie_path(path, which=0):

    delta_path = []
    if which:
        multiplier = 1
    else:
        multiplier = -1

    for i, point in enumerate(path):

        if i == len(path) - 1:
            break

        dx = path[i + 1][0] - point[0]
        dy = (path[i + 1][1] - point[1])*multiplier # because the map is actually upside down compared to what we had
        dtheta = get_smallest_rotation(point[2], path[i + 1][2] )

        item = (dx, dy, dtheta)

        delta_path.append(list(item))

    # print('full path ===========================================')
    # for i in delta_path:
    #     print(i)

    return delta_path
        
# test path generated for falcon
# takes in mm
def verify_falcon_path(start, path):

    actual_path = []
    x, y, theta = start
    
    # print(x,y,theta)

    # convert to absolute points in mm
    for point in path:
        x += point[0]
        y += point[1]
        theta += np.rad2deg(point[2])

        new_point_tuple = (x, y, theta)
        actual_path.append(new_point_tuple)
        # # print(actual_path)
        # wait = input('waiting')

    # # print(actual_path)
    # wait = input('verify path from fx verify_falcon_path.............................')

    return actual_path


# avoid angle wrapping
def get_smallest_rotation(angle1, angle2):
    dtheta = angle2 - angle1
    smallest = (dtheta + math.pi)%(2*math.pi) - math.pi
    return smallest

# takes in original path in mm
# from path get full trajectory for falcom sim
# verified correct
# which corresponds to-- 0: for falcon sim -- 1: for own visualization
def get_full_path_trajectory(path, trajectory_map, which=0):

    full_path = []

    # get every trajectory between every node in path
    for node in path:
        # print('Node: ', node)

        trajectory = trajectory_map[node]
        trajectory_formatted = []

        # format trajectories
        # print('Trajectory:')
        for p in trajectory:
            # print(p)
            pass


        #wait = input('wait')

        for point in trajectory:

            if not which:
                x_f, y_f, t_f = convert_to_falcon_format(point)
                item = (x_f, y_f, t_f)
            else:
                item = point
            trajectory_formatted.append(list(item))
        
        # convert to relative trajectory
        relative_trajectory = get_relatvie_path(trajectory_formatted, which)

        # add relative trajectory to full path
        full_path.extend(relative_trajectory)

        # print('full path up to: ')
        for i in full_path:
            # print(i)
            pass

    return full_path

    
# convert from (mm, mm, deg) to (cm, cm, rad)
def convert_to_falcon_format(node):

    x = node[0]/10.0
    y = node[1]/10.0
    theta = np.deg2rad(node[2])

    return [x, y, theta]


# Output path should be in the format List[List[dx,dy,dtheta]] with units cm and radians [[8, 0, 0.7854],[14,14,0]...]
def plan_path(start,
              end,
              robot_radius,
              clearance,
              delta_time,
              goal_threshold,
              wheel_radius,
              wheel_distance,
              rpm1,
              rpm2,
              vis = 0):

    ### Fill in your A* planner here!!!

    map_y = 3000

    # define action set
    DT = delta_time
    rpms = [rpm1, rpm2]
    action_set = get_action_set(rpms)       

    # convert from cm /rad to mm/deg
    WR = wheel_radius*10
    WB = wheel_distance*10
    RR = robot_radius*10
    user_clearance = clearance*10
    threshold = goal_threshold*10

    # adjust y to our frame
    start_mm = [start[0]*10, map_y - start[1]*10,  int(np.rad2deg(start[2])%360)]
    goal_mm = [end[0]*10,  map_y - end[1]*10]

    clearances_map = get_clearance(RR, user_clearance)

    # check start and goal
    if not is_valid(start_mm[0], start_mm[1], clearances_map):
        return -1
    
    if not is_valid(goal_mm[0], goal_mm[1], clearances_map):
        return -1

    print('starting a*')
    path_og, path_cm, exp_nodes, trajectory_map, trajectory_list= a_star(start_mm, goal_mm, threshold, clearances_map, action_set, WR, WB, DT)
    print('finished')

    if path_og is None:
        return -2
    
    
    path_cm_rel = get_full_path_trajectory(path_og, trajectory_map, which=vis)

    if vis:
        # UNCOMMENT TO VERIFY A STAR ALGORITHM IMPLEMENTATION
        visualize_environment(get_obstacles(), clearances_map, start_mm, goal_mm, path_og, trajectory_map, trajectory_list, RR)


        # UNCOMMENT TO VERIFY A STAR ALGORITHM FOR FALCON IMPLEMENTATION
        ver = verify_falcon_path(start_mm, path_cm_rel)
        visualize_falcon_path(get_obstacles(), clearances_map, start_mm, goal_mm, ver, RR)


    return path_cm_rel



if __name__ == '__main__':

    # in cm
    start = [30,30,0.0]
    end = [490,80]
    robot_radius = 22    
    clearance = 0.5  
    delta_time = 1.0    
    goal_threshold = 10
    wheel_radius = 3.3
    wheel_distance = 28.7
    rpm1 = 50
    rpm2 = 150



    path = plan_path(start,
              end,
              robot_radius,
              clearance,
              delta_time,
              goal_threshold,
              wheel_radius,
              wheel_distance,
              rpm1,
              rpm2, vis=1)
    
    if path != -1 or path != -2:
        for p in path:
            print(p)
        print('Success -- points: ', len(path))
    elif path == -1:
        print('failed due to invalid start/goal')
    elif path == -2:
        print('failed due to no path found')

        



'''

command needs to be in METERS, Falcon converts it to cm

ros2 launch astar_falcon_planner ros_falcon_astar.launch.py \
    start_position:="[0.4, 0.4, 0.0]"\
    end_position:="[4.7, 0.4, 0.0]"\
    robot_radius:=0.22\
    clearance:=0.001\
    delta_time:=1.0 \
    goal_threshold:=0.10\
    wheel_radius:=0.033 \
    wheel_distance:=0.287 \
    rpms:="[50.0, 100.0]"








'''