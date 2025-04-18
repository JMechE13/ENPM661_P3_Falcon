# ENPM661 Project 3 Phase 2 FalconSim Implementation

Riley Albert,
Directory ID: ralbert8
UID: 120985195

Adam Del Colliano,
Directory ID: adelcoll
UID: 115846982

Joseph Shaheen,
Directory ID: jshaheen
UID: 116534321

# Instructions
Due to the large size of the original repo, this repo only contains the files that were modified from the original

Follow the original repo (link provided at end of README.md) instructions up to and including step 4. After doing so, find the files that are mentioned in this repo in the original one and replace them with the files in this repo -- the file structure should be the same.

After making those changes, update the two paths in the launch.py file: 'cwd=' and '-scenario' to your own full paths as mentioned in step 5. No other changes need to be made.

After making the changes make sure you go to the workspace directory and build/source the packages:

```sh
colcon build
source install/setup.bash
```
Note: the above command must be run with any changes made inside the workspace

Then you can run the program with the defaul values using:

```sh
ros2 launch astar_falcon_planner ros_falcon_astar.launch.py
```

# Note

If the start position should be changed when launching - by either changing defaults or from command line parameter assigning, the starting position in the 'AMRPathPlanning.usda' need to be changed as that is where the turtlebot spawns in the Sim. The top left of the maze is the origin with right being +x and down being +y. The absolute coordinates for the origin are 1600,600,0 which are in cm. The parameters passed when naming them in the launch command are in meters relative to the origin as if the origin was 0,0. The defaults in the launch file are the same.

Example: 
To change start position to 0.5,0.5,0 in launch command/file you must go to 'AMRPathPlanning.usda' and change spawn point to 1600+50, 600+50,0 --> 1650,650,0 -- note the conversion from m to cm.

All parameters in the launch command/file are in meters, seconds, rpms.
To run with parameters changed on command line:

```sh
ros2 launch astar_falcon_planner ros_falcon_astar.launch.py \
    start_position:="[0.0, 0.0, 0.0]"\
    end_position:="[0.0, 0.0, 0.0]" \
    robot_radius:=0.0 \
    clearance:=0.0 \
    delta_time:=0.0 \
    wheel_radius:=0.0 \
    wheel_distance:=0.0 \
    rpms:="[0.0, 0.0]"
```
Note quotes around arrays, all values should be floats.


# Links
Link to Original Repo from Duality for this project: 
https://github.com/duality-robotics/UMD-ENPM661-AStar-FalconSim-Assignment/tree/main

Link to Project Videos:
https://drive.google.com/drive/folders/1iDzi45sgHjE6V_lIN9iSaKgw5NmZLQ8m?usp=drive_link

