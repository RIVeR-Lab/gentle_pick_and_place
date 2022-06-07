# Gentle_Pick_and_Place
Pick and place demo for gentle gripper.

## Running instructions
Open four terminals and run the source command in each terminal:
```
source ~/ur_ws/devel/setup.bash
```
Then run the following commands in each terminal by the same order:
1. Start the robot
```
roslaunch pick_and_place ur3e.launch
```
After this command, you need to start the `gentle_pick` program on the robot teaching pendant.

2. Start the camera node
```
roslaunch pick_and_place kinect.launch
```
3. Start the object clustering server
```
rosrun pick_and_place object_clustering_server
```
4. Start the pick and place demo
```
rosrun pick_and_place pick_and_place.py
```
If there is no item left on the talbe, the 4th program will automatically stop.   
You need to manually add items back to the table and rerun the 4th command in the same window.
