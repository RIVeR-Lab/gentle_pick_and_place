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
2. After this command, you need to start the `gentle_pick` program on the robot teaching pendant.
 - Start the robot teaching pendant by pressing the power button.
 - Once the pendant is on, tap on "load program" in the middle left, and select `gentle_pick.urp` from the new window.
 - In the bottom left corner, click the red "power off button" to open the initialization screen. Click "on" to go to idle mode, and again to go to operatopnal mode.
 - Exit the initialization screen and press the play button in the lower right. 

3. Start the camera node
```
roslaunch pick_and_place kinect.launch
```
4. Start the object clustering server
```
rosrun pick_and_place object_clustering_server
```
5. Start the pick and place demo
```
rosrun pick_and_place pick_and_place.py
```
If there is no item left on the talbe, the python program will automatically stop.   
You need to manually add items back to the table and rerun the last command in the same window.
