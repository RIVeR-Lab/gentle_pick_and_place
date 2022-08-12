# Gentle_Pick_and_Place
Pick and place demo for gentle gripper.

## Physical setup
The robot currently hard-defines the bins as ~15 inches to the left and right of the left side of the robot's base. (That is, when facing away from the robot, there's a bin to the left of the base 15 inches away from the left side of the base, and there's a bin to the right of the base 15 inches away from the left side of the base.) The bins have a 14-inch long side and a 9-inch short side.

The bin on the left side of the robot is considered the 'bottles' bin, and must be placed with the *long* side facing the robot. The bin on the right, meanwhile, is considered the 'cans' bin, and should be placed with the *short* side facing the robot.

Bottles or cans should be placed within approximately a 2-foot-by-2-foot square directly in front of the robot.

diagram:
```
---------------------------------------------------------------------------------
| [--------------]                  [XXXX]                         [-------]    |
| [              ]                  [XXXX]                         [       ]    |
| [      C       ]                  [XXXX]                         [       ]    |
| [              ]                        ^                        [   B   ]    |
| [______________] < --- ~15 inches --- > | < --- ~15 inches --- > [       ]    |
|                                         |                        [       ]    |
|                                         |                        [_______]    |
|                                                                               |
|                   < --- objs within ~2 feet from from base --->               |
|                                                                               |
|                            ccc          |    bbbbbb                           |
|                           ccc           |    bbbbbb                           |
|                          ccc            |                                     |
|                                         v                                     |
|                                                                               |
|                                                                               |
|                                                                               |
---------------------------------------------------------------------------------

c: can
C: can bin
b: bottle
B: bottle bin
X: robot base
```

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
