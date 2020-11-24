# Second Tutorial for PCIMR
This repository contains code for the second tutorial for PCIMR@TUM. Using a simulation available at https://github.com/dietriro/rto_simulation we allow our robot to avoid obstacles by reading LaserScan Data from a Robotino/p3dx Robot and figuring out if an obstacle would be in the way of our desired path, then we stop if the path is obstructed. The solution works to detect obstacles within +-2 radians from the center middle. Obstacles behind the robot are unavoidable since we have no sensors placed there and this algorithm doesnt use a precomputed map to avoid "invisible" obstacles.

## Does it work out of the box for rto-1?
No, since the node publishes to the wrong topic. Even if the node would publish to the right topic, only +- 45 degrees are considered for the robots front, so holonomic movements can not be covered completely.

## What was needed to implement omnidirectional drive aswell.
### What did I have to update
Change the robot publisher and subscriber values to subscribe to the correct nodes, look ahead changes direction based on movement to allow for near 90 view in calculated direction.

### Why did it not work
Because the node connections were wrong and the robot was simply looking ahead before and not to its left and right for the required angles to allow for omnidirectional movement in these directions.

## Commands needed for launch
These all have to be launched in their own terminals
~~~
roscore
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/input/cmd_vel
roslaunch rto_bringup_sim robot.launch 
roslaunch pcimr_tutorial_02 pcimr_tutorial_02.launch 
~~~
If the environment variable robot is not defined, this package defaults to the rto-1 robot since holonomic and differential drive options can be evaluated with one robot that way. 

