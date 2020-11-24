# Second Tutorial for PCIMR

## Does it work out of the box for rto-1?
No, since the node publishes to the wrong topic. Even if the node would publish to the right topic, only +- 45 degrees are considered for the robots front, so holonomic movements can not be covered completely.

## What was needed to implement omnidirectional drive aswell.
### What did you have to update
Change the robot publisher and subscriber values to subscribe to the correct nodes, look ahead changes direction based on movement to allow for near 90 view in calculated direction.

### Why did it not work
Because the node connections were wrong and the robot was simply looking ahead before and not to its left and right for the required angles to allow for omnidirectional movement in these directions.

