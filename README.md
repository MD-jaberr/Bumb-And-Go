This README file explains the modifications done on the given Bump and Go code such that it alligns with the requirements asked. 

*Note: In all parts, all necessary variable initializations are handled in the `.hpp` file.* 
------------------------------------------------------------------------------------
### Exercise 1  
------------------------------------------------------------------------------------
The robot is supposed to detect obstacles in front, on its left, and right diagonals. Instead of always turning to the same side, the robot should turn toward the side without an obstacle.  

==> The solution involves modifying the code to read an array of values instead of just the value at 0 deg. The 333rd index corresponds to the 0-degree axis. The ±45-degree lines, determined by the angle increments between scans, correspond to the 196th and 470th indices. To detect obstacles at these diagonals, the code now reads the range array from indices 196 to 470. The loop checks for the minimum range value within this range, and if the value is smaller than the obstacle threshold (set in the `br2_fsm_bumpgo_hpp` file), an obstacle is detected. The robot then turns in the opposite direction based on the comparison between the left and right minimum range values. The robot turns toward the side with the larger minimum range value. However, eventhough the robot is still in motion, the robot might get stuck between obstacles. 

------------------------------------------------------------------------------------
### Exercise 2 (Open Loop Approach):  
------------------------------------------------------------------------------------
The Bump and Go code snippets are modified to have the robot turn toward the angle with no obstacles or the farthest perceived obstacle based on precalculated time to turn and speed of turning.

==> The obstacle detection code from Part 1 is reused, where the robot checks between -45 and 45 degrees for obstacles. In this approach, we aim to determine the exact angle for the maximum perceived distance and move toward it. When an obstacle is detected in the 196-470 range, the code iterates over this range to find the maximum range value and its corresponding index. The angle between the robot's x-axis (0 degrees) and the maximum range is then calculated by adding `max_index * angle_increment` to the minimum angle value (-1.91 radians). The sign of this angle determines whether it is to the left or right of the x-axis. Assuming the robot rotates at a constant angular velocity of 0.3 rad/s (as set in the `br2_fsm_bumpgo_hpp` file), the turning duration is calculated by dividing the angle by this speed. The robot turns for the calculated duration and then begins moving forward.  One issue was when the robot's path was restricted but the view was not (e.g., under a table), causing the robot to try and turn toward an inaccessible area, getting stuck in a loop. To address this, if the maximum range value is found within the 196-470 range, these values are ignored, and the robot checks for the maximum value in the range (0-195) U (471-666).  

------------------------------------------------------------------------------------
### Exercise 2 (Closed Loop Approach):  
------------------------------------------------------------------------------------
The Bump and Go code snippets are modified to have the robot turn toward the angle with no obstacles or the farthest perceived obstacle according to detected clear space in front of the robot.  
 
==> This approach is similar to the open-loop method, using the same code for obstacle detection from Part 1 and for finding the maximum value in the range. However, in this case, we do not pre-calculate the `turning_duration_` since it is not needed. The robot begins turning in the preferred direction based on the index value. On each iteration, the robot checks the entire range for the maximum value index. If the maximum value is at index 333 or within 3 indices of it (to avoid overshoot and oscillations), the robot is facing the direction without obstacles and can begin moving forward.
