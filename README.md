This README file explains the modifications done on the given Bump and Go code such that it alligns with the requirements asked. 

------------------------------------------------------------------------------------
### Part 1:  
------------------------------------------------------------------------------------
**Requirement**: Modify the Bump and Go project so that the robot can detect obstacles in front, on its left, and right diagonals. Instead of always turning to the same side, the robot should turn toward the side without an obstacle.  
**Solution**:  
The solution involves modifying the code to read an array of values instead of just the value at the 333rd index (representing 0 degrees). The 333rd index corresponds to the 0-degree axis. The ±45-degree lines, determined by the angle increments between scans, correspond to the 196th and 470th indices. To detect obstacles at these diagonals, the code now reads the range array from indices 196 to 470. The loop checks for the minimum range value within this range, and if the value is smaller than the obstacle threshold (set in the `br2_fsm_bumpgo_hpp` file), an obstacle is detected. The robot then turns in the opposite direction based on the comparison between the left and right minimum range values. The robot turns toward the side with the larger minimum range value.  
*Note: All necessary variable initializations are handled in the `.hpp` file.*  
**Cons**: This approach sometimes causes the robot to become stuck between two obstacles (e.g., when trying to enter a doorway), which prevents the robot from fully exploring the map despite it still moving.

------------------------------------------------------------------------------------
### Part 2 - Open Loop Approach:  
-------------------------------------------------------------------------------------
**Requirement**: Modify the Bump and Go project to have the robot turn toward the angle with no obstacles or the farthest perceived obstacle. Implement two approaches:  
- **Open-loop**: Pre-calculate the time and speed required for turning.  
For the open-loop approach, the obstacle detection code from Part 1 is reused, where the robot checks between -45 and 45 degrees for obstacles. In this approach, we aim to determine the exact angle for the maximum perceived distance and move toward it. When an obstacle is detected in the 196-470 range, the code iterates over this range to find the maximum range value and its corresponding index. The angle between the robot's x-axis (0 degrees) and the maximum range is then calculated by adding `max_index * angle_increment` to the minimum angle value (-1.91 radians). The sign of this angle determines whether it is to the left or right of the x-axis. Assuming the robot rotates at a constant angular velocity of 0.3 rad/s (as set in the `br2_fsm_bumpgo_hpp` file), the turning duration is calculated by dividing the angle by this speed. The robot turns for the calculated duration and then begins moving forward.  
A challenge encountered during this implementation was when the robot's path was restricted but the view was not (e.g., under a table), causing the robot to try and turn toward an inaccessible area, getting stuck in a loop. To address this, if the maximum range value is found within the 196-470 range, these values are ignored, and the robot checks for the maximum value in the range (0-195) U (471-666).  
*Note: Variable initialization is handled in the `.hpp` file.*

------------------------------------------------------------------------------------
### Part 2 - Closed Loop Approach:  
------------------------------------------------------------------------------------
**Requirement**: Modify the Bump and Go project so that the robot turns toward the angle with no obstacles or the farthest perceived obstacle. Implement two approaches:  
- **Closed-loop**: The robot turns until a clear space in front is detected.  
This approach is similar to the open-loop method, using the same code for obstacle detection from Part 1 and for finding the maximum value in the range. However, in this case, we do not pre-calculate the `turning_duration_` since it is not needed. The robot begins turning in the preferred direction based on the index value. On each iteration, the robot checks the entire range for the maximum value index. If the maximum value is at index 333 or within 3 indices of it (to avoid overshoot and oscillations), the robot is facing the direction without obstacles and can begin moving forward.
