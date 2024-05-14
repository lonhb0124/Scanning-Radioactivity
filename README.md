# Scanning-Radioactivity
This project to simulate automation Scanning Radioactivity robot using RRT algorithm

* Requirement : Ros2 Humble, Gazebo, Linux, CV2

# Keyboard key implementation
● We used keys to operate different processes while our program is running:

![image](https://github.com/lonhb0124/Scanning-Radioactivity/assets/111609834/3dc8e905-5954-4d0e-a2d2-613f19f145e1)

# Model in the map (Gazebo)
● We used the blueprint floor plan model as world.

● This world includes the house’s walls as obstacles and a laser robot.

![image](https://github.com/lonhb0124/Scanning-Radioactivity/assets/111609834/af81208b-9a58-413c-af15-c5479ad96f20)


![image](https://github.com/lonhb0124/Scanning-Radioactivity/assets/111609834/114505da-9ffc-417d-ba7c-47781223fa9d)

# Generate Occupancy Map Image

● To generate our occupancy map, we divided the process into multiple steps

1) Press key ‘J’ to start the map recording process. Using a modified LIDAR Robot, we enabled the front, left and right sensors of the LIDAR. Then, once we move it to an unoccupied space, we spin the robot on the spot to generate our map in real-time

2) Save and read the generated occupancy map as a jpeg image. Stored in the current directory. As an additional implementation, we can re-display the saved map by pressing the key ‘L’.

![image](https://github.com/lonhb0124/Scanning-Radioactivity/assets/111609834/edb67bd7-6232-4220-ba87-7a4fdeab4483)

# Integrating RRT to Solve Start to Goal (Path Planning and Execution)

We included our generated occupancy map to work with our RRT code

● Press key ‘K’ to integrate RRT code

1. Dilation object point in the image

2. Generates nodes and line segments incrementally

- The length (line segments/path) between nodes is 0.45m which is 9 pixels.
- Once a single node sees a clear path to goal, RRT Tree generation stops and a direct path from current node to goal is created
- Initially, the goal point is red and other nodes are green. Once the shortest path is found, shortest path nodes change in colour from green to red (including the shortest path lines)

3. Includes Dijkstra’s algorithm to find the shortest path from start to goal

![image](https://github.com/lonhb0124/Scanning-Radioactivity/assets/111609834/9607b312-e61b-45d8-ae29-d3d25812089f)

# Scanning Radioactivity

● Press key ‘S’ to start Scanning Radioactivity

● This process has five states.

1. At Start
2. Heading to Task - move using RRT path for scanning radioactivity
3. Scan Radioactivity - new state that scans a rectangular grid and displays in map
4. Returning from Task - Returning using RRT path.
5. Task Done
-----------------------------------------------------------------------------------------------------

1. At Start state

● The current robot position is the initial position of FSM

● Wait 2 seconds to start and move to next state (Heading to task)

2. Heading to Task (move using RRT path for scanning radioactivity)

● In this state, the robot moves using the RRT shortest path (From initial position to goal position). Our algorithm is that we store shortest path nodes list and consider the final node as last goal

● We used the “drive to goal algorithm” to handle the robot’s movement.

● Final node of RRT is at the bottom-left corner point of the area for scanning radioactivity.

● This is the starting point for the scan radioactivity task

● If the robot arrive in the final nodes, move to next state (Scan Radioactivity)

3. Scan Radioactivity

● In this state the robot starts scanning radioactivity in an area with 1m width and 2m height.

● Instead of 2m x 1m (x by y) we implemented the grid to be 1m x 2m (x by y)

● During the scanning radioactivity, the scanning area will be filled in the display as a black rectangle.

4. Returning from Task

● In this state, the robot moves using the RRT shortest path, but the robot uses shortest path nodes list inversely to return to the initial point (initial position of FSM).

5. Task Done

● Wait 2 seconds and finish the task.

![image](https://github.com/lonhb0124/Scanning-Radioactivity/assets/111609834/3b0ab5dd-09b5-4a19-beca-c2baedc429f4)

# Scanning Radioactivity Pattern
The 1st step is to go right by offset_length, which is 0.25m.

The 2nd step is to move up across the length of 2m

The 3rd step is the same as the 1st step

The 4th step is to move down across the length of 2m

The 5th step is the same as First step

The 6th step is the same as the 2nd step.

The last step is to go back to the corner point of the area, which is the initial point for scanning radioactivity.

We only scan for radioactivity as we move 2m across the designated area

● Robot scans the right and left side by 0.25m (total 0.5m in length) as a line that is perpendicular along the 2m path.

![image](https://github.com/lonhb0124/Scanning-Radioactivity/assets/111609834/33266f09-49f0-493c-b771-f9715b87b947)

# Video Link

https://drive.google.com/file/d/10LB-rZV0ygeDI6XIYbqVe5WBhCARzBJc/view

