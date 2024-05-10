# Scanning-Radioactivity

# Keyboard key implementation
● We used keys to operate different processes while our program is running:

![image](https://github.com/lonhb0124/Scanning-Radioactivity/assets/111609834/3dc8e905-5954-4d0e-a2d2-613f19f145e1)

# Model in the map (Gazebo)
● We used the same world (blueprint floor plan model).
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
