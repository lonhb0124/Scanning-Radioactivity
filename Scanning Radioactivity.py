import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from rcl_interfaces.msg import SetParametersResult
import cv2
import random
import os
from enum import Enum

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def draw_obstacles(image, obstacles):
    for obstacle in obstacles:
        cv2.circle(image, tuple(map(int, obstacle[0])), int(obstacle[1]), (0, 0, 0), -1)

def draw_point(image, point, color):
    cv2.circle(image, tuple(map(int, point)), 3, color, -1)

def draw_line(image, start_point, end_point, color):
    cv2.line(image, tuple(map(int, start_point)), tuple(map(int, end_point)), color, 1)
    
class RRT(object):
    def __init__(self, start_position):
        image = cv2.imread('Occupancy_map.jpg')
        # image = cv2.imread('Dilation_map.jpg')
        self.side_len = len(image)
        self.obstacle_props = self.gen_obstacles(image)
        self.start, self.end = self.gen_start_end(self.side_len, start_position)
        self.nodes_list = [['q0', self.start, 'None']]
        self.segments_list = []
        self.gen_tree(self.side_len)
        self.path_nodes, self.path_segments = [], []
        self.find_path()

    def gen_obstacles(self, image):
        obstacles = []
        
        #while len(obstacles) < num:
        for i in range(len(image)):
            for j in range(len(image)):
                if np.all(image[i][j] == 0):
                    #overlap = []
                    center = (j, i)
                    #radius = 2 # for Dilation_map
                    radius = 5 # for Occupancy_map  
                    obstacles.append([center, radius])

        return obstacles
    
    def calc_distance(self, p_1, p_2):
        return math.sqrt((p_2[0] - p_1[0]) ** 2 + (p_2[1] - p_1[1]) ** 2)

    def obstacle_check(self, point):
        for _, props in enumerate(self.obstacle_props):
            if self.calc_distance(point, props[0]) <= props[1]:
                return True
            else:
                pass

        return False

    def gen_start_end(self, length, start_position):
        start_ok, end_ok = False, False

        while not start_ok:
            start = start_position

            if self.obstacle_check(start):
                pass
            else:
                start_ok = True
  # ========================================== End point =============================
        while not end_ok:
            #end = (random.randint(1, length), random.randint(1, length))
            end = [250, 450]

            if self.obstacle_check(end):
                pass
            else:
                end_ok = True

        return start, end
        
    def find_closest(self, point):
        d_list = [self.calc_distance(point, node[1]) for node in self.nodes_list]

        return min(range(len(d_list)), key=d_list.__getitem__)

    def gen_node(self, length):
        point_ok = False
        node_name = "q{}".format(len(self.nodes_list))
        # stepsize = 8
        stepsize = 9

        while not point_ok:
            p_coords = (random.randint(1, length), random.randint(1, length))
            parent = self.nodes_list[self.find_closest(p_coords)]
            d_x = p_coords[0] - parent[1][0]
            d_y = p_coords[1] - parent[1][1]
            vec_mag = math.sqrt((d_x ** 2) + (d_y ** 2))
            node = (parent[1][0] + d_x / vec_mag *stepsize,
                    parent[1][1] + d_y / vec_mag * stepsize)

            if self.obstacle_check(node):
                pass
            else:
                point_ok = True

        self.nodes_list.append([node_name, node, parent[0]])
        self.segments_list.append([parent[1], node])
                
    def path_check(self, point):
        path_collisions = []

        for obs in self.obstacle_props:
            too_close, between = False, False
            r_u, d_obs = self.point_to_line(point, self.end, obs[0])

            if 0 <= r_u <= 1:
                between = True

            if d_obs <= obs[1]:
                too_close = True

            if between and too_close:
                path_collisions.append(True)
            else:
                path_collisions.append(False)

        return any(path_collisions)

    def point_to_line(self, p_1, p_2, p_3):
        dist = math.sqrt((p_2[0] - p_1[0]) ** 2 + (p_2[1] - p_1[1]) ** 2)
        r_u = ((p_3[0] - p_1[0]) * (p_2[0] - p_1[0]) + (p_3[1] - p_1[1]) * (p_2[1] - p_1[1])) / (dist ** 2)
        p_i = (p_1[0] + r_u * (p_2[0] - p_1[0]), p_1[1] + r_u * (p_2[1] - p_1[1]))
        tan_len = self.calc_distance(p_i, p_3)

        return r_u, tan_len

    def gen_end_seg(self):
        self.segments_list.append([self.nodes_list[-1][1], self.end])

    def gen_tree(self, length):
        done = False

        while not done:
            self.gen_node(length)

            if not self.path_check(self.nodes_list[-1][1]):
                done = True

        self.gen_end_seg()

        self.nodes_list.append(["q{}".format(len(self.nodes_list)), self.end, self.nodes_list[-1][0]])

    def find_path(self):
        current = self.nodes_list[-1]
        self.path_nodes.append(current[1])

        for _, j in reversed(list(enumerate(self.nodes_list))):
            if current[2] == j[0]:
                self.path_nodes.insert(0, j[1])
                self.path_segments.insert(0, (j[1], current[1]))
                current = j

class FSM_STATES(Enum):
    AT_START = 'AT STart',
    HEADING_TO_TASK = 'Heading to Task',
    RETURNING_FROM_TASK = 'Returning from Task',
    TASK_DONE = 'Task Done'
    SCAN_RADIOACTIVITY = 'Scanning Radioactivity'
        
class CollectLidar(Node):
    _WIDTH = 900
    _HEIGHT = 900
    _M_PER_PIXEL = 0.05
    cur_x = 0
    cur_y = 0
    cur_t = 0

    def __init__(self):
        super().__init__('collect_lidar')
        self.get_logger().info(f'{self.get_name()} created')

        self._map = np.full((CollectLidar._HEIGHT, CollectLidar._WIDTH), 255, dtype=np.uint8)


        self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self._recording = False
        self._save = False
        self._RRT = False
        self._scan_radioactive = False
        self._callmap = False
        self._path = [[450, 450], [500, 460]]
        self._i = 0
        
        
        # additional condition
        #self._map = np.full((FSM._HEIGHT, FSM._WIDTH), 255, dtype=np.uint8)
        self._row_length = 2.0
        self._row_offset = 0.25
        self._current_row = 1.0
        self._move_to_offset = True
        self._final_row = 3
        self._scan_radioacitiviy = False
        #self._start_task_x = -2.0
        #self._start_task_y = -2.0
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._cur_state = FSM_STATES.AT_START
        self._time_measure = True



    def _scan_callback(self, msg):
        cv2.imshow('map',self._map)
        key = cv2.waitKey(4)
        
        if key == 106: # Key J - Map Recording
            self._recording = True
            self._save = False
            self._RRT = False
            self._scan_radioactive = False
            self._callmap = False
            
        elif key == 107: # Key K - Operate RRT
            self._RRT = True
            self._recording = False
            self._save = False
            self._scan_radioactive = False
            self._callmap = False
            
        elif key == 108: # Key L - Save as image
            self._save = True
            self._recording = False
            self._RRT = False
            self._scan_radioactive = False
            self._callmap = False
        
        elif key == 32: # key Space - Stop Everything
            self._recording = False
            self._save = False
            self._RRT = False
            self._scan_radioactive = False
            self._callmap = False
        
        elif key == 115: # Key S - scan_radioactive
            self._scan_radioactive = True
            self._recording = False
            self._save = False
            self._RRT = False
            self._callmap = False
            
        elif key == 120: # Key X - call occupancy_map
            self._scan_radioactive = False
            self._recording = False
            self._save = False
            self._RRT = False
            self._callmap = True
 
  # ================================ Recording map ======================            
        if self._recording:
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_increment = msg.angle_increment
            ranges = msg.ranges
            a = ranges[0]
            b = ranges[89]
            c = ranges[179]
            d = ranges[269]
            e = ranges[359]
            f = ranges[1]
            #self.get_logger().info(f"lidar ({angle_min},{angle_max},{angle_increment},{len(ranges)})")
            #self.get_logger().info(f"lidar ({a}, {b}, {c}, {d}, {e})")
            closest_range = min(ranges)
            
            if not (math.isinf(f)) and f < 4:
                x_range = f * math.cos(CollectLidar.cur_t)
                y_range = f * math.sin(CollectLidar.cur_t)
                object_pixelx = int((CollectLidar.cur_x - x_range)/ CollectLidar._M_PER_PIXEL)
                object_pixely = int((CollectLidar.cur_y - y_range)/ CollectLidar._M_PER_PIXEL)
                self._map[450-object_pixely, 450+object_pixelx] = 0
                self.get_logger().info(f"lidar ({f}, {CollectLidar.cur_t}, {x_range}, {y_range})")
            if not (math.isinf(d)) and d < 4:
                x_range = d * math.cos(CollectLidar.cur_t + (6*math.pi/4))
                y_range = d * math.sin(CollectLidar.cur_t + (6*math.pi/4))
                object_pixelx = int((CollectLidar.cur_x - x_range)/ CollectLidar._M_PER_PIXEL)
                object_pixely = int((CollectLidar.cur_y - y_range)/ CollectLidar._M_PER_PIXEL)
                self._map[450-object_pixely, 450+object_pixelx] = 0
                self.get_logger().info(f"lidar ({d}, {CollectLidar.cur_t}, {x_range}, {y_range})")
            if not (math.isinf(b)) and b < 4:
                x_range = b * math.cos(CollectLidar.cur_t + (2*math.pi/4))
                y_range = b * math.sin(CollectLidar.cur_t + (2*math.pi/4))
                object_pixelx = int((CollectLidar.cur_x - x_range)/ CollectLidar._M_PER_PIXEL)
                object_pixely = int((CollectLidar.cur_y - y_range)/ CollectLidar._M_PER_PIXEL)
                self._map[450-object_pixely, 450+object_pixelx] = 0
                self.get_logger().info(f"lidar ({d}, {CollectLidar.cur_t}, {x_range}, {y_range})")
 # ================================ RRT ======================   
        if self._RRT:
            object_pixelx = int(CollectLidar.cur_x / CollectLidar._M_PER_PIXEL)
            object_pixely = int(CollectLidar.cur_y / CollectLidar._M_PER_PIXEL)
            start_position = [450+object_pixelx, 450-object_pixely]
            self.get_logger().info(f"start ({start_position})")
            # Create an empty image
            image_size = 900
            self._map = np.ones((image_size, image_size, 3), dtype=np.uint8) * 255
            rrt = RRT(start_position)
            # Draw obstacles on the image
            draw_obstacles(self._map, rrt.obstacle_props)
            # Draw start and end points
            draw_point(self._map, rrt.start, (0, 255, 0))  # green for start
            draw_point(self._map, rrt.end, (0, 0, 255))    # red for end
            # Draw all nodes/edges
            for k in enumerate(rrt.nodes_list):
                draw_point(self._map, k[1][1], (255, 255, 0))  # yellow for nodes
                
                if k[0] > 0:
                    draw_line(self._map, rrt.segments_list[k[0] - 1][0], rrt.segments_list[k[0] - 1][1], (255, 255, 0))  # yellow for edges
                cv2.imshow('map', self._map)
                cv2.waitKey(20)
                
            # Draw path nodes/edges
            self._path = rrt.path_nodes
            for m in enumerate(rrt.path_nodes):
                draw_point(self._map, m[1], (0, 0, 255))  # red for path nodes
                if m[0] > 0:
                    draw_line(self._map, rrt.path_segments[m[0] - 1][0], rrt.path_segments[m[0] - 1][1], (0, 0, 255))  # red for path edges
                cv2.imshow('map', self._map)
                cv2.waitKey(20)
                
                
            cv2.imwrite("RRT_map.jpg",self._map)          
            self._RRT = False

        
        if self._callmap:
            self._map = cv2.imread("Occupancy_map.jpg")
            self._callmap = False
  
       
        if self._save:
            """
            self._map = cv2.imread("Occupancy_map.jpg")
            robot_radius = 0.18 # meters
            robot_pixelx = int(robot_radius / CollectLidar._M_PER_PIXEL)
            kernel = np.ones((2*robot_pixelx+1, 2*robot_pixelx+1), np.uint8)
            self._map = cv2.erode(self._map, kernel)
            self.get_logger().info(f"map ({self._map})")
            cv2.imwrite("Dilation_map.jpg",self._map)
            self._dilation = False
            """
          
        if self._scan_radioactive:
            self._state_machine()
	
        cv2.imshow('map',self._map)
        
# ===================================== drive to goal function ==================    
    def _drive_to_goal(self, goal_x, goal_y, goal_theta):
        self.get_logger().info(f'{self.get_name()} drive to goal')
        twist = Twist()

        x_diff = goal_x - CollectLidar.cur_x
        y_diff = goal_y - CollectLidar.cur_y
        dist = x_diff * x_diff + y_diff * y_diff
        self.get_logger().info(f'{self.get_name()} {x_diff} {y_diff}')

        # turn to the goal
        heading = math.atan2(y_diff, x_diff)
        
        if self._scan_radioacitiviy:
            object_pixelx = int(CollectLidar.cur_x / CollectLidar._M_PER_PIXEL)
            object_pixely = int(CollectLidar.cur_y / CollectLidar._M_PER_PIXEL)
            for i in range(int(self._row_offset*2 / CollectLidar._M_PER_PIXEL)):
                self._map[450-object_pixely, 450+object_pixelx+(-5+i)] = 0

        
        self.get_logger().info(f'Heading: {heading}, cur_t: {CollectLidar.cur_t}, diff_angle: {abs(CollectLidar.cur_t - heading)}')
        if abs(CollectLidar.cur_t - heading) > math.pi/20: 
            if heading > 18*math.pi/20:
                twist.linear.x = 0.2
            else:
                if heading > CollectLidar.cur_t:
                    twist.angular.z = 0.2
                else:
                   twist.angular.z = -0.2
            self.get_logger().info(f'{self.get_name()} turning towards goal')
            self._publisher.publish(twist)
            return False

        # pointing the right direction, so go there
        if dist > 0.1*0.1:
            twist.linear.x = 0.5
            self._publisher.publish(twist)
            self.get_logger().info(f'{self.get_name()} driving to goal')
            return False

        # we are there, set the correct angle
        
        if abs(goal_theta - CollectLidar.cur_t) > math.pi/20: 
            if goal_theta > CollectLidar.cur_t:
                twist.angular.z = 0.1
            else:
                twist.angular.z = -0.1
            self.get_logger().info(f'{self.get_name()} turning to goal direction')
            self._publisher.publish(twist)
           
        self.get_logger().info(f'{self.get_name()} at goal pose')
        return True
        
# ===================================== Start state ==================
       
    def _do_state_at_start(self):
        self.get_logger().info(f'{self.get_name()} in start state')
        if self._time_measure:
            self._start_time = self.get_clock().now().nanoseconds * 1e-9
            self._time_measure = False
        now = self.get_clock().now().nanoseconds * 1e-9
        if now > (self._start_time + 2):
            self._cur_state = FSM_STATES.HEADING_TO_TASK
            self._time_measure = True
            
# ===================================== Heading to task ==================

    def _do_state_heading_to_task(self):
        vel_gain=3
        max_vel=0.3
        max_pos_err= 0.1
        max_angle_error = 22.5 * np.pi/180
             
        goal_x_pix = self._path[self._i][0] - 450
        goal_y_pix = 450 - self._path[self._i][1]
        goal_x = goal_x_pix * CollectLidar._M_PER_PIXEL
        goal_y = goal_y_pix * CollectLidar._M_PER_PIXEL
        self.get_logger().info(f"goal at ({goal_x}, {goal_y})")
        self.get_logger().info(f"current position at ({CollectLidar.cur_x}, {CollectLidar.cur_y}, {CollectLidar.cur_t})")
        x_diff = goal_x - CollectLidar.cur_x
        y_diff = goal_y - CollectLidar.cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        angle = math.atan2(y_diff, x_diff)
        #self.get_logger().info(f"error distance between goal and current position ({dist})")
        t_diff = angle - CollectLidar.cur_t

        twist = Twist()
        twist.angular.z = t_diff
        if np.abs(t_diff) < max_angle_error:
            if dist > max_pos_err:
                twist.linear.x = max((min((dist) * vel_gain, max_vel)), -max_vel)
                self.get_logger().info(f"at ({CollectLidar.cur_x},{CollectLidar.cur_y},{CollectLidar.cur_t}) goal ({goal_x},{goal_y})")
            else:
                if self._i < len(self._path) - 1:
                    self._i = self._i + 1
                else:
                    self._cur_state = FSM_STATES.SCAN_RADIOACTIVITY
                    
        self.get_logger().info(f"Node number: ({self._i})")    
        self._publisher.publish(twist)
        self.get_logger().info(f'{self.get_name()} heading to task ')

# ===================================== Scan radioacitivity ==================
    def _do_state_scanning_radioactivity(self):
        self.get_logger().info(f'{self.get_name()} mowing lawn to {CollectLidar.cur_x} {CollectLidar.cur_y} {CollectLidar.cur_t}')
        self.get_logger().info(f'row offset is {self._row_offset}')
        goal_x_pix = self._path[self._i][0] - 450
        goal_y_pix = 450 - self._path[self._i][1]
        goal_x = goal_x_pix * CollectLidar._M_PER_PIXEL
        goal_y = goal_y_pix * CollectLidar._M_PER_PIXEL
        if self._current_row <= self._final_row and not self._move_to_offset:
            self._scan_radioacitiviy = True
            self.get_logger().info(f'current row is {self._current_row}')
            if self._current_row % 2 == 1:
                if self._drive_to_goal(goal_x + (self._current_row * self._row_offset), goal_y + self._row_length, math.pi):
                    self._move_to_offset = True
                    self._current_row += 1
            else:
                if self._drive_to_goal(goal_x + (self._current_row * self._row_offset), goal_y, math.pi):
                    self._move_to_offset = True
                    self._current_row += 1
            
        elif self._current_row <= self._final_row and self._move_to_offset:
            self._scan_radioacitiviy = False
            self.get_logger().info(f'Go to next row is {self._current_row}')
            if self._current_row % 2 == 0:
                if self._drive_to_goal(goal_x + (self._current_row * self._row_offset), goal_y + self._row_length, -math.pi/2):
                    self._move_to_offset = False
                    
            else:
                if self._drive_to_goal(goal_x + (self._current_row * self._row_offset), goal_y, math.pi/2):
                    self._move_to_offset = False
                
        else:
            self.get_logger().info(f'Finish mowing lawn and comback to initial position')
            if self._drive_to_goal(goal_x, goal_y, -math.pi/2):
                self._cur_state = FSM_STATES.RETURNING_FROM_TASK

# ======================== Returning_from_task =================================     
    def _do_state_returning_from_task(self):
        vel_gain=3
        max_vel=0.3
        max_pos_err= 0.1
        max_angle_error = 22.5 * np.pi/180
             
        goal_x_pix = self._path[self._i][0] - 450
        goal_y_pix = 450 - self._path[self._i][1]
        goal_x = goal_x_pix * CollectLidar._M_PER_PIXEL
        goal_y = goal_y_pix * CollectLidar._M_PER_PIXEL
        self.get_logger().info(f"goal at ({goal_x}, {goal_y})")
        self.get_logger().info(f"current position at ({CollectLidar.cur_x}, {CollectLidar.cur_y}, {CollectLidar.cur_t})")
        x_diff = goal_x - CollectLidar.cur_x
        y_diff = goal_y - CollectLidar.cur_y
            
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        angle = math.atan2(y_diff, x_diff)
        self.get_logger().info(f"error distance between goal and current position ({dist})")
        t_diff = angle - CollectLidar.cur_t

        twist = Twist()
        twist.angular.z = t_diff
        if np.abs(t_diff) < max_angle_error:
            if dist > max_pos_err:
                twist.linear.x = max((min((dist) * vel_gain, max_vel)), -max_vel)
                self.get_logger().info(f"at ({CollectLidar.cur_x},{CollectLidar.cur_y},{CollectLidar.cur_t}) goal ({goal_x},{goal_y})")
            else:
                if self._i > 0:
                    self._i = self._i - 1
                else:
                    self._cur_state = FSM_STATES.TASK_DONE
        self.get_logger().info(f"Node number: ({self._i})")    
        self._publisher.publish(twist)
        self.get_logger().info(f'{self.get_name()} returning from task ')
        
# ============================ Task Done =======================================
    def _do_state_task_done(self):
        self.get_logger().info(f'{self.get_name()} task done')
        if self._time_measure:
            self._start_time = self.get_clock().now().nanoseconds * 1e-9
            self._time_measure = False
        now = self.get_clock().now().nanoseconds * 1e-9
        if now > (self._start_time + 2):
            self._cur_state = FSM_STATES.AT_START
            self._time_measure = True
            self._scan_radioactive = False
            self.get_logger().info(f'Finish scaning radioactivity process')

 
# =========================== State Machine ===================================               
    
    def _state_machine(self):
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.HEADING_TO_TASK:
            self._do_state_heading_to_task()
        elif self._cur_state == FSM_STATES.SCAN_RADIOACTIVITY:
            self._do_state_scanning_radioactivity()
        elif self._cur_state == FSM_STATES.RETURNING_FROM_TASK:
            self._do_state_returning_from_task()
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()
        else:
            self.get_logger().info(f'{self.get_name()} bad state {state_cur_state}')

    def _odom_callback(self, msg):
        pose = msg.pose.pose

        CollectLidar.cur_x = pose.position.x
        CollectLidar.cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        CollectLidar.cur_t = yaw
        
        #self.get_logger().info(f"at ({cur_x},{cur_y},{cur_t})")

def main(args=None):
    rclpy.init(args=args)
    node = CollectLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

