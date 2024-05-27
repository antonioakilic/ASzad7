import math
import numpy as np 
from time import sleep 
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class Bug2(Node):
    def __init__(self):
        super().__init__('bug2')

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)

        self.forward_speed = 0.1
        self.turning_speed = 0.6

        self.turning_speed_yaw_adjustment = 0.08

        self.turning_speed_wf_fast = 0.8
        self.turning_speed_wf_slow = 0.35

        self.yaw_precision = 2 *(math.pi / 180)
        self.dist_precision = 0.1 # tol za cilj
        self.distance_to_start_goal_line_precision = 0.1

        self.leave_point_to_hit_point_diff = 0.2

        self.dist_thresh_obs = 0.5 # dist za trazenje prepreka
        self.dist_thresh_wf = 0.45 # dist na kojoj se prati zid
        self.dist_too_close_to_wall = 0.4 # dist di je robot pre blizu zida
        self.dist_thresh_bug2 = 0.5 # dist di robot dolazi do zida

        self.leftfront_dist = 0.0
        self.rightfront_dist = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.goal_x_coordinates = False
        self.goal_y_coordinates = False

        self.goal_idx = 0
        self.goal_max_idx =  None

        self.robot_mode = "go to goal mode"

        self.go_to_goal_state = "adjust heading"
        self.wall_following_state = "turn left"

        self.bug2_switch = "ON"
        self.start_goal_line_calculated = False

        self.start_goal_line_slope_m = 0
        self.start_goal_line_y_intercept = 0
        self.start_goal_line_xstart = 0
        self.start_goal_line_xgoal = 0
        self.start_goal_line_ystart = 0
        self.start_goal_line_ygoal = 0

        self.hit_point_x = 0
        self.hit_point_y = 0

        self.leave_point_x = 0
        self.leave_point_y = 0

        self.distance_to_goal_from_hit_point = 0.0
        self.distance_to_goal_from_leave_point = 0.0
        
        self.brojac = 0
        self.dodris_linija = 0

    def goal_callback(self, msg):
        self.goal_x_coordinates = msg.pose.position.x
        self.goal_y_coordinates = msg.pose.position.y

    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_yaw = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    def fl_sensor_callback(self, msg):
        self.leftfront_dist = msg.range

    def fr_sensor_callback(self, msg):
        self.rightfront_dist = msg.range

    def bug_algorithm_callback(self):
        if self.robot_mode == "obstacle avoidance mode":
            self.avoid_obstacles()
        
        if self.goal_x_coordinates == False and self.goal_y_coordinates == False:
            return
        
        self.bug2()

    def avoid_obstacles(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        d = self.dist_thresh_obs

        if   self.leftfront_dist > d and self.rightfront_dist > d:
            msg.linear.x = self.forward_speed   # forw

        elif self.leftfront_dist > d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed  # rot left

        elif self.leftfront_dist < d and self.rightfront_dist > d:
            msg.angular.z = -self.turning_speed # rot right

        elif self.leftfront_dist < d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed  # rot left

        else:
            pass
             
        self.cmd_pub.publish(msg)

    def go_to_goal(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
    
        d = self.dist_thresh_bug2
        d_2 = self.dist_too_close_to_wall

        if (self.leftfront_dist < d or self.rightfront_dist < d_2):   
            self.robot_mode = "wall following mode"

            self.hit_point_x = self.current_x
            self.hit_point_y = self.current_y
            
            self.distance_to_goal_from_hit_point = math.sqrt(
                (pow(self.goal_x_coordinates - self.hit_point_x, 2)) +
                (pow(self.goal_y_coordinates - self.hit_point_y, 2)))  

            msg.linear.x = 0.0   
            msg.angular.z = self.turning_speed_wf_fast +1 # rot left
                    
            self.cmd_pub.publish(msg)
            return
    
        if (self.go_to_goal_state == "adjust heading"):
            desired_yaw = math.atan2(
                    self.goal_y_coordinates - self.current_y,
                    self.goal_x_coordinates - self.current_x)     
            yaw_error = desired_yaw - self.current_yaw

            if math.fabs(yaw_error) > self.yaw_precision:
             
                if yaw_error > 0:          
                    msg.angular.z = self.turning_speed_yaw_adjustment   # rot left
                else:
                    msg.angular.z = -self.turning_speed_yaw_adjustment  # rot right
                 
                self.cmd_pub.publish(msg)

            else:               
                self.go_to_goal_state = "go straight"
                self.cmd_pub.publish(msg)        
                                     
        elif (self.go_to_goal_state == "go straight"):
             
            position_error = math.sqrt(
                pow(self.goal_x_coordinates - self.current_x, 2) + 
                pow(self.goal_y_coordinates - self.current_y, 2))
                       
            if position_error > self.dist_precision:
 
                msg.linear.x = self.forward_speed
                self.cmd_pub.publish(msg)

                desired_yaw = math.atan2(
                    self.goal_y_coordinates - self.current_y,
                    self.goal_x_coordinates - self.current_x)
  
                yaw_error = desired_yaw - self.current_yaw     
                if math.fabs(yaw_error) > self.yaw_precision:
                    self.go_to_goal_state = "adjust heading"

            else:           
                self.go_to_goal_state = "goal achieved"

                self.robot_mode = "done"
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
                raise ExitLoopException()
        
        elif (self.go_to_goal_state == "goal achieved"):
            self.start_goal_line_calculated = False            
         
        else:
            pass

    
    def follow_wall(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0   

        x_start_goal_line = self.current_x
        y_start_goal_line =(self.start_goal_line_slope_m *(x_start_goal_line)) + (self.start_goal_line_y_intercept)

        distance_to_start_goal_line = math.sqrt(
            pow(x_start_goal_line - self.current_x, 2) + 
            pow(y_start_goal_line - self.current_y, 2)) 
           
        if distance_to_start_goal_line < self.distance_to_start_goal_line_precision:
            self.leave_point_x = self.current_x
            self.leave_point_y = self.current_y

            self.distance_to_goal_from_leave_point = math.sqrt(
                pow(self.goal_x_coordinates - self.leave_point_x, 2) + 
                pow(self.goal_y_coordinates - self.leave_point_y, 2)) 
 
            diff = self.distance_to_goal_from_hit_point - self.distance_to_goal_from_leave_point

            if diff > self.leave_point_to_hit_point_diff:
                self.robot_mode = "go to goal mode"

                self.go_to_goal_state = "adjust heading"

                msg.linear.x = 0.0
                msg.angular.z = self.turning_speed_wf_fast + 2 # rot left
                    
                self.cmd_pub.publish(msg)

                return             
         
        d = self.dist_thresh_wf
        d_2 = self.dist_too_close_to_wall
         
        if self.leftfront_dist > d and self.rightfront_dist > d:
            # turn right
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow + 0.05
             
        elif (self.leftfront_dist > d and self.rightfront_dist < d):
            if (self.rightfront_dist < self.dist_too_close_to_wall):
                # turn left
                msg.linear.x = self.forward_speed
                msg.angular.z = self.turning_speed_wf_fast
            else:           
                msg.linear.x = self.forward_speed
                                     
        elif self.leftfront_dist < d_2 and self.rightfront_dist > d:
            msg.angular.z = self.turning_speed_wf_slow
        
        elif self.leftfront_dist < d and self.rightfront_dist > d:

            msg.linear.x = self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow + 0.15
              
        elif self.leftfront_dist < d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed_wf_slow

        else:
            pass

        self.cmd_pub.publish(msg)

    def bug2(self):
        if self.start_goal_line_calculated == False:
            self.robot_mode = "go to goal mode"            
 
            self.start_goal_line_xstart = self.current_x
            self.start_goal_line_xgoal = self.goal_x_coordinates
            self.start_goal_line_ystart = self.current_y
            self.start_goal_line_ygoal = self.goal_y_coordinates
            self.start_goal_line_slope_m = (
                (self.start_goal_line_ygoal - self.start_goal_line_ystart) / 
                (self.start_goal_line_xgoal - self.start_goal_line_xstart))
             
            self.start_goal_line_y_intercept = (
                self.start_goal_line_ygoal - (self.start_goal_line_slope_m * self.start_goal_line_xgoal))
            self.start_goal_line_calculated = True
             
        if self.robot_mode == "go to goal mode":
            self.go_to_goal()     

        elif self.robot_mode == "wall following mode":
            self.follow_wall()

        elif self.robot_mode == "done":
            pass

class ExitLoopException(Exception):
    pass

def main(args=None):
    try:
        rclpy.init(args=args)
        bug_node = Bug2()
        rclpy.spin(bug_node)
    except ExitLoopException:
        pass
    finally:
        bug_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()