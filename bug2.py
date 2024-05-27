import math
import numpy as np 
from time import sleep 
import rclpy
from rclpy.node import Node
import tf_transformations as tf_trans
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class Bug2Algorithm(Node):

    def __init__(self):
        super().__init__('bug2_algorithm')

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.update_position, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.update_leftfront_sensor, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.update_rightfront_sensor, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.update_goal, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.algorithm_timer = self.create_timer(0.1, self.algorithm_callback)

        self.speed_forward = 0.1
        self.speed_turn = 0.6
        self.speed_turn_adjust = 0.08
        self.speed_turn_fast = 0.8
        self.speed_turn_slow = 0.35

        self.yaw_tolerance = 2 * (math.pi / 180)
        self.goal_distance_tolerance = 0.1
        self.line_distance_tolerance = 0.1
        self.leave_hit_point_diff = 0.2

        self.obstacle_threshold = 0.5
        self.wall_follow_threshold = 0.45
        self.too_close_to_wall = 0.4
        self.bug2_threshold = 0.5

        self.dist_leftfront = 0.0
        self.dist_rightfront = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw_angle = 0.0

        self.goal_x = False
        self.goal_y = False
        self.goal_index = 0
        self.max_goal_index = None

        self.robot_mode = "go to goal mode"
        self.goal_state = "adjust heading"
        self.wall_follow_state = "turn left"
        self.bug2_status = "ON"
        self.line_calculated = False

        self.line_slope = 0
        self.line_intercept = 0
        self.line_start_x = 0
        self.line_goal_x = 0
        self.line_start_y = 0
        self.line_goal_y = 0


        self.hit_point_x = 0
        self.hit_point_y = 0
        self.leave_point_x = 0
        self.leave_point_y = 0
        self.dist_to_goal_from_hit = 0.0
        self.dist_to_goal_from_leave = 0.0
        self.counter = 0
        self.line_reached = 0
        self.msg_1 = ""
        self.msg_2 = ""
        self.msg_3 = ""

    def update_goal(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def update_position(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.yaw_angle = tf_trans.euler_from_quaternion(q)[2]

    def update_leftfront_sensor(self, msg):
        self.dist_leftfront = msg.range

    def update_rightfront_sensor(self, msg):
        self.dist_rightfront = msg.range

    def algorithm_callback(self):
        print("New Message")
        print(self.msg_1)
        print(self.msg_2)
        print(self.msg_3)
        print("")

        if self.robot_mode == "obstacle avoidance mode":
            self.avoid_obstacle()
        
        if self.goal_x == False and self.goal_y == False:
            return
        
        self.bug2_logic()

    def avoid_obstacle(self):
        self.msg_1 = "METHOD: avoid_obstacle"

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
         
        d = self.obstacle_threshold

        if   self.dist_leftfront > d and self.dist_rightfront > d:
            msg.linear.x = self.speed_forward

        elif self.dist_leftfront > d and self.dist_rightfront < d:
            msg.angular.z = self.speed_turn

        elif self.dist_leftfront < d and self.dist_rightfront > d:
            msg.angular.z = -self.speed_turn

        elif self.dist_leftfront < d and self.dist_rightfront < d:
            msg.angular.z = self.speed_turn

        else:
            pass
             
        self.cmd_vel_pub.publish(msg)

    def go_to_goal(self):
        self.msg_1 = "METHOD: go_to_goal"

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
    
        d = self.bug2_threshold
        d_2 = self.too_close_to_wall

        if (self.dist_leftfront < d or self.dist_rightfront < d_2):
            self.msg_3 = "MESSAGE: ROBOT ENCOUNTERED WALL"
            self.robot_mode = "wall following mode"
            self.msg_2 = "ROBOT_MODE = wall following mode"
            self.hit_point_x = self.pos_x
            self.hit_point_y = self.pos_y
            
            self.dist_to_goal_from_hit = math.sqrt(
                (pow(self.goal_x - self.hit_point_x, 2)) +
                (pow(self.goal_y - self.hit_point_y, 2)))  

            msg.linear.x = 0.0   
            msg.angular.z = self.speed_turn_fast + 1
                    
            self.cmd_vel_pub.publish(msg)
            return
                   
        if (self.goal_state == "adjust heading"):
            self.msg_3 = "MESSAGE: ROBOT ADJUSTING HEADING"
             
            desired_yaw = math.atan2(
                    self.goal_y - self.pos_y,
                    self.goal_x - self.pos_x)
                   
            yaw_error = desired_yaw - self.yaw_angle
             
            if math.fabs(yaw_error) > self.yaw_tolerance:
                if yaw_error > 0:          
                    msg.angular.z = self.speed_turn_adjust
                else:
                    msg.angular.z = -self.speed_turn_adjust
                 
                self.cmd_vel_pub.publish(msg)
                 
            else:               
                self.goal_state = "go straight"
                self.msg_2 = "ROBOT_MODE = go to goal // go straight"
                self.cmd_vel_pub.publish(msg)        
                                  
        elif (self.goal_state == "go straight"):
            position_error = math.sqrt(
                pow(self.goal_x - self.pos_x, 2) + 
                pow(self.goal_y - self.pos_y, 2))
                                            
            if position_error > self.goal_distance_tolerance:
                msg.linear.x = self.speed_forward
                self.cmd_vel_pub.publish(msg)
             
                desired_yaw = math.atan2(
                    self.goal_y - self.pos_y,
                    self.goal_x - self.pos_x)
                 
                yaw_error = desired_yaw - self.yaw_angle      
         
                if math.fabs(yaw_error) > self.yaw_tolerance:
                    self.goal_state = "adjust heading"
                    self.msg_2 = "ROBOT_MODE = go to goal // adjust heading"
                 
            else:           
                self.goal_state = "goal reached"
                self.msg_2 = "ROBOT_MODE = goal reached"
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)         

    def follow_wall(self):
        self.msg_1 = "METHOD: follow_wall"
        d = self.wall_follow_threshold

        if (self.dist_leftfront > d and self.dist_rightfront > d):
            self.wall_follow_state = "turn left"
        elif (self.dist_leftfront > d and self.dist_rightfront < d):
            self.wall_follow_state = "follow wall"
        elif (self.dist_leftfront < d and self.dist_rightfront > d):
            self.wall_follow_state = "turn right"
        elif (self.dist_leftfront < d and self.dist_rightfront < d):
            self.wall_follow_state = "turn right"
        else:
            pass

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if (self.wall_follow_state == "turn left"):
            msg.angular.z = self.speed_turn
            msg.linear.x = 0.0
            self.cmd_vel_pub.publish(msg)
        elif (self.wall_follow_state == "turn right"):
            msg.angular.z = -self.speed_turn
            msg.linear.x = 0.0
            self.cmd_vel_pub.publish(msg)
        elif (self.wall_follow_state == "follow wall"):
            msg.linear.x = self.speed_forward
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
        else:
            pass
         
    def bug2_logic(self):
        self.msg_1 = "METHOD: bug2_logic"
        if self.bug2_status == "ON" and self.line_calculated == False:
            self.line_start_x = self.pos_x
            self.line_goal_x = self.goal_x
            self.line_start_y = self.pos_y
            self.line_goal_y = self.goal_y
            self.line_slope = (self.line_goal_y - self.line_start_y) / (self.line_goal_x - self.line_start_x)
            self.line_intercept = self.line_goal_y - self.line_slope * self.line_goal_x

            self.line_calculated = True

        self.msg_2 = "STATUS = " + self.robot_mode

        if self.robot_mode == "go to goal mode":
            self.go_to_goal()
        elif self.robot_mode == "wall following mode":
            self.follow_wall()
            self.leave_point_check()

    def leave_point_check(self):
        current_distance = math.sqrt(
            (pow(self.goal_x - self.pos_x, 2)) +
            (pow(self.goal_y - self.pos_y, 2)))

        if current_distance < self.dist_to_goal_from_hit:
            distance_from_line = math.fabs((self.line_slope * self.pos_x) - self.pos_y + self.line_intercept) / math.sqrt((pow(self.line_slope, 2)) + 1)
            self.msg_3 = "dist_from_line = " + str(distance_from_line)

            if distance_from_line < self.line_distance_tolerance:
                self.line_reached += 1
                if self.line_reached > 15:
                    self.line_reached = 0

                    self.robot_mode = "go to goal mode"
                    self.msg_2 = "ROBOT_MODE = go to goal mode"

                    self.leave_point_x = self.pos_x
                    self.leave_point_y = self.pos_y

                    self.dist_to_goal_from_leave = math.sqrt(
                        (pow(self.goal_x - self.leave_point_x, 2)) +
                        (pow(self.goal_y - self.leave_point_y, 2)))

                    self.hit_point_x = 0
                    self.hit_point_y = 0

                    self.dist_to_goal_from_hit = 0.0

                    sleep(1.5)

def main(args=None):
    rclpy.init(args=args)
    node = Bug2Algorithm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

