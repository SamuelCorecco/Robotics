import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys
import math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt


import time
from Robotics_Project.camera_functions import plot_img_with_line2, select_type, check_if_sample,get_pendence
from Robotics_Project.graph import Node_graph, Graph
from Robotics_Project.utility_robo import swap_dir_array,get_angle

# ros2 launch Robotics_Project controller.launch.xml thymio_name:=thymio0


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.odom_pose = None
        self.odom_velocity = None
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
    
        # SENSORS
        self.sensor_center_sub = self.create_subscription(Range, '/thymio0/proximity/center', self.center_callback, 10)
        self.sensor_center_left_sub = self.create_subscription(Range, '/thymio0/proximity/center_left', self.left_callback, 10)
        self.sensor_center_right_sub = self.create_subscription(Range, '/thymio0/proximity/center_right', self.right_callback, 10)
        self.sensor_rear_left_sub = self.create_subscription(Range, '/thymio0/proximity/rear_left', self.rear_l_callback, 10)
        self.sensor_rear_right_sub = self.create_subscription(Range, '/thymio0/proximity/rear_right', self.rear_r_callback, 10)

        # Poistion of object from the sensors
        self.left_obj = -1
        self.right_obj = -1
        self.center_obj = -1
        self.back_l = -1
        self.back_r = -1
        
        # thresholds
        self.threshoold = 0.01      # this threshold is for make the rotation of 90 deg



        # Graph information
        self.graph = Graph()                # this is the Graph class
        self.prev_node_id = -1              # node id to trak our position in the graph
        self.max_node_id = -1               # node id counter
        self.old_time = -1                  # time used to mesure the distance form different nodes
        self.sample_next_node = [None]      # an array to save history of the observed type of nodes


        # Vision information like camera ecc
        self.subscription = self.create_subscription( Image, '/thymio0/camera', self.listener_callback, 10)
        self.subscription  
        self.bridge = CvBridge()
        self.VERTICAL_THRESHOLD = np.pi/180 * 10


        # controller for robot
        self.speed = 0.05           # speed
        self.angle_rot = 0.0        # angle
        self.orientation = "E"      # orienttation

        # flag for our FSM
        self.go_back = 0            # this is a flag to move in case of EXPLORARION AND DEADEND
                                    # whit this we stop thymio to move rotate of 180 deg and prepared to go into
                                    # next node to explore
        self.curve = 0.0            # this is a flag to move right 
        self.curve_state = 0        # curve state are 2 substate of this state we can have start to rotate and during rotation
                                    # during rotation we use the camera vision to understan if we are rotate or not
        self.next_direction = None  # self.next_direction
        self.flag_angle_See = None  # we need to se id line are correct or not

        self.STATE = 0              # STATE 0 == EXPLORE
                                    # STATE 1 == EXPLORE but we are in front of an explored node --> we need to enter do 3 sec go forward
                                    # STATE 2 == we are rotate for do a curve, we need to go back to see next node
                                    # 
                                    # STATE 999 == GO next node to explore cambia pure numero
        self.timer_05 = None
        self.timer_02 = None        # timer to go back in state 2





    def listener_callback(self, msg):
        
        # WHAT CAMERA SEE IFF WE ARE IN EXPLORATION
        if self.STATE == 0:

            # if we need to rotate --> no information of camera are needed
            if self.go_back > 0:
                return
            
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # curve we need only pendende to understand wen we are corret copistion
            if self.curve != 0.0:
                self.flag_angle_See = get_pendence(cv_image)
                return
                

            image_tmp = cv_image.copy()

            # STEP 1:   get 2 information from frst check:
            #           a) if we have a ground line orizzontal in the correct place --> we can sample the next node direction
            #           b) whether we are straight or crooked during forward movement
            
            sample_flag, angle_to_move = check_if_sample(image_tmp)

            # if we need to correct angle we use information of slope of the straight line 
            # to get the angle correction 
            if angle_to_move != -1:
                self.angle_rot = -angle_to_move
            else: 
                self.angle_rot = 0.0
            


            image_tmp = cv_image.copy()

            # STEP 2 we have 2 case:    a) we need to sample the node type
            #                           b) no sample (we have 2 possibility doing nothing or create new node)
            # STEP 2 a:     get type of next node in [North, sout, Est, West] that are the neigbhoor 
            if sample_flag :
                value = select_type(image_tmp)
                # if any(value):
                if value != [0,0,0,0]:
                    self.sample_next_node.append(value)
    
            # STEP 2 b if we not None in the end of the list we generate a new node in graph with past informations
            # NOTE: None is a flag to divide the node from each other
            elif self.sample_next_node[-1]:

                # add None and count all type for next node
                # we choose only the max thype example 3 dead end and 1 curve right we choose dead end
                self.sample_next_node.append(None)
                flag = False
                i = len(self.sample_next_node)-2
                dict_count = {}
                while i >= 0 and self.sample_next_node[i]:
                    key_val = str(self.sample_next_node[i])
                    if key_val not in dict_count:
                        dict_count[key_val] = 1
                        flag = True
                    else:
                        dict_count[key_val] += 1
                    i -= 11
                if flag:
                    max_key = max(dict_count, key=dict_count.get)
                    max_key = eval(max_key)
                    
                    # If dead end we activate flag to rotate!
                    if max_key  == [0,1,0,0]:
                        self.go_back = 1

                    # correct array form [N,S,E,O] we have another angolation and the array is build assuming we focus North for semplicity
                    max_key = swap_dir_array(max_key, self.orientation)

                    # Generate new node and update information of the graph
                    new_node_id = 1 + self.max_node_id
                    distance = 0.0
                    curr_time = time.time()
                    if self.prev_node_id >= 0:
                        distance = (curr_time - self.old_time)
                        is_close_a_node = self.graph.exist_closed_node(self.prev_node_id,self.orientation,distance,max_key, threshold=2)
                        # if the node is already visited stop and go next node
                        if is_close_a_node is not None:
                            self.graph.add_edge(self.prev_node_id , is_close_a_node, self.orientation, distance)
                            self.prev_node_id = is_close_a_node
                            self.STATE = 1
                            self.timer_05 = time.time()
                            return
                        
                        self.graph.add_edge(self.prev_node_id , new_node_id, self.orientation, distance)
                    else:
                        self.graph.add_node(new_node_id, 0, 0)
                    self.prev_node_id = new_node_id
                    self.max_node_id = new_node_id
                    self.old_time = curr_time
                    self.graph.node_information(new_node_id, max_key)
                    #print("Node info: ", max_key)
        
        


# ##############################################
# ######### SENSORI PROSSIMITY #################
# ##############################################

    def left_callback(self, msg):
        self.left_obj = msg.range

    def right_callback(self, msg):
        self.right_obj = msg.range

    def rear_l_callback(self,msg):
        self.back_l = msg.range

    def rear_r_callback(self,msg):
        self.back_r = msg.range
        
        # Rotation for the dead end
        # this just align the back sensors
        if self.STATE == 0:
            if self.go_back == 2:
                print("AJHGJDSGHJDGASJGDJHGASD")
                diff_r_l = np.sign(self.back_r )*self.back_r - np.sign(self.back_l )*self.back_l
                if (self.back_r == -1 or self.back_l ==-1) or np.abs(diff_r_l) > self.threshoold:
                    self.angle_rot = 0.2
                    if (self.back_r != -1 or self.back_l !=-1):
                        self.angle_rot = -1/2*diff_r_l
                    self.threshoold = 0.003

                else:
                    self.angle_rot = 0
                    self.threshoold += 0.002        # this is done for better accuration
                    if self.threshoold > 0.02:
                        self.threshoold = 0.01      # reset the initial threshoold
                        self.STATE = 999
                        self.speed = 0.0 # todo cancella
        
                

    
    def center_callback(self, msg):
        self.center_obj = msg.range
        
        # Exploration and hit a wall --> Dead end or we need to curve
        if self.STATE == 0:
            if self.center_obj > 0 and self.center_obj < 0.15:
                self.angle_rot = 0.0
                self.speed = 0.0
                if self.go_back == 1:
                    if self.center_obj < 0.07:
                        self.go_back = 2
                        self.speed = 0.0
                    else:
                        # if we are in 0.15 we can't use back sensore
                        self.speed = 0.02
                elif self.go_back == 0:
                    next_dir = self.graph.get_direction_uneplored(self.prev_node_id)
                    if next_dir:
                        self.curve = get_angle(self.orientation, next_dir)
                        self.next_direction = next_dir





# #########################################
# #########################################
# #########################################
        

            

    def start(self):
        print("START")
        plt.show(block=False)
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
        

    def update_callback(self):
        cmd_vel = Twist() 

        if self.STATE == 0:
            cmd_vel.linear.x  = self.speed 
            cmd_vel.angular.z = 5.0 * (self.angle_rot)
            
            # case rotate right or left dureing Exploration
            if self.curve and self.curve != 0.0:
                # chekc 1 we have already or not the prev angle:
                cmd_vel.angular.z = 1.0 * self.curve

                # if camera see using camera information else curve using fixed value
                if self.flag_angle_See:
                    cmd_vel.angular.z = -self.flag_angle_See*2.0

                # if we are in rotation we  check if the camera vision tell us we are correct 90 deg rotate or not
                # if yes stop rotation if no remain in state rotation
                if self.curve_state == 1 and self.flag_angle_See == 0:
                    self.flag_angle_See = None
                    self.curve = 0.0
                    self.speed = 0.05
                    self.orientation = self.next_direction
                    self.next_direction = None
                    self.STATE = 2

                self.curve_state = 1

        elif self.STATE == 1:
            cmd_vel.linear.x  = self.speed 
            print(time.time() - self.timer_05)
            if time.time() - self.timer_05 >=3:
                self.STATE = 999
                self.speed = 0.0
        elif self.STATE == 2:
            self.speed  = 0.05
            cmd_vel.linear.x  = -self.speed 
            if self.timer_02 is None:
                self.timer_02 = time.time()
            if time.time() - self.timer_02 > 2:
                self.STATE = 0
                self.timer_02 = None

        self.vel_publisher.publish(cmd_vel)




def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    print("NO FRA")
    node.stop()


if __name__ == '__main__':
    main()
