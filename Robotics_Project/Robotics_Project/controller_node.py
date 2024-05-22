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

from rosgraph_msgs.msg import Clock



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
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

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
        self.NORMAL_SPEED = 0.05



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
        self.speed = self.NORMAL_SPEED   # speed
        self.angle_rot = 0.0        # angle
        self.orientation = "N"      # orienttation

        # flag for our FSM
        self.go_back = 0            # this is a flag to move in case of EXPLORARION AND DEADEND
                                    # whit this we stop thymio to move rotate of 180 deg and prepared to go into
                                    # next node to explore
        self.curve = 0.0            # this is a flag to move right 
        self.curve_state = 0        # curve state are 2 substate of this state we can have start to rotate and during rotation
                                    # during rotation we use the camera vision to understan if we are rotate or not
        self.next_direction = None  # self.next_direction
        self.flag_angle_See = None  # we need to se id line are correct or not

        self.STATE = 0              
                                    # STATE -1 == STOP END EXPLORING
                                    # STATE 0 == EXPLORE
                                    # STATE 1 == EXPLORE but we are in front of an explored node --> we need to enter do 3 sec go forward
                                    # STATE 2 == we are rotate for do a curve, we need to go back to see next node
                                    # STATE 3 == No sample we go direct on a wall
                                    # STATE 4 == DEAD_END
                                    # STATE 5 == DEAD_END 180 DEG

        self.FALG_TMP = -1          # state tmp to convert toreal state
                                    # STATE 999 == GO next node to explore cambia pure numero
        self.timer_05 = None        # prev state time we can use for compute a fake distance 
        self.timer_02 = None        # timer to go back in state 2
        self.timer_03 = None        # timer to correct the prev time if we curve
        self.timer_04 = None        # Needed to know when to turn if when going to the next unexplored node there isn't a wall to use for turning
        self.timer_01 = None
        self.timer_06 = None        # timer to rotate in state 996
        self.timer_07 = None        # go back case we are in state 3 in a curve
        self.time_until_Stop = None

        self.prev_state = (0,0)     # use odometry to compute distance in meters
        self.current_ste = (0,0)


        # STESTE GO NEXT TO EXPLORE
        self.path_to_follow = []            # path to follow 
        self.next_node_targ = [-1, "X"]     # next node

        self.continue_same_direction = False
        self.restart_explore = False


        self.PREV_STATE = -1
        self.FLAG_RESTART_EXPLORATION = False       # if we are back to exploration is true used to change imag to avoid problem

        self.PREV_BACK = 0.0               # if we move after hit wall we have no same time to move that if we go back
        self.PREV_BACK2 = 0.0               # if we move after hit wall we have no same time to move that if we find new node



        # TIMER OF SIMULATIONS
        self.current_time = 0.0
        self.TIMER_01_SIM = None        # FOR STATE 5 180 deg rotation 
        self.TIMER_PREV_D = 0.0         # FOR compute distance



        self.TIMER_PREV_D2 = 0.0         # FOR compute distance 
        self.TIMER_997 = None


        




    def clock_callback(self, msg):
        second = msg.clock.sec
        nanoseconds = msg.clock.nanosec
        second + nanoseconds / 1e9
        self.current_time = second + nanoseconds / 1e9


    def listener_callback(self, msg):
        
        # WHAT CAMERA SEE IFF WE ARE IN EXPLORATION
        if self.STATE == 0:

            # if we need to rotate --> no information of camera are needed
            if self.go_back > 0 or self.STATE == 3:
                self.sample_next_node.append(None)
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
            
            if self.FALG_TMP == 1:
                return

            image_tmp = cv_image.copy()

            if self.FLAG_RESTART_EXPLORATION:
                image_tmp = image_tmp[:, int(image_tmp.shape[1]*0.1):int(image_tmp.shape[1]*0.9)]


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
                    i -= 1
                
                self.sample_next_node = [None]

                if flag:

                    self.FLAG_RESTART_EXPLORATION = False

                    max_key = max(dict_count, key=dict_count.get)
                    max_key = eval(max_key)
                                        
                        
                    
                    # If dead end we activate flag to rotate!
                    if max_key  == [0,1,0,0]:
                        self.STATE = 4
                    elif max_key[0] == 0:
                        self.STATE = 3


                    # correct array form [N,S,E,O] we have another angolation and the array is build assuming we focus North for semplicity
                    max_key = swap_dir_array(max_key, self.orientation)

                    # Generate new node and update information of the graph
                    new_node_id = 1 + self.max_node_id
                    distance = 0.0
                    curr_time = time.time()
                    if self.prev_node_id >= 0:
                        distance = (curr_time - self.old_time)
                        distance = (self.current_time - self.TIMER_PREV_D)/4        # NOTE this is the distance using timer of coppelia != real time
                        #print("A ",self.current_time, self.TIMER_PREV_D)
                        distance = max(distance, 3.0)
                        distance += self.PREV_BACK2                             # add time if before we hit wall we have less time tha normal
                        distance = round(distance / 3) 
                        is_close_a_node = self.graph.exist_closed_node(self.prev_node_id,self.orientation,distance,max_key, threshold=0.1)
                        # if the node is already visited stop and go next node
                        if is_close_a_node is not None:
                            self.graph.add_edge(self.prev_node_id , is_close_a_node, self.orientation, distance)
                            self.prev_node_id = is_close_a_node
                            self.STATE = 1
                            self.PREV_BACK2 = 0.0                               # reset time
                            self.timer_05 = time.time()
                            return
                        
                        self.graph.add_edge(self.prev_node_id , new_node_id, self.orientation, distance)
                    else:
                        self.graph.add_node(new_node_id, 0, 0)
                    self.prev_node_id = new_node_id
                    self.max_node_id = new_node_id
                    self.old_time = curr_time

                    self.TIMER_PREV_D = self.current_time
                    
                    self.prev_state = self.current_ste
                    self.graph.node_information(new_node_id, max_key)
                    self.PREV_BACK2 = 0.0                               # reset time
        
        elif self.STATE == 2 or self.STATE == 5:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.flag_angle_See = get_pendence(cv_image)

        elif self.STATE == 997 or self.STATE == 998 or self.STATE == 998_2:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.flag_angle_See = get_pendence(cv_image)
        
        


# ##############################################
# ######### SENSORI PROSSIMITY #################
# ##############################################

    def left_callback(self, msg):
        self.left_obj = msg.range

    def right_callback(self, msg):
        self.right_obj = msg.range

    def rear_l_callback(self,msg):
        self.back_l = msg.range

        # if we go back and we are in the state 996 we set PREV_BACK to correct distance
        if self.STATE == 996 and (self.back_r == -1 or self.back_l ==-1):
            self.PREV_BACK  = 0.0
        
        if self.STATE == 2 and (self.back_r == -1 or self.back_l ==-1):
            self.PREV_BACK2  = 1.0
        

    def rear_r_callback(self,msg):
        self.back_r = msg.range

        
                

    
    def center_callback(self, msg):
        self.center_obj = msg.range

        if self.STATE == 3 and  self.center_obj > 0 and self.center_obj < 0.5:
            self.STATE = 0
            self.FALG_TMP = 1
        
        # Exploration and hit a wall --> Dead end or we need to curve
        if self.STATE == 0:
            if self.center_obj > 0 and self.center_obj < 0.3:
                self.angle_rot = 0.0
                self.speed = 0.0
                if self.go_back == 1:
                    if self.center_obj < 0.05:
                        self.go_back = 2
                        self.speed = 0.0
                    else:
                        # if we are in 0.15 we can't use back sensore
                        self.speed = 0.02


                elif self.go_back == 0:
                    next_dir = self.graph.get_direction_unexplored(self.prev_node_id, self.orientation)
                    if next_dir:
                        self.curve = get_angle(self.orientation, next_dir)
                        self.next_direction = next_dir
                    else:
                        self.STATE = 999

        elif self.STATE == 4:
            if self.center_obj > 0 and self.center_obj < 0.05:
                self.speed = 0.0
                self.STATE = 5
                self.timer_01 = time.time()
                #LJKHDSK

        elif self.STATE == 998 or self.STATE == 996:
             if self.center_obj > 0 and self.center_obj < 0.3:
                self.angle_rot = 0.0
                self.speed = 0.0
                self.curve = get_angle(self.orientation, self.next_direction)
                self.STATE = 997

        elif self.STATE == 998_2:
            if self.center_obj > 0 and self.center_obj < 0.3:
                self.angle_rot = 0.0
                self.speed = 0.0

            





# #########################################
# #########################################
# #########################################
        

            

    def start(self):
        plt.show(block=False)
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        self.current_ste = (self.odom_pose.position.x, self.odom_pose.position.y)
        
    
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
                cmd_vel.angular.z = 1.0/2 * self.curve

                # if camera see using camera information else curve using fixed value
                if self.flag_angle_See:
                    cmd_vel.angular.z = -self.flag_angle_See*2.0

                if self.timer_07 is None:
                    self.timer_07 = time.time()

                # if we are in rotation we  check if the camera vision tell us we are correct 90 deg rotate or not
                # if yes stop rotation if no remain in state rotation
                if ( time.time() - self.timer_07 > 3) or (self.curve_state == 1 and (self.flag_angle_See is not None and abs(self.flag_angle_See) < 0.01)):
                    self.flag_angle_See = None
                    self.curve = 0.0
                    self.speed = self.NORMAL_SPEED
                    self.orientation = self.next_direction
                    self.next_direction = None
                    self.STATE = 2
                    self.FALG_TMP = -1
                    self.timer_07 = None

                self.curve_state = 1


        elif self.STATE == 1:

            cmd_vel.linear.x  = self.speed 
            if time.time() - self.timer_05 >=3:
                self.STATE = 999
                self.speed = 0.0

        elif self.STATE == 2:
            self.speed  = self.NORMAL_SPEED
            cmd_vel.linear.x  = -self.speed
            # FLAG CIAO
            if self.flag_angle_See is not None:
                cmd_vel.angular.z = -self.flag_angle_See*5.0
            if self.timer_02 is None:
                self.timer_02 = time.time()
            if time.time() - self.timer_02 > 2:
                self.STATE = 0
                self.old_time = time.time() - 1
                self.TIMER_PREV_D = self.current_time - 4
                self.timer_02 = None

        elif self.STATE == 3 :
            cmd_vel.linear.x  = self.speed 

        elif self.STATE == 4 :
            self.speed = self.NORMAL_SPEED
            cmd_vel.linear.x  = self.speed 
            

        # STATE:
        # IN FRONT TO DEAD END AND WE NEED TURN 180 DEG

        elif self.STATE == 5:
            # This is base data
            cmd_vel.linear.x  = 0.0
            cmd_vel.angular.z = -1/4
        
            # Start timer if we are entered in this state
            if self.TIMER_01_SIM is None:
                self.TIMER_01_SIM = self.current_time

            # we need a base of X time to start using camero to coordinate rotation (else we see the wall in fron and we cant perfonm 180 deg rotation)

            time_difference = self.current_time - self.TIMER_01_SIM
            if time_difference  > 10:
                cmd_vel.angular.z = -1/10
                # if we see sometings
                if self.flag_angle_See is not None:
                    cmd_vel.angular.z = - self.flag_angle_See*5.0

                # we also use a max time and start next state to avoid noise camera that can block our thymio 
                if (self.flag_angle_See is not None and abs(self.flag_angle_See) < 0.01) or time_difference  > 30:
                    self.STATE = 6
                    self.TIMER_01_SIM = None





        elif self.STATE == 6 : 
            self.speed = self.NORMAL_SPEED
            cmd_vel.linear.x  = -self.speed  

            if self.timer_02 is None:
                self.timer_02 = time.time()

            if time.time() - self.timer_02 > 2:
                self.STATE = 999
                self.old_time = time.time() - 1
                self.TIMER_PREV_D = self.current_time - 4
                self.timer_02 = None
                opposite_direction = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}[self.orientation]
                self.orientation = opposite_direction


        # stato 4 set path
        # SET THE NEXT DIRECTION FROM WALL
        elif self.STATE == 999:
            if len(self.path_to_follow) == 0:
                target = self.graph.closest_unexplored(self.prev_node_id)                   # form Node_id, Direction
                if target is None:
                    self.STATE = -1
                    self.speed = 0.0
                    
                self.path_to_follow = self.graph.get_path(self.prev_node_id ,target[0])
                
            #print(self.graph.print_graph_code_to_build())
            if len(self.path_to_follow) > 0:


                flag_last_same_direction = False

                if len(self.path_to_follow) == 2:
                        self.restart_explore  = True
                        self.prev_node_id = self.path_to_follow[0]
                        self.next_direction = self.graph.get_direction_unexplored(self.path_to_follow[1], self.orientation) 

                        flag_last_same_direction = True #self.next_direction == self.orientation

                        self.STATE = 998_2

                        self.timer_04 = time.time()
                        self.TIMER_PREV_D2 = self.current_time
                            
                        node_a = self.graph.get_node(self.path_to_follow[0])
                        node_b = self.graph.get_node(self.path_to_follow[1])                    
                        self.time_until_Stop = max(abs(node_a.position[0]- node_b.position[0]), abs(node_a.position[1]- node_b.position[1]))*3 
                


                else:
                    self.next_node_targ = None
                    idx = 0

                
                    # TODO caso ultimo stessa direzione
                    tmp_node = None
                    while idx < len(self.path_to_follow):
                        
                        if self.prev_node_id == self.path_to_follow[idx]:
                            tmp_node = self.path_to_follow.pop(0)
                            continue
                        
                        next_direction = self.graph.get_direction_node_to_node(self.path_to_follow[idx], self.path_to_follow[idx+1])
                        # case id ==
                        if next_direction is None:
                            tmp_node = self.path_to_follow.pop(0)
                            continue
                        elif next_direction == self.orientation:
                            tmp_node = self.path_to_follow.pop(0)
                            continue
                        else:
                            self.next_direction = next_direction
                            self.STATE = 998
                            break
                    
                    node_a = self.graph.get_node(self.prev_node_id)
                    node_b = self.graph.get_node(self.path_to_follow[0])
                    if not flag_last_same_direction and node_b.neighbors[self.orientation][0] != 'X':
                        self.STATE = 998_2
                        self.timer_04 = time.time()
                        self.TIMER_PREV_D2 = self.current_time
                        self.time_until_Stop = max(abs(node_a.position[0]- node_b.position[0]), abs(node_a.position[1]- node_b.position[1]))*3 
                    

            self.speed  = self.NORMAL_SPEED
            cmd_vel.linear.x  = self.speed 

        # VAI A SBATTERE FRONTALE 
        elif self.STATE == 998:
            cmd_vel.linear.x  = self.speed 
            if self.flag_angle_See:
                cmd_vel.angular.z = -self.flag_angle_See*2.0
        
        elif self.STATE == 998_2:
            cmd_vel.linear.x  = self.speed 
            if self.flag_angle_See:
                cmd_vel.angular.z = -self.flag_angle_See*2.0
            
            # same thing to front wall
            # NOTA treshold 0.2 for precision
            # if time.time() - self.timer_04 >= self.time_until_Stop + 0.5 + self.PREV_BACK:
            if (self.current_time - self.TIMER_PREV_D2)/4 >= self.time_until_Stop + self.PREV_BACK:
                self.STATE = 997
                self.timer_04 = None
                self.TIMER_PREV_D2 = None
                self.angle_rot = 0.0
                self.speed = 0.0
                self.curve = get_angle(self.orientation, self.next_direction)
                if self.orientation == self.next_direction:
                    self.curve = 0.0

                self.PREV_BACK = 0.0

                self.STATE = 997

            
        # TI GIRI
        elif self.STATE == 997 and False:
            
            cmd_vel.angular.z = 1.0/2 * self.curve


            if self.timer_06 is None:
                self.timer_06 = time.time()

            if self.flag_angle_See and time.time() - self.timer_06 > 0.5:
                cmd_vel.angular.z = - self.flag_angle_See*5.0

            if time.time() - self.timer_06 > 3 or (self.curve_state == 1 and (self.flag_angle_See is not None and abs(self.flag_angle_See) > 0.02) and time.time() - self.timer_06 > 1):
                self.curve_state = 0
                self.flag_angle_See = None
                self.curve = 0.0
                self.speed = self.NORMAL_SPEED
                self.orientation = self.next_direction
                self.next_direction = None
                self.timer_06 = None
                self.STATE = 996
                self.PREV_BACK = 0.0
            
            self.curve_state = 1

        
        elif self.STATE == 997 and True:
            # This is base data
            cmd_vel.linear.x  = 0.0
            cmd_vel.angular.z = 1/2* self.curve
        
            # Start timer if we are entered in this state
            if self.TIMER_997 is None:
                self.TIMER_997 = self.current_time

            # we need a base of X time to start using camero to coordinate rotation (else we see the wall in fron and we cant perfonm 180 deg rotation)

            time_difference = self.current_time - self.TIMER_997
            waiting_time = 5
            if time_difference  > 2:
                cmd_vel.angular.z = 1/10 * self.curve
                # if we see sometings
                if self.flag_angle_See is not None:
                    cmd_vel.angular.z = - self.flag_angle_See*5.0
            
                # we also use a max time and start next state to avoid noise camera that can block our thymio 
                if (self.flag_angle_See is not None and abs(self.flag_angle_See) < 0.01) or time_difference  > 7:
                    self.curve_state = 0
                    self.flag_angle_See = None
                    self.curve = 0.0
                    self.speed = self.NORMAL_SPEED
                    self.orientation = self.next_direction
                    self.next_direction = None
                    self.TIMER_997 = None
                    self.STATE = 996
                    self.PREV_BACK = 0.0





        # ALLINEAMENTO MURO all'indietro
        elif  self.STATE == 996:
            self.speed  = self.NORMAL_SPEED
            cmd_vel.linear.x  = -self.speed
            # FLAG CIAO
            if self.flag_angle_See is not None:
                cmd_vel.angular.z = -self.flag_angle_See*5.0
            if self.timer_02 is None:
                self.timer_02 = time.time()

            if time.time() - self.timer_02 > 2 or (self.continue_same_direction and time.time() - self.timer_02 > 1.5):
                
                # restart exploration
                if self.restart_explore :

                    self.prev_node_id = self.path_to_follow[-1]
                    self.time_until_Stop = None
                    self.path_to_follow = []           
                    self.next_node_targ = [-1, "X"] 
                    self.timer_02 = None  
                    self.timer_03 = None        
                    self.timer_04 = None        
                    self.time_until_Stop = None
                    self.curve = 0.0
                    self.PREV_BACK = 0.0

                    self.STATE = 0
                    self.speed = self.NORMAL_SPEED
                    self.restart_explore = False
                    self.FLAG_RESTART_EXPLORATION = True

                    self.old_time = time.time()-2
                    self.TIMER_PREV_D = self.current_time - 10
                    if self.continue_same_direction:
                        self.old_time += 1
                        self.TIMER_PREV_D += 8
                    
                
                else:
                    self.STATE = 999
                    tmp_node = self.prev_node_id 
                    self.prev_node_id = self.path_to_follow[0]
                    self.timer_02 = None

                    if self.graph.get_direction_unexplored(self.prev_node_id, self.orientation):
                        self.next_direction = self.graph.get_direction_unexplored(self.prev_node_id, self.orientation)
                        self.STATE = 998_2
                        self.timer_04 = time.time()
                        node_a = self.graph.get_node(tmp_node)
                        node_b = self.graph.get_node(self.prev_node_id)
                        self.time_until_Stop = max(abs(node_a.position[0]- node_b.position[0]), abs(node_a.position[1]- node_b.position[1]))*3

        # self.graph.print_nodes()
        
        self.vel_publisher.publish(cmd_vel)

        if self.PREV_STATE != self.STATE:
            print( self.STATE)
            self.PREV_STATE = self.STATE
    





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
    node.stop()


if __name__ == '__main__':
    main()
