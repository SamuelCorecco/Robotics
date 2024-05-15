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
from Robotics_Project.camera_functions import plot_img_with_line2, select_type, check_if_sample
from Robotics_Project.graph import Node_graph, Graph
from Robotics_Project.utility_robo import swap_dir_array

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

        self.left_obj = -1
        self.right_obj = -1
        self.center_obj = -1
        self.back_l = -1
        self.back_r = -1
        self.threshoold = 0.01

        self.graph = Graph()
        self.last_node_id = -1
        self.prev_node_id = -1
        self.old_time = -1

        self.subscription = self.create_subscription(
            Image,  # Usa Image o CompressedImage a seconda del tipo di messaggio
            '/thymio0/camera',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.VERTICAL_THRESHOLD = np.pi/180 * 10

        self.next_node_neig = [0,0,0,0]
        self.sample_next_node = [None]

        self.speed = 0.05
        self.angle_rot = 0.0
        self.orientation = "E"

        self.go_back = 0





    def listener_callback(self, msg):

        if self.go_back > 0:
            return
        
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_tmp = cv_image.copy()
        #plot_img_with_line2(cv_image)
        sample_flag, angle_to_move = check_if_sample(image_tmp)
        # check if no line detected on ground
        if angle_to_move != -1:
            self.angle_rot = -angle_to_move
        else: 
            self.angle_rot = 0.0

        image_tmp = cv_image.copy()


        if sample_flag:
            value = select_type(image_tmp)
            # if any(value):
            if value != [0,0,0,0]:
                self.sample_next_node.append(value)
            


        elif self.sample_next_node[-1]:
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

                if max_key  == [0,1,0,0]:
                    self.go_back = 1
                # TODO: correggere in base all'orientamento
                # TODO: gestire il prev node se ci ripassiamo
                print(max_key)
                max_key = swap_dir_array(max_key, self.orientation)

                self.last_node_id += 1
                distance = 0.0
                curr_time = time.time()
                if self.old_time > 0:
                    distance = (curr_time - self.old_time)
                    print("prev   ",self.prev_node_id , "   current  ",self.last_node_id, "   distance  ", distance, "  vicini: ",max_key)
                    self.graph.add_edge(self.prev_node_id , self.last_node_id, self.orientation, distance)
                else:
                    self.graph.add_node(self.last_node_id, 0, 0)

                self.prev_node_id = self.last_node_id
                self.old_time = curr_time
                self.graph.node_information(self.last_node_id, max_key)
            

            

    

        



        
        # TODO: quando ricevi la value il thymio punta sempre a NORD quindi devi aggiustare la lista che arriva in formato N S E O 
        # nel formato aggiornato quindi se timio punta a Sud swappi N e S e O e E ecc
        new_value = 0

        # TODO: aggiornare self.next_node_neig solo quando sei al centro della casella prima nel punto
        if False:
            self.next_node_neig = new_value

        # Ricorda questi valori li settererai poi quando arrivi alla casella
        # se sei circa una casella davanti ad A (circa nella meta prima) salvi i vicini che dice la camera
        # appena arrivi ad A esempio nell odometry fai 
        # graph.add_node(NAME_NODE, 0, 0)
        # graph.node_information(NAME_NODE, self.next_node_neig = new_value)
        
        


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
        
        if self.go_back == 2:
            diff_r_l = np.sign(self.back_r )*self.back_r - np.sign(self.back_l )*self.back_l
            if (self.back_r == -1 or self.back_l ==-1) or np.abs(diff_r_l) > self.threshoold:
                self.angle_rot = 0.2
                if (self.back_r != -1 or self.back_l !=-1):
                    self.angle_rot = -1*diff_r_l
                self.threshoold = 0.003

            else:
                self.angle_rot = 0
                self.threshoold += 0.002
                if self.threshoold > 0.02:
                    self.threshoold = 0.02

                    # todo go_back = 0

    
    def center_callback(self, msg):
        self.center_obj = msg.range
        
        if self.center_obj > 0 and self.center_obj < 0.1:
            self.angle_rot = 0.0
            self.speed = 0.0
            if self.go_back == 1:
                self.go_back = 2




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
        cmd_vel.linear.x  = self.speed 
        cmd_vel.angular.z = 1.0 * (self.angle_rot)
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
