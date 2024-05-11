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

from Robotics_Project.camera_functions import plot_img_with_line2, select_type
from Robotics_Project.graph import Node_graph, Graph

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

        self.graph = Graph()

        self.subscription = self.create_subscription(
            Image,  # Usa Image o CompressedImage a seconda del tipo di messaggio
            '/thymio/camera',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.VERTICAL_THRESHOLD = np.pi/180 * 10

        self.next_node_neig = [0,0,0,0]





    def listener_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #plot_img_with_line2(cv_image)
        value = select_type(cv_image)
        
        # TODO: quando ricevi la value il thymio punta sempre a NORD quindi devi aggiustare la lista che arriva in formato N S E O 
        # nel formato aggiornato quindi se timio punta a Sud swappi N e S e O e E ecc
        new_value = value

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

    
    def center_callback(self, msg):
        self.center_obj = msg.range


# #########################################
# #########################################
# #########################################
        

            

    def start(self):
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
        cmd_vel.linear.x  = 0.2 * 0
        cmd_vel.angular.z = 1.0 * 0
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
    node.stop()


if __name__ == '__main__':
    main()
