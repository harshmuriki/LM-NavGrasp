#!/usr/bin/env python3

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float32MultiArray
from rerail_stretchit_manipulation.srv import RerailManip, RerailManipResponse

# Initialize a global variable for the image or action data
received_action = ""

# Convert Euler angles (roll, pitch, yaw) to quaternion
def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tft.quaternion_from_euler(roll, pitch, yaw)
    rospy.loginfo("Converted Euler angles to quaternion: %s", quaternion)
    return quaternion


def calculate_x(x):
    # scaled_up = (x+1) * 128
    # print("scaledup", scaled_up)
    min_x = -10
    max_x = 10

    new_x = (max_x - min_x) * (x/255)
    print("new_x", new_x)
    return new_x

# Function to send pose to manipulation service
def send_quaternion_to_service(x, y, z, roll, pitch, yaw):
    # Convert Euler angles to quaternion
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    
    # Create a Pose message
    target_pose = Pose()
    target_pose.position.x = calculate_x(x)
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]
    
    rospy.loginfo("Target Pose: %s", target_pose)
    
    # Call the manipulation service
    rospy.wait_for_service('/rerail_stretchit_manipulation/manipulate')
    try:
        manipulate_service = rospy.ServiceProxy('/rerail_stretchit_manipulation/manipulate', RerailManip)
        response = manipulate_service(target_pose)
        rospy.loginfo("Service call successful: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

# Callback for /action topic
def action_callback(msg):
    global received_action
    received_action = msg.data  # The action is expected to be a string
    
    rospy.loginfo("Received action: %s", received_action)
    
    # Map the action to a pose (you can modify this logic based on your application)
    x, y, z, roll, pitch, yaw, close = received_action
    
    # Send the calculated pose to the manipulation service
    send_quaternion_to_service(x, y, z, roll, pitch, yaw)

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('action_listener_node')  # Initialize the ROS node
        
        rospy.loginfo("Action listener node started, waiting for actions...")
        
        # Subscribe to the /action topic where actions are published
        rospy.Subscriber('/action', Float32MultiArray, action_callback)
        
        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
