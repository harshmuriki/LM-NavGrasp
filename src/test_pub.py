#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from sensor_msgs.msg import Image


def image_callback(image_msg):
    try:
        # Print the received float data
        # rospy.loginfo(f"Received float data: {image_msg.data}")
        
        # Run SAM and CLIP (assuming you're processing the float data here)
        print("Running SAM and CLIP")

        # Load the depth data from a file (as an example)
        # This file should contain depth data in a 2D matrix (e.g., CSV format)
        depth_image = np.loadtxt("/home/hello-robot/catkin_ws/src/lm_navgrasp/src/array_10.txt", delimiter=',')
        rospy.loginfo(f"Depth image shape: {depth_image.shape}")

        # Create a Float32MultiArray message to send the depth data
        data_msg = Float32MultiArray()
        data_msg.data = depth_image.flatten().tolist()  # Flatten the array to send as a list of floats

        # Publish the depth image data to /detected_coordinates topic
        for _ in range(2):  # Publish multiple times if needed
            data_pub.publish(data_msg)
            rospy.sleep(2)  # Sleep for a bit to allow the message to be sent
            rospy.loginfo(f"Published depth image data: {data_msg.data[:5]}...")  # Print first 5 values for debugging

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

def listener():
    # Initialize the ROS node
    print("inside")
    rospy.init_node('image_processor', anonymous=True)

    # Create a publisher to send processed data (of type Float32MultiArray)
    global data_pub
    data_pub = rospy.Publisher('/detected_coordinates', Float32MultiArray, queue_size=10)

    rospy.Subscriber('/image_sam', Image, image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        print("outside")
        listener()  # Start the listener to process incoming messages
    except rospy.ROSInterruptException:
        pass
