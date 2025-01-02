#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def camera_publisher():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)

    # Create a publisher to send the camera images
    image_pub = rospy.Publisher('/camera/image_usb', Image, queue_size=10)

    # Create a CvBridge object to convert OpenCV images to ROS Image messages
    bridge = CvBridge()

    # OpenCV video capture object (0 is usually the default camera)
    cap = cv2.VideoCapture(6)

    if not cap.isOpened():
        rospy.logerr("Failed to open camera.")
        return

    # Set the camera resolution (optional, default is 640x480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Publish the camera feed in a loop
    rate = rospy.Rate(30)  # Publish at 30 Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to grab frame.")
            break

        # Convert OpenCV image to ROS Image message
        try:
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the image
            image_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))

        # Sleep to maintain the desired loop rate
        rate.sleep()

    # Release the camera when done
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
