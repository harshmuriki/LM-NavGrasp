#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatus
import time

# Define the goals as a list of poses
goals = [
    {'x':4.407, 'y': -0.098, 'z': 0.0,
      'qx': 0.0, 'qy': 0.0, 'qz': 0.712, 'qw': 0.702}, # Table 1 (forwarding to the desk) 

    {'x':3.407, 'y': -0.098, 'z': 0.0,
      'qx': 0.0, 'qy': 0.0, 'qz': 1.000, 'qw': 0.024}, # Table 1 go back (forwarding to the desk) 

    {'x':4.45, 'y': 1.1, 'z': 0.0,
      'qx': 0.0, 'qy': 0.0, 'qz': 0.710, 'qw': 0.704}, # passway
    
    {'x':7.443, 'y': 1.269, 'z': 0.0,
      'qx': 0.0, 'qy': 0.0, 'qz': -0.999, 'qw': 0.039}, # Table 2 
    {'x':4.390, 'y': 1.190, 'z': 0.0,
      'qx': 0.0, 'qy': 0.0, 'qz': 0.710, 'qw': 0.704}, # passway
    {'x':4.780, 'y': 1.989, 'z': 0.0,
      'qx': 0.0, 'qy': 0.0, 'qz': 0.713, 'qw': 0.701}, # Table 3
    {'x':4.007, 'y': -0.098, 'z': 0.0,
      'qx': 0.0, 'qy': 0.0, 'qz': 0.712, 'qw': 0.702}, # help waypoints
    
    {'x': 0.0, 'y': 0.0, 'z': 0.0, 
     'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': -1.0} # start point
]

def send_goal_to_move_base(goal_pose):
    # Initialize ROS node
    rospy.init_node('send_sequential_goals')

    # Create a publisher to send goals to the move_base action server
    goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    # Give some time for the publisher to connect
    rospy.sleep(1)

    # Create the goal message
    goal_msg = MoveBaseActionGoal()

    # Fill in the header fields
    goal_msg.header.seq = 0
    goal_msg.header.stamp = rospy.Time.now()  # Current timestamp
    goal_msg.header.frame_id = 'map'

    # Set the goal_id to a unique value (can be empty if not required)
    goal_msg.goal_id.stamp = rospy.Time(0)
    goal_msg.goal_id.id = ''

    # Set the target pose
    target_pose = PoseStamped()
    target_pose.header.seq = 0
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = 'map'

    target_pose.pose.position.x = goal_pose['x']
    target_pose.pose.position.y = goal_pose['y']
    target_pose.pose.position.z = goal_pose['z']

    target_pose.pose.orientation.x = goal_pose['qx']
    target_pose.pose.orientation.y = goal_pose['qy']
    target_pose.pose.orientation.z = goal_pose['qz']
    target_pose.pose.orientation.w = goal_pose['qw']

    goal_msg.goal.target_pose = target_pose

    # Publish the goal
    rospy.loginfo("Sending goal: %s", goal_msg)
    goal_pub.publish(goal_msg)

    # Send the message and then wait fpr /move_base/result

    # Wait for the result by subscribing to the /move_base/result topic
    result_received = False
    while not result_received:
        result_msg = rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
        if result_msg.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached: %s", result_msg)
            result_received = True
        elif result_msg.status.status == GoalStatus.PREEMPTED:
            rospy.loginfo("Goal was preempted.")
            result_received = True
        else:
            rospy.loginfo("Goal is still being processed...")

    # After it goes to the table, wait for manipulation to send /manipulation_status -> Done/Failed
    # then go to the next table

    # Next goal

def execute_goals():
    for goal in goals:
        send_goal_to_move_base(goal)
        rospy.sleep(1)  # Optional: wait for a short time before sending the next goal

if __name__ == '__main__':
    # scan the map:
    # 1. roslaunch stretch_navigation mapping.launch
    # 2. rosrun map_server map_saver -f $HOME/stretch_user/maps/{DATE}

    # launch the navigation:
    # 1. roslaunch stretch_navigation navigation.launch map_yaml:=$HOME/stretch_user/maps/{DATE}.yaml
    # 2. rosrun lm_navgrasp move.py



    try:
        execute_goals()
    except rospy.ROSInterruptException:
        pass
