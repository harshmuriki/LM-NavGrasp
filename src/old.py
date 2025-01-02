#!/usr/bin/env python3

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction

class StowCommand(hm.HelloNode):
    def __init__(self):
        # Initialize the HelloNode class
        super(StowCommand, self).__init__()

        # Initialize class variables
        self.received_action = ""
        self.current_joint_positions = {}

        # Create an action client for the FollowJointTrajectoryAction
        self.trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        # Wait for the action server to be available
        rospy.loginfo("Waiting for trajectory action server...")
        self.trajectory_client.wait_for_server()
        print("done")

        # Subscribe to the /action and /joint_states topics
        rospy.Subscriber('/action', Float32MultiArray, self.action_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    # Callback to store current joint positions
    def joint_state_callback(self, msg):
        # Update the joint positions dictionary with the latest values
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]

    # Function to move arm to a specific pose (absolute joint positions)
    def move_to_pose(self, lift_position, wrist_position, gripper_position):
        # Create a JointTrajectoryPoint with the target joint positions
        move_point = JointTrajectoryPoint()
        move_point.time_from_start = rospy.Duration(1.0)  # Duration for the movement
        if lift_position < 0:
            lift_position = 0
        if lift_position > 1.09:
            lift_position = 1.09

        if gripper_position > 0.2:
            gripper_position = 0.2

        if gripper_position < -0.3:
            gripper_position = -0.3
        move_point.positions = [wrist_position, lift_position, gripper_position]

        # Create a FollowJointTrajectoryGoal to send to the trajectory server
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_gripper_finger_left']
        trajectory_goal.trajectory.points = [move_point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        # Send the trajectory goal to the trajectory server
        rospy.loginfo('Sending absolute move goal = {}'.format(trajectory_goal))
        self.trajectory_client.send_goal(trajectory_goal)
        self.trajectory_client.wait_for_result()

        rospy.loginfo("Moved arm to new positions: lift: {0}, wrist: {1}, gripper: {2}".format(
            lift_position, wrist_position, gripper_position))

    # Callback for /action topic
    def action_callback(self, msg):
        self.received_action = msg.data  # The action is expected to be a list or array
        
        rospy.loginfo("Received action: %s", self.received_action)
        
        # Map the action to a pose (assuming received_action is a list of [x, y, z, roll, pitch, yaw, close])
        x, y, z, roll, pitch, yaw, close = self.received_action
        
        # Move the arm to an absolute pose (e.g., set specific joint positions)
        # Here, x maps to lift_position, z maps to wrist_position, and close maps to gripper_position

        self.move_to_pose(x, z, close)

# Main function
if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('stow_command_node')
        
        # Create an instance of StowCommand
        node = StowCommand()    

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
