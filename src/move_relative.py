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

    # Function to move arm relative to the current joint positions
    def move_arm_relative(self, lift_offset, wrist_offset, gripper_offset):
        # Get the current joint positions (assuming the joints have been initialized)
        current_lift = self.current_joint_positions.get('joint_lift', 0)
        current_wrist = self.current_joint_positions.get('wrist_extension', 0)
        current_gripper = self.current_joint_positions.get('joint_gripper_finger_left', 0)

        # Calculate new joint positions by adding offsets
        new_lift = (current_lift + lift_offset * 2)
        print("lift:", current_lift, new_lift)
        new_wrist = (current_wrist + wrist_offset * 2)
        print("wrist", current_wrist, new_wrist)
        new_gripper = current_gripper + gripper_offset

        if new_lift < 0:
            new_lift = 0
        if new_lift > 1.09:
            new_lift = 1.09

        if new_wrist > 1:
            new_wrist = 1
        if new_wrist < 0.0:
            new_wrist = 0.0

        if new_gripper < -0.3:
            new_gripper = -0.3
        if new_gripper > 0.20:
            new_gripper = 0.20

        # Create a JointTrajectoryPoint with the new joint positions
        move_point = JointTrajectoryPoint()
        move_point.time_from_start = rospy.Duration(1.0)  # Duration for the movement
        move_point.positions = [new_lift, new_wrist, new_gripper]

        # Create a FollowJointTrajectoryGoal to send to the trajectory server
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_gripper_finger_left']
        trajectory_goal.trajectory.points = [move_point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        # Send the trajectory goal to the trajectory server
        # rospy.loginfo('Sending relative move goal = {}'.format(trajectory_goal))
        self.trajectory_client.send_goal(trajectory_goal)
        self.trajectory_client.wait_for_result()

        rospy.loginfo("Moved arm to new positions: lift: {0}, wrist: {1}, gripper: {2}".format(
            new_lift, new_wrist, new_gripper))

    # Callback for /action topic
    def action_callback(self, msg):
        self.received_action = msg.data  # The action is expected to be a list or array
        
        rospy.loginfo("Received action: %s", self.received_action)
        
        # Map the action to a pose (assuming received_action is a list of [x, y, z, roll, pitch, yaw, close])
        x, y, z, roll, pitch, yaw, close = self.received_action
        
        # Move the arm relative to the current position with a fixed offset (you can modify this logic as needed)
        lift_offset = x  # Example relative movement for joint_lift
        wrist_offset = z  # Example relative movement for wrist_extension
        gripper_offset = close #close  # Example relative movement for joint_gripper_finger_left

        # Move the arm relative to its current position
        self.move_arm_relative(lift_offset, wrist_offset, gripper_offset)

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