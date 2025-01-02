#!/usr/bin/env python3

# import rospy
# import moveit_commander
# from moveit_commander.robot_trajectory import RobotTrajectory
# from geometry_msgs.msg import Pose
# from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
# from moveit_commander.robot_trajectory import RobotTrajectory

# def move_arm_to_xyz(x, y, z):
#     # Initialize the moveit_commander and ROS node
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('move_arm_to_xyz', anonymous=True)

#     # Initialize the robot and planning scene
#     robot = RobotCommander()
#     scene = PlanningSceneInterface()

#     # Set up the move group for the robot arm
#     move_group = MoveGroupCommander("arm")  # Assuming "arm" is the name of the MoveIt! group for the robot arm

#     # Set the reference frame (optional)
#     reference_frame = "base_link"  # The frame from which the pose is referenced (can be "base_link", "world", etc.)
#     move_group.set_pose_reference_frame(reference_frame)

#     # Define the target pose (x, y, z)
#     target_pose = Pose()
#     target_pose.orientation.w = 1.0  # Assuming no rotation, you can modify this as needed.
#     target_pose.position.x = x
#     target_pose.position.y = y
#     target_pose.position.z = z

#     # Set the target pose for the arm to move to
#     move_group.set_pose_target(target_pose)

#     # Plan and execute the trajectory to the goal position
#     plan = move_group.go(wait=True)  # Execute the motion
#     rospy.loginfo("Move completed!")

#     # Stop the robot after the motion
#     move_group.stop()

#     # Clear the targets for safety
#     move_group.clear_pose_targets()

#     # Shut down moveit_commander
#     moveit_commander.roscpp_shutdown()

# if __name__ == "__main__":
#     try:
#         # Example target positions (in meters)
#         x = 0.5
#         y = 0.0
#         z = 0.3

#         # Move the arm to the target position
#         move_arm_to_xyz(x, y, z)
#     except rospy.ROSInterruptException:
#         pass

import rospy
from geometry_msgs.msg import Pose
import stretch_body.robot

def move_to_pos(x, y, z):
    rospy.init_node('stretch_movexyz', anonymous=True)

    robot = stretch_body.robot.Robot()

    robot.startup()

    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    robot.base.translate_by(x)
    robot.base.rotate_by(y)
    robot.arm.move_to(z)
    robot.push_command()

    robot.stop()

if __name__ == '__main__':
    try:
        move_to_pos(1, 0, 1)

    except rospy.ROSInterruptException:
        pass