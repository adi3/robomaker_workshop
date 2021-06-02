#!/usr/bin/env python

import sys
import math
import rospy
import numpy as np
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list



# Check whether actual position is close to goal within the given tolerance
def check(goal, actual, tolerance):
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return check(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return check(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True



class PX100(object):
  def __init__(self):
    super(PX100, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('px100', anonymous=True)

    self.ARM_NAME = "interbotix_arm"
    self.GRIPPER_NAME = "interbotix_gripper"
    self.X_OFFSET = -0.15   # Offset of robot from center of table
    self.SLEEP_POSITIONS = [0.0, -1.88, 1.5, 0.8]   # [waist, shoulder, elbow, wrist_angle]
    self.DEPOSIT_POSITIONS = [3.14, 0.0, 0.8, -0.8]   # [waist, shoulder, elbow, wrist_angle]

    self.GRIPPER_OPEN = 0.035
    self.GRIPPER_CLOSE = 0.022
    
    self.arm = moveit_commander.MoveGroupCommander(robot_description="px100/robot_description", ns="px100", name=self.ARM_NAME)
    self.gripper = moveit_commander.MoveGroupCommander(robot_description="px100/robot_description", ns="px100", name=self.GRIPPER_NAME)
    

  def deposit(self):
    rospy.loginfo('Moving robot to DEPOSIT position...')
    self.arm.go(self.DEPOSIT_POSITIONS, wait=True)
    self.arm.stop()
    return self.open_gripper()
 

  def move_gripper(self, position):
    goal = [position, -1*position]
    self.gripper.go(goal, wait=True)
    self.gripper.stop()

    current = self.gripper.get_current_joint_values()
    return check(goal, current, 0.01)
    

  def open_gripper(self):
    rospy.loginfo('Opening robot gripper...')
    return self.move_gripper(self.GRIPPER_OPEN)
    
  
  def close_gripper(self):
    rospy.loginfo('Closing robot gripper...')
    return self.move_gripper(self.GRIPPER_CLOSE)
    
  

  def home(self):
    rospy.loginfo('Moving robot to HOME position...')
    home_positions = [0] * len(self.arm.get_active_joints())
    self.arm.go(home_positions)

    arm_joints = self.arm.get_current_joint_values()
    return check(home_positions, arm_joints, 0.01)


  def sleep(self):
    rospy.loginfo('Moving robot to SLEEP position...')
    self.arm.go(self.SLEEP_POSITIONS)
    self.arm.stop()

    arm_joints = self.arm.get_current_joint_values()
    return check(self.SLEEP_POSITIONS, arm_joints, 0.01)



  def go_to(self, goal):
    x = goal[0] - self.X_OFFSET
    y = goal[1]
    z = goal[2]
    rospy.loginfo('Moving robot gripper to (%.2f, %.2f, %.2f)...' % (goal[0], y, z))

    # Separate Y and XZ movements
    waist_angle = math.atan2(y, x)

    # First move to theta in polar coordinates with FK
    # Only waist movement required to accomplish this
    current_joints = self.arm.get_current_joint_values()
    current_joints[0] = waist_angle
    self.arm.go(current_joints, wait=True)
    self.arm.stop()
    # print(self.arm.get_current_pose().pose)

    # Determine 1D distance between EE and coin
    if np.around(math.cos(waist_angle), decimals=2) != 0:
      x_dash = x / math.cos(waist_angle)
    else:
      x_dash = y

    # Only interested in finding XZ plan from Moveit
    # IK for 3D space fails on PX100
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x_dash
    pose_goal.position.z = z # height for picking up coin
    self.arm.set_pose_target(pose_goal)
    print(pose_goal)

    # Determine final joint states for XZ from trajectory
    xz_plan = self.arm.plan()

    if not xz_plan.joint_trajectory.points:
      rospy.logwarn('No path found to given location.')
      return False

    xz_joints = np.asarray(xz_plan.joint_trajectory.points[-1].positions)

    # Now move to r in polar coordinates
    # Cannot do 3D motion in one go as EE collides with object
    xz_joints[0] = waist_angle
    self.arm.go(xz_joints, wait=True)
    self.arm.stop()

    # Confirm current position is within tolerable range of desired goal
    current = self.arm.get_current_pose().pose.position
    print(current)
    return check([x, y], [current.x, current.y], 0.01)