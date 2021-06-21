#!/usr/bin/env python

import sys
import rospy
import numpy as np
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from interbotix_xs_modules.arm import InterbotixManipulatorXS


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
  def __init__(self, simulated=True):
    super(PX100, self).__init__()

    self._SIM = simulated
    self._ROBOT_NAME = "px100"

    self._ARM_NAME = "interbotix_arm"
    self._GRIPPER_NAME = "interbotix_gripper"
    self._DESC_PARAM = "/robot_description"
    
    # Offset of robot from center of table
    self._X_OFFSET_SIM = 0.15
    self._X_OFFSET_REAL = 0.1
    # Height above table at which coin can be picked up
    self._Z_OFFSET_SIM = 0.01
    self._Z_OFFSET_REAL = 0.03
    
    self._SLEEP_POSITIONS = [0.0, -1.88, 1.5, 0.8]   # [waist, shoulder, elbow, wrist_angle]
    self._DEPOSIT_POSITIONS = [2.5, 0.0, 0.8, -0.8]   # [waist, shoulder, elbow, wrist_angle]
    self._GRIPPER_OPEN = 0.035
    self._GRIPPER_CLOSE = 0.022
    
    self._arm = None
    self._gripper = None

    if self._SIM:
      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node(self._ROBOT_NAME, anonymous=True)
      self._arm = moveit_commander.MoveGroupCommander(robot_description=self._DESC_PARAM, name=self._ARM_NAME)
      self._gripper = moveit_commander.MoveGroupCommander(robot_description=self._DESC_PARAM, name=self._GRIPPER_NAME)
    else:
      robot = InterbotixManipulatorXS(self._ROBOT_NAME, group_name="arm", gripper_name="gripper", gripper_pressure=0.8)
      self._arm = robot.arm
      self._gripper = robot.gripper



  def deposit(self):
    rospy.loginfo('Moving robot to DEPOSIT position...')
    if self._SIM:
      self._arm.go(self._DEPOSIT_POSITIONS, wait=True)
      self._arm.stop()
      return self.open_gripper()
    else:
      self._arm.set_joint_positions(self._DEPOSIT_POSITIONS)
      self._gripper.open()
 


  def move_gripper(self, position):
    goal = [position, -1*position]
    self._gripper.go(goal, wait=True)
    self._gripper.stop()

    current = self._gripper.get_current_joint_values()
    return check(goal, current, 0.01)
    


  def open_gripper(self):
    rospy.loginfo('Opening robot gripper...')
    if self._SIM:
      return self.move_gripper(self._GRIPPER_OPEN)
    else:
      self._gripper.open()
    
  

  def close_gripper(self):
    rospy.loginfo('Closing robot gripper...')
    if self._SIM:
      return self.move_gripper(self._GRIPPER_CLOSE)
    else:
      self._gripper.close()
    
  

  def home(self):
    rospy.loginfo('Moving robot to HOME position...')
    if self._SIM:
      home_positions = [0] * len(self._arm.get_active_joints())
      self._arm.go(home_positions)
      return check(home_positions, self._arm.get_current_joint_values(), 0.01)
    else:
      self._arm.go_to_home_pose()



  def sleep(self):
    rospy.loginfo('Moving robot to SLEEP position...')
    if self._SIM:
      self._arm.go(self._SLEEP_POSITIONS)
      self._arm.stop()
      return check(self._SLEEP_POSITIONS, self._arm.get_current_joint_values(), 0.01)
    else:
      self._arm.go_to_sleep_pose()
    

  def pick(self, x, y):
    x_offset = self._X_OFFSET_SIM if self._SIM else self._X_OFFSET_REAL
    z_offset = self._Z_OFFSET_SIM if self._SIM else self._Z_OFFSET_REAL
    goal = [x + x_offset, y, z_offset]
    
    success = self.go_to(goal)
    if success:
        self.close_gripper()
        self.home()
    return success


  def go_to(self, goal):
    x = goal[0]
    y = goal[1]
    z = goal[2]
    rospy.loginfo('Moving robot gripper to (%.2f, %.2f, %.2f)...' % (x, y, z))

    # Separate Y and XZ movements
    waist_angle = np.arctan2(y, x)

    # First move to theta in polar coordinates with FK
    # Only waist movement required to accomplish this
    if self._SIM:
      joints = self._arm.get_current_joint_values()
      joints[0] = waist_angle
      self._arm.go(joints, wait=True)
      self._arm.stop()
      # print(self._arm.get_current_pose().pose)
    else:
      self._arm.set_single_joint_position("waist", waist_angle)
    

    # Determine 1D distance between EE and coin
    if np.around(np.cos(waist_angle), decimals=2) != 0:
      x_dash = x / np.cos(waist_angle)
    else:
      x_dash = y

    # NB: Interbotix arms do not work out-of-the-box with MoveIt.
    # Hence, we need to do IK yourself for the simulated scenario.
    # Moving the physical arm is much simpler.
    if self._SIM:
      # Only interested in finding XZ plan from Moveit
      # IK for 3D space fails on PX100
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.position.x = x_dash
      pose_goal.position.z = z # height for picking up coin
      # print(pose_goal)
      self._arm.set_pose_target(pose_goal)

      # Determine final joint states for XZ from trajectory
      xz_plan = self._arm.plan()

      if not xz_plan.joint_trajectory.points:
        rospy.logwarn('No path found to given location.')
        return False

      xz_joints = np.asarray(xz_plan.joint_trajectory.points[-1].positions)

      # Now move to r in polar coordinates
      # Cannot do 3D motion in one go as EE collides with object
      xz_joints[0] = waist_angle
      self._arm.go(xz_joints, wait=True)
      self._arm.stop()

      # Confirm current position is within tolerable range of desired goal
      current = self._arm.get_current_pose().pose.position
      # print(current)
      return check([x, y], [current.x, current.y], 0.01)
    else:
     theta_list, result = self._arm.set_ee_pose_components(x=x, y=y, z=z)
     return result
