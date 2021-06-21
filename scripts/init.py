#!/usr/bin/env python

import sys
import rospy
from px100 import PX100


def main():
  # Sim option will use move_it to drive arm in Gazebo
  # Otherwise script will attempt to move physical arm
  _sim = True if "--sim" in sys.argv else False
    
  try:
    robot = PX100(simulated = _sim)
    
    robot.home()
    robot.sleep()
    robot.close_gripper()
    rospy.loginfo("Robot initialization successful")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

