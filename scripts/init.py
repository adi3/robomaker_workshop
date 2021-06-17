#!/usr/bin/env python

import sys
import rospy
from px100 import PX100


def main():
  # Sim option will use move_it to drive arm in Gazebo
  # Otherwise script will attempt to move physical arm
  _sim = False
  if len(sys.argv) > 1:
    if sys.argv[1] == "--sim":
      _sim = True
    
  try:
    robot = PX100(simulated = _sim)
    
    robot.home()
    robot.sleep()
    rospy.loginfo("Robot initialization successful")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

