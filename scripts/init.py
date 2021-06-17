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
      
  rospy.init_node("init", anonymous=True)
  robot = PX100(simulated = _sim)
    
  try:
    robot.home()
    if robot.sleep():
      rospy.loginfo("Initialized robot to sleep position")
    else:
      rospy.loginfo("Failed to initialize robot correctly")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

