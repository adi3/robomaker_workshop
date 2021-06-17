#!/usr/bin/env python

import rospy
from px100 import PX100


def main():
  rospy.init_node("init", anonymous=True)
  robot = PX100()
    
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

