#!/usr/bin/env python

import os
import sys
import rospy
from px100 import PX100
import utilities as util

ARN_BASE = "arn:aws:rekognition:eu-central-1:517502204741:project/PX100/"
PROJECT_ID = "1621861684686"
SIM_MODEL_NAME = "PX100.2021-06-07T01.15.17"
REAL_MODEL_NAME = "PX100.2021-05-28T12.46.07"
SIM_MODEL_ID = "1623021317812"
REAL_MODEL_ID = "1622198767106"
CONFIDENCE_THRESHOLD = 90

MODEL_ACCESS_PROFILE = ""#"rekognition_access"


def main():
  # Sim option will use move_it to drive arm in Gazebo
  # Otherwise script will attempt to move physical arm
  _sim = False
  if len(sys.argv) > 1:
    if sys.argv[1] == "--sim":
      _sim = True
      
  # Select model based on real or simulated option
  model_name = SIM_MODEL_NAME if _sim else REAL_MODEL_NAME
  model_id = SIM_MODEL_ID if _sim else REAL_MODEL_ID
  model_arn = ARN_BASE + 'version/' + model_name + '/' + model_id
  
  try:
    rospy.init_node("main", anonymous=True)
    
    ########################################################################################
    #------------------------------------ Begin STEP 1 ------------------------------------#
    ########################################################################################
    rospy.loginfo("Checking state of Rekognition model...")
    # print("[  INFO  ] Checking state of Rekognition model...")
    status = util.model_status(ARN_BASE + PROJECT_ID, model_name, MODEL_ACCESS_PROFILE)

    rospy.loginfo('Current model state: %s' % status)
    # print('[  INFO  ] Current model state: %s' % status)
    if status != 'RUNNING':
      rospy.logerr('Rekognition model needs to be in RUNNING state')
      # print('[  ERROR ] Rekognition model needs to be in RUNNING state')
      return
    ########################################################################################
    #------------------------------------- End STEP 1 -------------------------------------#
    ########################################################################################


    ########################################################################################
    #------------------------------------ Begin STEP 2 ------------------------------------#
    ########################################################################################
    rospy.logwarn('Press Enter to snap image from ROS topic')
    raw_input()
    
    image = util.snap_image()
    if image == None:
      rospy.logerr('Trouble snapping image from ROS topic')
      # print('[  ERROR ] Trouble snapping image from ROS topic')
      return
    
    rospy.loginfo('Snapped image from local camera stream: %s' % image)
    # print('[  INFO  ] Snapped image from local camera stream: %s' % image)
    ########################################################################################
    #------------------------------------- End STEP 2 -------------------------------------#
    ########################################################################################
   
   
    ########################################################################################
    #------------------------------------ Begin STEP 3 ------------------------------------#
    ########################################################################################
    rospy.logwarn('Press Enter to discover labels with Rekognition')
    raw_input()
    
    labels = util.find_coins(image, model_arn, CONFIDENCE_THRESHOLD, MODEL_ACCESS_PROFILE)
    rospy.loginfo('Found %d labels in image' % len(labels))
    # print('[  INFO  ] Found %d labels in image' % len(labels))
    
    util.print_labels(labels)
    # util.display_labels(image, labels)
    ########################################################################################
    #------------------------------------- End STEP 3 -------------------------------------#
    ########################################################################################


    ########################################################################################
    #------------------------------------ Begin STEP 4 ------------------------------------#
    ########################################################################################
    print("Press Enter to transform coin positions into physical coordinates")
    raw_input()
    
    rospy.loginfo('Transforming pixels to physical coordinates...' % len(labels))
    coins = {}
    
    for l in labels:
      name = l['Name']
      x, y = util.get_coin_position(l['Geometry']['BoundingBox'])
      rospy.loginfo(name)
      rospy.loginfo('\tX: ' + str(x))
      rospy.loginfo('\tY: ' + str(y))
      # print(name)
      # print('\tX: ' + str(x))
      # print('\tY: ' + str(y))
      coins[name] = [x, y]
    ########################################################################################
    #------------------------------------- End STEP 4 -------------------------------------#
    ########################################################################################
    
    return
     
    ########################################################################################
    #------------------------------------ Begin STEP 5 ------------------------------------#
    ########################################################################################
    print("Press Enter to instruct robot to pick a coin")
    raw_input()
    
    robot = PX100(simulated = _sim)

    # CHALLENGE 1: Play around with confidence threshold
    # CHALLENGE 2: Decide which coin to pick up first
    for name, position in coins.items():
      robot.home()
      robot.open_gripper()

      x = position[0]
      y = position[1]
      
      print("[  INFO  ] Picking up %s..." % name)
      success = robot.go_to([x, y, 0.01])
      
      if success:
        robot.close_gripper()
        robot.home()
        robot.deposit()
        
      print("Press Enter to pick up another coin")
      raw_input()

    print("[  INFO  ] No more coins. Going to sleep...")
    robot.sleep()
    ########################################################################################
    #------------------------------------- End STEP 5 -------------------------------------#
    ########################################################################################

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

