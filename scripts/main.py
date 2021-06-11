#!/usr/bin/env python

import os
import sys
import glob
import rospy
from px100 import PX100
import utilities as util

ACCESS_PROFILE = "robomaker_workshop"
ARN_BASE = "arn:aws:rekognition:eu-central-1:517502204741:project/PX100/"
PROJECT_ID = "1621861684686"
SIM_MODEL_NAME = "PX100.2021-06-07T01.15.17"
REAL_MODEL_NAME = "PX100.2021-05-28T12.46.07"
SIM_MODEL_ID = "1623021317812"
REAL_MODEL_ID = "1622198767106"
CONFIDENCE_THRESHOLD = 85


def main():
  # Sim option will use move_it to drive arm in Gazebo
  # Otherwise script will attempt to move physical arm
  _sim = False
  if len(sys.argv) > 1:
    if sys.argv[1] == "--sim":
      _sim = True
      
  # If accessing Rekognition model from an internal account,
  # then no separate role-based profile is needed
  _internal = False
  if len(sys.argv) > 2:
    if sys.argv[2] == "--internal":
      _internal = True
      
  access_profile = "" if _internal else ACCESS_PROFILE

  # Select model based on real or simulated option
  model_name = SIM_MODEL_NAME if _sim else REAL_MODEL_NAME
  model_id = SIM_MODEL_ID if _sim else REAL_MODEL_ID
  model_arn = ARN_BASE + 'version/' + model_name + '/' + model_id
  
  [os.remove(img) for img in glob.glob("*.png")]
  
  try:
    rospy.init_node("main", anonymous=True)
    
    ########################################################################################
    #------------------------------------ Begin STEP 1 ------------------------------------#
    ########################################################################################
    rospy.loginfo("Checking state of Rekognition model...")
    status = util.model_status(ARN_BASE + PROJECT_ID, model_name, access_profile)

    rospy.loginfo('Current model state: %s' % status)
    if status != 'RUNNING':
      rospy.logerr('Rekognition model needs to be in RUNNING state')
      return
    ########################################################################################
    #------------------------------------- End STEP 1 -------------------------------------#
    ########################################################################################

    return
    ########################################################################################
    #------------------------------------ Begin STEP 2 ------------------------------------#
    ########################################################################################
    rospy.logwarn('Press Enter to snap image from ROS topic')
    # raw_input()
    
    image = util.snap_image()
    if image == None:
      rospy.logerr('Trouble snapping image from ROS topic')
      return
    
    rospy.loginfo('Snapped image from local camera stream: %s' % image)
    ########################################################################################
    #------------------------------------- End STEP 2 -------------------------------------#
    ########################################################################################
   
   
    ########################################################################################
    #------------------------------------ Begin STEP 3 ------------------------------------#
    ########################################################################################
    rospy.logwarn('Press Enter to discover labels with Rekognition')
    # raw_input()
    
    labels = util.find_coins(image, model_arn, CONFIDENCE_THRESHOLD, access_profile)
    rospy.loginfo('Found %d labels in image' % len(labels))
    
    util.print_labels(labels)
    # util.display_labels(image, labels)
    ########################################################################################
    #------------------------------------- End STEP 3 -------------------------------------#
    ########################################################################################


    ########################################################################################
    #------------------------------------ Begin STEP 4 ------------------------------------#
    ########################################################################################
    rospy.logwarn("Press Enter to transform coin positions into physical coordinates")
    # raw_input()
    
    rospy.loginfo('Transforming pixels to physical coordinates...')
    coins = {}
    
    for l in labels:
      name = l['Name']
      x, y = util.get_coin_position(l['Geometry']['BoundingBox'])
      rospy.loginfo(name)
      rospy.loginfo('\tX: ' + str(x) + ' m')
      rospy.loginfo('\tY: ' + str(y) + ' m')
      coins[name] = [x, y]
    ########################################################################################
    #------------------------------------- End STEP 4 -------------------------------------#
    ########################################################################################
  
     
    ########################################################################################
    #------------------------------------ Begin STEP 5 ------------------------------------#
    ########################################################################################
    rospy.logwarn("Press Enter to instruct robot to pick a coin")
    # raw_input()
    
    robot = PX100(simulated = _sim)

    # CHALLENGE 1: Play around with confidence threshold
    # CHALLENGE 2: Decide which coin to pick up first
    for name, position in coins.items():
      robot.home()
      robot.open_gripper()

      x = position[0]
      y = position[1]
      
      rospy.loginfo("Picking up %s..." % name)
      success = robot.go_to([x, y, 0.01])
      
      if success:
        robot.close_gripper()
        robot.home()
        robot.deposit()
        
      # print("Press Enter to pick up another coin")
      # raw_input()

    rospy.loginfo("No more coins. Going to sleep...")
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

