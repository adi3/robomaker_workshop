#!/usr/bin/env python

import os
import sys
import rospy
from px100 import PX100
import aws_utilities as util


KVS_STREAM = "px100Stream"
S3_BUCKET = "adsnghw-misc" #"custom-labels-console-eu-central-1-8a96d04acd"
S3_PATH = "" #"datasets/PX100/"
ARN_BASE = "arn:aws:rekognition:eu-central-1:517502204741:project/PX100/"
PROJECT_ID = "1621861684686"
SIM_MODEL_NAME = "PX100.2021-05-24T15.08.29"
REAL_MODEL_NAME = "PX100.2021-05-28T12.46.07"
SIM_MODEL_ID = "1621861709475"
REAL_MODEL_ID = "1622198767106"
CONFIDENCE_THRESHOLD = 60


def main():
  # Sim option will use move_it to drive arm in Gazebo
  # Otherwise script will execute motion on physical arm
  _sim = False
  if len(sys.argv) > 1:
    if sys.argv[1] == "--sim":
      _sim = True
      
  # Select model based on real or simulated option
  model_name = SIM_MODEL_NAME if _sim else REAL_MODEL_NAME
  model_id = SIM_MODEL_ID if _sim else REAL_MODEL_ID
  model_arn = ARN_BASE + 'version/' + model_name + '/' + model_id
  
  
  try:
    ########################################################################################
    #------------------------------------ Begin STEP 0 ------------------------------------#
    ########################################################################################
    rospy.loginfo("Checking state of Rekognition model...")
    print("[  INFO  ] Checking state of Rekognition model...")
    status = util.model_status(ARN_BASE + PROJECT_ID, model_name)

    print('[  INFO  ] Current model state: %s' % status)
    if status != 'RUNNING':
      print('[  ERROR ] Rekognition model needs to be in RUNNING state')
      return
    ########################################################################################
    #------------------------------------- End STEP 0 -------------------------------------#
    ########################################################################################


    ########################################################################################
    #------------------------------------ Begin STEP 1 ------------------------------------#
    ########################################################################################
    print("Press Enter to snap image from ROS topic")
    raw_input()
    
    image = util.snap_image()
    if image == None:
      print('[  ERROR ] Trouble snapping image from ROS topic')
      return
    
    print('[  INFO  ] Snapped image from local camera stream: %s' % image)
    ########################################################################################
    #------------------------------------- End STEP 1 -------------------------------------#
    ########################################################################################
   
   
    ########################################################################################
    #------------------------------------ Begin STEP 2 ------------------------------------#
    ########################################################################################
    print("Press Enter to discover labels with Rekognition")
    raw_input()
    
    labels = util.find_coins(image, model_arn, CONFIDENCE_THRESHOLD)
    print('[  INFO  ] Found %d labels in image' % len(labels))
    
    util.print_labels(labels)
    # util.display_labels(image, labels)
    ########################################################################################
    #------------------------------------- End STEP 2 -------------------------------------#
    ########################################################################################


    ########################################################################################
    #------------------------------------ Begin STEP 3 ------------------------------------#
    ########################################################################################
    print("Press Enter to transform coin positions into real world coordinates")
    raw_input()
    
    coins = {}
    for l in labels:
      name = l['Name']
      x, y = util.get_coin_position(l['Geometry']['BoundingBox'])
      print(name)
      print('\tX: ' + str(x))
      print('\tY: ' + str(y))
      coins[name] = [x, y]
    ########################################################################################
    #------------------------------------- End STEP 3 -------------------------------------#
    ########################################################################################
    
     
    ########################################################################################
    #------------------------------------ Begin STEP 4 ------------------------------------#
    ########################################################################################
    print("Press Enter to instruct robot to pick a coin")
    raw_input()
    
    robot = PX100(simulated = _sim)

    # CHALLENGE 1: Play around with confidence treshold
    # CHALLENGE 2: Decide which coin to pick up first
    for name, position in coins.items():
      robot.home()
      robot.open_gripper()

      x = position[0]
      y = position[1]
      
      print("[  INFO  ] Picking up %s..." % name)
      
      if robot.go_to([x, y, 0.01]):
        robot.close_gripper()
        robot.home()
        robot.deposit()
        
      print("Press Enter to pick up another coin")
      raw_input()

    print("[  INFO  ] No more coins. Going to sleep...")
    robot.sleep()
    ########################################################################################
    #------------------------------------- End STEP 4 -------------------------------------#
    ########################################################################################

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

