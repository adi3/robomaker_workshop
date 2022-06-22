#!/usr/bin/env python

import os
import sys
import glob
import rospy
from px100 import PX100
import utilities as util


SIM_MODEL_ARN     = "arn:aws:rekognition:eu-central-1:517502204741:project/PX100-Sim/version/PX100-Sim.2022-04-25T09.51.03/1650873063273"
REAL_MODEL_ARN    = "arn:aws:rekognition:us-east-2:007281172979:project/ai-robotics-reinvent-2021/version/ai-robotics-reinvent-2021.2021-12-01T11.57.19/1638377839210"
IMAGE_NAME        = "src/robomaker_workshop/images/image_cap.png"
ACCESS_PROFILE    = "robomaker_workshop"

CONFIDENCE_THRESHOLD = 85

def main():
  # Sim option will use move_it to drive arm in Gazebo
  # Otherwise script will attempt to move physical arm
  _sim = False if "--real" in sys.argv else True
      
  # If accessing Rekognition model from an internal account,
  # then no separate role-based profile is needed
  _internal = False if "--external" in sys.argv else True
  
  # Use to break program execution midway
  _breakpoint = True if "--break" in sys.argv else False
  
  [os.remove(img) for img in glob.glob("*.png")]
  
  try:
    robot = PX100(simulated = _sim)
    
    if not _sim and not REAL_MODEL_ARN:
      rospy.logerr('Model ARN undefined for real hardware')
      return
      
    access_profile = "" if _internal else ACCESS_PROFILE
    model_arn = SIM_MODEL_ARN if _sim else REAL_MODEL_ARN
    model_name = model_arn.split('/')[-2]
    project_name = model_arn.split('/')[1]
    
    ########################################################################################
    #------------------------------------ Begin STEP 1 ------------------------------------#
    ########################################################################################
    rospy.loginfo("Checking state of Rekognition model...")
    status = util.model_status(project_name, model_name, access_profile)

    rospy.loginfo('Current model state: %s' % status)
    if status != 'RUNNING':
      rospy.logerr('Rekognition model needs to be in RUNNING state')
      return
    ########################################################################################
    #------------------------------------- End STEP 1 -------------------------------------#
    ########################################################################################
   
    
    ########################################################################################
    #------------------------------------ Begin STEP 2 ------------------------------------#
    ########################################################################################
    util.snap_image()
    rospy.loginfo('Snapped image: %s' % IMAGE_NAME)
    
    labels = util.find_coins(IMAGE_NAME, model_arn, CONFIDENCE_THRESHOLD, access_profile)
    rospy.loginfo('Found %d labels in image' % len(labels))
    
    util.print_labels(labels)
    if _sim:
      util.display_labels(IMAGE_NAME, labels)
    ########################################################################################
    #------------------------------------- End STEP 2 -------------------------------------#
    ########################################################################################

    if _breakpoint:
      return

    ########################################################################################
    #------------------------------------ Begin STEP 3 ------------------------------------#
    ########################################################################################
    rospy.loginfo('Transforming pixels to physical coordinates...')
    coins = {}
    
    for l in labels:
      name = l['Name']
      x, y = util.get_coin_position(l['Geometry']['BoundingBox'], _sim)
      rospy.loginfo(name)
      rospy.loginfo('\tX: ' + str(x) + ' m')
      rospy.loginfo('\tY: ' + str(y) + ' m')
      coins[name] = [x, y]
    ########################################################################################
    #------------------------------------- End STEP 3 -------------------------------------#
    ########################################################################################
  
     
    ########################################################################################
    #------------------------------------ Begin STEP 4 ------------------------------------#
    ########################################################################################
    for name, position in coins.items():
      robot.home()
      robot.open_gripper()

      x = position[0]
      y = position[1]
      
      rospy.loginfo("Picking up %s..." % name)
      success = robot.pick(x, y)
      if success:
        robot.deposit()

    rospy.loginfo("No more coins. Going to sleep...")
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

