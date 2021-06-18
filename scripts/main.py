#!/usr/bin/env python

import os
import sys
import glob
import rospy
from px100 import PX100
import utilities as util

SIM_MODEL_ARN     = "arn:aws:rekognition:eu-central-1:517502204741:project/PX100/version/PX100.2021-06-07T01.15.17/1623021317812"
REAL_MODEL_ARN    = ""
ACCESS_PROFILE    = "robomaker_workshop"
IMAGE_NAME        = "/home/ubuntu/environment/aws_ws/src/robomaker_workshop/image_cap.png"

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
    
    # Add code here
    
    ########################################################################################
    #------------------------------------- End STEP 1 -------------------------------------#
    ########################################################################################

   
    ########################################################################################
    #------------------------------------ Begin STEP 2 ------------------------------------#
    ########################################################################################

    # Add code here
    
    ########################################################################################
    #------------------------------------- End STEP 2 -------------------------------------#
    ########################################################################################


    ########################################################################################
    #------------------------------------ Begin STEP 3 ------------------------------------#
    ########################################################################################
    
    # Add code here
    
    ########################################################################################
    #------------------------------------- End STEP 3 -------------------------------------#
    ########################################################################################
  
     
    ########################################################################################
    #------------------------------------ Begin STEP 4 ------------------------------------#
    ########################################################################################
    
    # Add code here

    ########################################################################################
    #------------------------------------- End STEP 4 -------------------------------------#
    ########################################################################################

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

