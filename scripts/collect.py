#!/usr/bin/env python

import os
import sys
import rospy
from px100 import PX100
import aws_utilities as util


KVS_STREAM = "px100Stream"
S3_BUCKET = "adsnghw-misc" #"custom-labels-console-eu-central-1-8a96d04acd"
S3_PATH = "" #"datasets/PX100/"
PROJECT_ARN = "arn:aws:rekognition:eu-central-1:517502204741:project/PX100/1621861684686"
SIM_MODEL = "version/PX100.2021-05-24T15.08.29/1621861709475"
REAL_MODEL = "version/PX100.2021-05-28T12.46.07/1622198767106"
CONFIDENCE_THRESHOLD = 80


def main():
  # Sim option will use move_it to drive arm in Gazebo
  # Otherwise script will execute motion on physical arm
  _sim = False
  if len(sys.argv) > 1:
    if sys.argv[1] == "--sim":
      _sim = True

  # Remote option will source image from KVS clip
  # Otherwise script will snap image from local topic
  _kvs = False
  if len(sys.argv) > 2:
    if sys.argv[2] == "--kvs":
      _kvs = True

  # Select model based on real or simulated option
  model_arn = PROJECT_ARN + (SIM_MODEL if _sim else REAL_MODEL)
  
  try:
    rospy.loginfo("Checking state of Rekognition model...")
    print("[  INFO  ] Checking state of Rekognition model...")
    status = util.model_status(project_arn)

    print('[  INFO  ] Current model state: %s' % status)
    if status != 'RUNNING':
      print('[  ERROR ] Rekognition model needs to be in RUNNING state')
      return

    if _kvs:
      print("Press Enter to poll KVS livestream")
      raw_input()

      print('[  INFO  ] Polling KVS for livestream data...')
      video = util.retrieve_clip(KVS_STREAM)
      if video == None:
        print('[  ERROR ] Make sure ROS application is live and broadcasting to the correct KVS stream')
        return
  
      print('[  INFO  ] Retrieved clip from KVS: %s' % video)

      print("Press Enter to extract frame from KVS clip")
      raw_input()
      
      image = util.extract_frame(video)
      if image == None:
        print('[  ERROR ] Trouble extracting frame from clip')
        return
      
      os.remove(video)
      print('[  INFO  ] Extracted frame from clip: %s' % image)
    
    else:
      print("Press Enter to snap image from ROS topic")
      raw_input()
      
      image = util.snap_image()
      if image == None:
        print('[  ERROR ] Trouble snapping image from ROS topic')
        return
      
      print('[  INFO  ] Snapped image from local camera stream: %s' % image)
      
    

    print("Press Enter to upload extracted frame to S3")
    raw_input()
    
    if not util.upload_image(image, S3_BUCKET, S3_PATH):
      print('[  ERROR ] S3 upload failed')
      return

    os.remove(image)
    print('[  INFO  ] Uploaded file to AWS: s3://%s' % (S3_BUCKET + "/" + S3_PATH + image))
   
   
   
    print("Press Enter to discover labels with Rekognition")
    raw_input()
    
    labels = util.find_coins(model_arn, S3_BUCKET, S3_PATH + image, CONFIDENCE_THRESHOLD)
    print('[  INFO  ] Found %d labels in image' % len(labels))
    
    # util.print_labels(labels)
    # util.display_labels(S3_BUCKET, S3_PATH + image, labels)
    util.delete_image(S3_BUCKET, S3_PATH + image)

    coins = {}
    for l in labels:
      name = l['Name']
      x, y = util.get_coin_position(l['Geometry']['BoundingBox'])
      print(name)
      print('\tX: ' + str(x))
      print('\tY: ' + str(y))
      coins[name] = [x, y]
  
    
    print("Press Enter to instruct robot to pick a coin")
    raw_input()
    
    robot = PX100(simulated = _sim)

    # TODO: Decide which coin to pick up first
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

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

