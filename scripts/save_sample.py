#!/usr/bin/env python

import os
import uuid
import utilities as util

S3_BUCKET = "custom-labels-console-eu-central-1-8a96d04acd"
S3_PATH = "datasets/sim_coins/"


def main():
  try:

    print('[  INFO  ] Capturing image from camera stream...')
    image = util.snap_image()
    if image == None:
      print('[  ERROR ] Trouble snapping image from ROS topic')
      return
    
    new_name = str(uuid.uuid4()).split('-')[0] + ".png"
    os.rename(image, new_name)
    
    print('[  INFO  ] Saved image as: %s' % new_name)
    if not util.upload_image(new_name, S3_BUCKET, S3_PATH):
      print('[  ERROR  ] S3 upload failed')
      return

    os.remove(new_name)
    print('[  INFO  ] Uploaded file to AWS: s3://%s' % (S3_BUCKET + "/" + S3_PATH + new_name))

  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

