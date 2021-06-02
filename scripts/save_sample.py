#!/usr/bin/env python

import os
import aws_utilities as util


KVS_STREAM = "px100Stream"
S3_BUCKET = "custom-labels-console-eu-central-1-8a96d04acd"
S3_PATH = "datasets/real_coins/"


def main():
  try:

    print('[  INFO  ] Polling KVS for livestream data...')
    video = util.retrieve_clip(KVS_STREAM)
    if video == None:
      print('[  ERROR  ] Make sure ROS application is live and broadcasting to the correct KVS stream')
      return

    print('[  INFO  ] Retrieved clip from KVS: %s' % video)

    image = util.extract_frame(video)
    if image == None:
      print('[  ERROR  ] Trouble extracting frame from clip')
      return
      
    os.remove(video)
    print('[  INFO  ] Extracted frame from clip: %s' % image)
    if not util.upload_image(image, S3_BUCKET, S3_PATH):
      print('[  ERROR  ] S3 upload failed')
      return

    os.remove(image)
    print('[  INFO  ] Uploaded file to AWS: s3://%s' % (S3_BUCKET + "/" + S3_PATH + image))

  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()

