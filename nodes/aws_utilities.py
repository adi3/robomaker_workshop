#!/usr/bin/env python

import os
import io
import cv2
import glob
import boto3
import uuid
import rospy
import numpy as np
from std_srvs.srv import Empty
from PIL import Image, ImageDraw, ImageFont


boto3.compat.filter_python_deprecation_warnings()

LABEL_COLOR = '#FF9900'
SNAP_SRV = '/px100/image_saver/save'

# For Linux
FONT_TYPE = '/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf'
FONT_SIZE = 30

# For macOS
# FONT_TYPE = '/Library/Fonts/Arial.ttf'
# FONT_SIZE = 50

X_RES_SCALING = 0.435
Y_RES_SCALING = 0.765
DECIMALS = 2


# Retrieve latest ~1s clip from KVS
def retrieve_clip(stream_name):
	kvs = boto3.client("kinesisvideo")
	endpoint = kvs.get_data_endpoint(
		StreamName=stream_name,
		APIName='GET_CLIP')['DataEndpoint']
	video_name = str(uuid.uuid4()).split('-')[0] + ".mp4"

	# For Linux
	start = "$(date -d \"5 seconds ago\" -u \"+%FT%T+0000\")"
	end = "$(date -u \"+%FT%T+0000\")"

	# For macOS
	# start = "$(date -v -5S -u \"+%FT%T+0000\")"
	# end = "$(date -u \"+%FT%T+0000\")"

	# Python API call fails but bash command works
	command = "aws kinesis-video-archived-media get-clip --endpoint-url " + endpoint + \
	" --stream-name px100Stream --clip-fragment-selector \"FragmentSelectorType=SERVER_TIMESTAMP,\
	TimestampRange={StartTimestamp=" + start + ",EndTimestamp=" + end + "}\" " + video_name + " >/dev/null"

	if os.system(command) == 0:
		return video_name
	else:
		return None



# Extract first frame of clip
def extract_frame(video_name):
	# Open video in openCV
	video = cv2.VideoCapture(video_name)
	# num_frames = video.get(cv2.CAP_PROP_FRAME_COUNT);
	# video.set(cv2.CAP_PROP_POS_FRAMES, num_frames);
	success,image = video.read()
	image_name = video_name.split(".")[0] + ".png"

	if success:
		# Save clip as image file
		cv2.imwrite(image_name, image)
		return image_name
	else:
		return None



# Take snapshot from camera stream
def snap_image():
	rospy.wait_for_service(SNAP_SRV)
	try:
		rospy.ServiceProxy(SNAP_SRV, Empty)()
		rospy.sleep(1)
		[os.remove(ini) for ini in glob.glob("*.ini")]
		return glob.glob("*.png")[0]
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
		return None
      
      

# Upload image to S3
def upload_image(image_name, s3_bucket, s3_path):
	s3 = boto3.client('s3')
	try:
		s3.upload_file(image_name, s3_bucket, s3_path + image_name)
		return True
	except:
		return False


# Delete image from S3
def delete_image(s3_bucket, image_key):
	s3 = boto3.client('s3')
	try:
		s3.delete_object(Bucket=s3_bucket, Key=image_key)
		return True
	except:
		return False



# Fetch Rekognition model status
def model_status(model_arn):
	rekognition = boto3.client('rekognition')
	response = rekognition.describe_project_versions(ProjectArn=model_arn)
	return response["ProjectVersionDescriptions"][0]["Status"]



# Call Rekognition inference on image
def find_coins(model_arn, s3_bucket, image_path, min_confidence):
	rekognition = boto3.client('rekognition')
	
	response = rekognition.detect_custom_labels(Image={'S3Object': {'Bucket': s3_bucket, 'Name': image_path}},
        		ProjectVersionArn=model_arn, MinConfidence=min_confidence)
	return response['CustomLabels']



# Print labels in human-readable form to console
def print_labels(labels):
	for l in labels:
		print('Label: ' + str(l['Name']))
		print('\tConfidence: ' + str(l['Confidence']))
		
		bbox = l['Geometry']['BoundingBox']
		print('\tLeft: '   + str(bbox['Left']))
		print('\tTop: '	   + str(bbox['Top']))
		print('\tWidth: '  + str(bbox['Width']))
		print('\tHeight: ' + str(bbox['Height']))



# Show labels superimposed on an image
def display_labels(s3_bucket, image_path, labels):
	# Load image from S3 bucket
	s3 = boto3.resource('s3')
	response = s3.Object(s3_bucket, image_path).get()
	image = Image.open(io.BytesIO(response['Body'].read()))
	
	img_width, img_height = image.size
	draw = ImageDraw.Draw(image)
	
	# Calculate and plot bounding boxes for each detected custom label
	for l in labels:
		bbox = l['Geometry']['BoundingBox']
		left = img_width * bbox['Left']
		top = img_height * bbox['Top']
		width = img_width * bbox['Width']
		height = img_height * bbox['Height']

		fnt = ImageFont.truetype(FONT_TYPE, FONT_SIZE)
		draw.text((left,top), l['Name'], fill=LABEL_COLOR, font=fnt)

		points = (
			(left, top),
			(left + width, top),
			(left + width, top + height),
			(left , top + height),
			(left, top)
		)
		draw.line(points, fill=LABEL_COLOR, width=5)

	image.show()


# Obtain physical position of coin from Rekognition bbox
def get_coin_position(bbox):
	"""
    Rekognition provides top-left corner and size of bounding box.
    We exploit the fact that ratios between points in both the image and the physical world remain the same
    1. Find midpoint of bbox by adding half of height or width
    2. Obtain fraction position of midpoint with respect to center of frame
    3. Multiply with a constant scaling factor to get physical coordinates
    """
	
	x = 2 * X_RES_SCALING * (0.5 - (bbox['Top'] + 0.5*bbox['Height']))
	y = 2 * Y_RES_SCALING* (0.5 - (bbox['Left'] + 0.5*bbox['Width']))
	return np.around(x, decimals=DECIMALS), np.around(y, decimals=DECIMALS)