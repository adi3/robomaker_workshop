#!/usr/bin/env python

import os
import io
import cv2
import glob
import boto3
import uuid
import rospy
import base64
import numpy as np
from std_srvs.srv import Empty
from PIL import Image, ImageDraw, ImageFont

boto3.compat.filter_python_deprecation_warnings()

LABEL_COLOR = '#cc0000'
SNAP_SRV = '/image_saver/save'

# For Linux
FONT_TYPE = '/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf'
FONT_SIZE = 30

# For macOS
# FONT_TYPE = '/Library/Fonts/Arial.ttf'
# FONT_SIZE = 50

X_RES_SCALING = 0.435
Y_RES_SCALING = 0.765
DECIMALS = 2


# Fetch Rekognition model status
def model_status(project_arn, model_name, access_profile):
	rekognition = boto3.client('rekognition')
	if access_profile:
		rekognition = boto3.Session(profile_name=access_profile).client('rekognition')
		
	response = rekognition.describe_project_versions(
				ProjectArn=project_arn,
				VersionNames=[model_name])
				
	return response["ProjectVersionDescriptions"][0]["Status"]
	
	
	
# Take snapshot from local camera stream
def snap_image():
	rospy.wait_for_service(SNAP_SRV)
	try:
		rospy.ServiceProxy(SNAP_SRV, Empty)()
		rospy.sleep(1)
		[os.remove(ini) for ini in glob.glob("*.ini")]
		return sorted(glob.glob("*.png"))[-1]
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: %s"%e)
		return None



# Call Rekognition inference on image
def find_coins(image_name, model_arn, min_confidence, access_profile):
	rekognition = boto3.client('rekognition')
	if access_profile:
		rekognition = boto3.Session(profile_name=access_profile).client('rekognition')
	
	with open(image_name, "rb") as img:
		base64_image=base64.b64encode(img.read()).decode('utf-8')
		base_64_binary = base64.b64decode(base64_image)
        
	response = rekognition.detect_custom_labels(Image={'Bytes': base_64_binary},
        		ProjectVersionArn=model_arn, MinConfidence=min_confidence)
	return response['CustomLabels']
	


# Print labels in human-readable form to console
def print_labels(labels):
	for l in labels:
		rospy.loginfo('Label: ' + str(l['Name']))
		rospy.loginfo('\tConfidence: ' + str(l['Confidence']))
		
		bbox = l['Geometry']['BoundingBox']
		rospy.loginfo('\tLeft: '   + str(bbox['Left']))
		rospy.loginfo('\tTop: '	   + str(bbox['Top']))
		rospy.loginfo('\tWidth: '  + str(bbox['Width']))
		rospy.loginfo('\tHeight: ' + str(bbox['Height']))
		print('')



# Show labels superimposed on an image
def display_labels(image_name, labels):
	image = Image.open(open(image_name, 'rb'))
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
	y = 2 * Y_RES_SCALING * (0.5 - (bbox['Left'] + 0.5*bbox['Width']))
	return np.around(x, decimals=DECIMALS), np.around(y, decimals=DECIMALS)
	
	
	
# Upload image to S3
def upload_image(image_name, s3_bucket, s3_path):
	s3 = boto3.client('s3')
	try:
		s3.upload_file(image_name, s3_bucket, s3_path + image_name)
		return True
	except:
		return False