#!/usr/bin/env bash

set -ef -o pipefail

echo -ne "- Downloading dataset from remote S3 bucket..."
aws s3 cp s3://adsnghw-robotics/px100-dataset/real_coins.zip /tmp/px100-dataset.zip --quiet --profile robomaker_workshop
echo -e "success"

echo -ne "- Unpacking images from archive..."
unzip -q /tmp/px100-dataset.zip -d /tmp/px100-dataset
echo -e "success"

echo -ne "- Fetching name of S3 bucket associated with Rekognition Custom Labels..."
BUCKET=$(aws s3 ls | awk '{print $3}' | grep "custom-labels-console")
echo -e "success"

echo -ne "- Uploading images dataset to S3..."
aws s3 cp /tmp/px100-dataset s3://$BUCKET/assets/px100-dataset --recursive --quiet
echo -e "success"

echo -ne "- Removing temporary local resources..."
rm -rf /tmp/px100-dataset /tmp/px100-dataset.zip
echo -e "success"

NUM=$(aws s3 ls s3://$BUCKET/assets/px100-dataset/ | wc -l)
echo -e "\nUploaded $NUM images to $BUCKET on S3"
echo -e "\tPATH: s3://$BUCKET/assets/px100-dataset/"
