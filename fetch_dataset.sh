#!/usr/bin/env bash

set -ef -o pipefail

aws s3 cp s3://adsnghw-robotics/px100-dataset/real_coins.zip /tmp/px100-dataset.zip --profile robomaker_workshop
unzip /tmp/px100-dataset.zip -d /tmp/px100-dataset

BUCKET=$(aws s3 ls | awk '{print $3}' | grep "custom-labels-console")
aws s3 cp /tmp/px100-dataset s3://$BUCKET/assets/px100-dataset --recursive

NUM=$(aws s3 ls s3://$BUCKET/assets/px100-dataset/ | wc -l)
echo "Uploaded $NUM images to $BUCKET bucket on S3"
