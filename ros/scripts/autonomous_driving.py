# File: /Omega-Code/ros/scripts/autonomous_driving.py

"""
Autonomous Driving Node

This script handles the autonomous driving of the robot using a pre-trained machine learning model.
It subscribes to the camera image topic, processes the images, and controls the robot based on model predictions.

Functions:
- image_callback: Processes incoming camera images and makes predictions.
- control_robot: Sends commands to control the robot based on predictions.
- main: Initializes the ROS node and starts the image subscriber.

Dependencies:
- ROS: rospy, geometry_msgs, sensor_msgs, cv_bridge
- TensorFlow: tf.keras

Usage:
- Run this script to start the autonomous driving node.
"""

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf

model = tf.keras.models.load_model('/path/to/robot_navigation_model.h5')
bridge = CvBridge()

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # Preprocess image
    img = cv2.resize(cv_image, (64, 64))
    img = img / 255.0
    img = img.reshape(1, 64, 64, 3)

    # Predict with model
    prediction = model.predict(img)
    control_robot(prediction)

def control_robot(prediction):
    cmd = Twist()
    if prediction[0] > 0.5:
        cmd.linear.x = 0.5  # Move forward
    else:
        cmd.angular.z = 0.5  # Turn
    pub.publish(cmd)

def main():
    rospy.init_node('autonomous_driving')
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
