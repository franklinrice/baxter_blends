#!/usr/bin/env python
import sys, time, cv2
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
import subprocess
from sensor_msgs.msg import Image
from lab4_cam.srv import ImageSrv, ImageSrvResponse
from cv_bridge import CvBridge, CvBridgeError

class 