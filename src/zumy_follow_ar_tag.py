#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist, Quaternion, TransformStamped
import exp_quat_func as eqf
from kalman_zumy.srv import *
#import NuSrv

baxter_listener = None
tf_listener = None
tf_broadcaster = None
zumy_pub = None
zumy_tag = 'ar_marker_3'
timeOutDuration = 4
ar_tags = {}
prevZ = -1
speed = 0.2

offset = 0.10
tolerance = 0.01


def loadWaypoints(load_location, drop_location):
	waypoints = []
	waypoints.append( ("prep", load_location, (0, offset, 0) , rotNone() ) )
	waypoints.append( ("load", load_location, (0, offset, 0) , rot_CCW() ) )
	waypoints.append( ("drop", drop_location, (-1.5*offset, 0,0) , rotNone() ) )
	waypoints.append( ("load", load_location, (0, offset, 0) , rot_CCW() ) )
	### Uncomment this line to send Zumy back to the prep waypoint
	# waypoints.append( ("prep", load_location, (0, offset, 0) , rotNone() ) )
	return waypoints

def broadcastWaypoints(waypoints):
	for waypoint in waypoints:

		tf_broadcaster.sendTransform(waypoint[2], 
									waypoint[3], 
									rospy.Time.now(), 
									getWaypointName(waypoint),  
									waypoint[1] )
		print "Sending Transform", getWaypointName(waypoint), "whose parent is ", waypoint[1]

def doCommand(load_location, drop_location):
	waypoints = loadWaypoints(load_location, drop_location)

	print "Waypoints loaded: "
	print waypoints

	
	currentTargetIndex = 0
	rate = rospy.Rate(10)
	# while currentTargetIndex < len(waypoints) : #we need to change this to loop until it hits all waypoints
	while not rospy.is_shutdown():
		broadcastWaypoints(waypoints)
		rate.sleep()

		### Additional stopping constraints if Zumy is an aggressive driver
		# currentWaypoint = waypoints[currentTargetIndex]
		# currentPosDiff, currentRotDiff = lookup(load_location, getWaypointName(currentWaypoint))

		# while abs(currentPosDiff[0]) > tolerance and abs(currentPosDiff[1]) > tolerance and abs(currentRotDiff[0]) > tolerance and abs(currentRotDiff[1]) > tolerance:
		# 	# goToLocation(getWaypointName(currentWaypoint))
		# 	currentPosDiff, currentRotDiff = lookup(zumy_tag, getWaypointName(currentWaypoint))
		# 	print "Positional difference:", currentPosDiff
		# 	print "Rotational difference:", currentRotDiff


def rot_CCW ():
	return (0, 0, 0.707, 0.707)

def rotNone ():
	return (0, 0, 0, 1)

def getWaypointName(waypoint):
	return waypoint[0] + "_" + waypoint[1]

def getMsgFromTransform (trans, rot):
	omega, theta = eqf.quaternion_to_exp(np.array(rot))
	omega = np.reshape(omega, (3,1))
	v = eqf.find_v(omega, theta, trans)
	twist = np.vstack((v, omega))

	linear = Vector3()
	linear.x = v[0]*speed
	linear.y = v[1]*speed
	linear.z = v[2]*speed

	angular = Vector3()
	angular.x = omega[0]*theta/2
	angular.y = omega[1]*theta/2
	angular.z = omega[2]*theta/2

	message = Twist()
	message.linear = linear
	message.angular = angular

	return message

 
  
if __name__=='__main__':
	rospy.init_node('zumy_node')
	tf_broadcaster = tf.TransformBroadcaster()
	doCommand(load_location='ar_marker_4', drop_location='ar_marker_5')
	
	rospy.spin()