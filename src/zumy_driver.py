#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist, Quaternion, TransformStamped
import exp_quat_func as eqf
#from kalman_zumy.srv import *
#import NuSrv

baxter_listener = None
tf_listener = None
zumy_pub = None
zumy_tag = 'ar_marker_3'
timeOutDuration = 4
ar_tags = {}
speed = 0.001
offset = 0.10
tolerance = 0.025


def loadWaypoints(load_location, drop_location):
	waypoints = []
	waypoints.append( ("prep", load_location, (0, offset, 0) , rotNone() ) )
	waypoints.append( ("load", load_location, (0, offset, 0) , rot_CCW() ) )
	waypoints.append( ("drop", drop_location, (-1.5*offset, 0,0) , rotNone() ) )
	waypoints.append( ("load", load_location, (0, offset, 0) , rot_CCW() ) )
	# waypoints.append( ("prep", load_location, (0, 1.5*offset, 0) , rotNone() ) )
	return waypoints

def doCommand(load_location, drop_location):
	waypoints = loadWaypoints(load_location, drop_location)

	print "Waypoints loaded: "
	print waypoints

	
	currentTargetIndex = 0
	rate = rospy.Rate(10)
	keepDriving = True
	while not rospy.is_shutdown() and keepDriving: 
		try:
			currentWaypoint = waypoints[currentTargetIndex]
			currentPosDiff, currentRotDiff = goToLocation(getWaypointName(currentWaypoint))

			if abs(currentPosDiff[0]) < tolerance and abs(currentPosDiff[1]) < tolerance:
				print "Either something broke, or we're there!"
				currentTargetIndex += 1
				keepDriving = currentTargetIndex < len(waypoints)
					
		except:
			print(sys.exc_info())
			continue
			# continue
	message = Twist()
	message.linear = Vector3(0,0,0)
	message.angular = Vector3(0,0,0)
	zumy_pub.publish(message)

	print "We should be stopping."



def lookup(tag1, tag2):
	print "Looking up transform between", tag1, "and", tag2
	# while not rospy.is_shutdown():
	# 	try: 
			# tf_listener.waitForTransform(tag1, tag2, rospy.Time(0), rospy.Duration(timeOutDuration))
			# (trans, rot) = tf_listener.lookupTransform(tag1, tag2, rospy.Time(0))
			# return (trans, rot)
		# except:
		# 	print(sys.exc_info())
		# 	continue
	tf_listener.waitForTransform(tag1, tag2, rospy.Time(0), rospy.Duration(timeOutDuration))
	(trans, rot) = tf_listener.lookupTransform(tag1, tag2, rospy.Time(0))
	return (trans, rot)


def goToLocation (location):
	trans, rot = lookup(zumy_tag, location)
	message = getMsgFromTransform (trans, rot)
	zumy_pub.publish(message)
	return (trans, rot)


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
	computeScale = lambda x,y,z, target: (float(target)/(x*x+y*y+z*z))**(0.5)
	a = computeScale (v[0], v[1], v[2], speed)
	print "Scale", a
	linear.x = v[0]*a
	linear.y = v[1]*a
	linear.z = v[2]*a

	angular = Vector3()
	angular.x = omega[0]*theta/2
	angular.y = omega[1]*theta/2
	angular.z = omega[2]*theta/2

	message = Twist()
	message.linear = linear
	message.angular = angular

	return message

 
  
if __name__=='__main__':
	rospy.init_node('zumy_node_driver')
	ar_tags = {}
	zumy_name = sys.argv[1]
	tf_listener = tf.TransformListener()
	zumy_pub = rospy.Publisher('%s/cmd_vel' % zumy_name, Twist, queue_size=2)

	doCommand(load_location='ar_marker_4', drop_location='ar_marker_5')
	# rospy.Subscriber('/zumy_command_handle', ZumyCmd, doCommand)
	
	rospy.spin()