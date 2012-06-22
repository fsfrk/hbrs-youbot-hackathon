

import roslib; roslib.load_manifest('raw_move_spiral') 
import rospy
import re
import math
import time
import numpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class MoveBase:
	
	def __init__(self):
		rospy.init_node('omni', anonymous=True)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)
		rospy.Subscriber('odom',Odometry,self.odom_callback)
		rospy.Subscriber('current_pose',PoseWithCovarianceStamped,self.pose_callback)
		rospy.Subscriber('goal_pose',PoseWithCovarianceStamped,self.goal_callback)
		self.x = 0
		self.y = 0
		self.z = 0
		self.track = PoseWithCovarianceStamped()
		self.robot_pose = numpy.matrix([[0],[0],[0]])
		self.goal_pose = numpy.matrix([[0],[0],[0]])
	
	
	def pose_callback(self,current_pose):
		x = current_pose.pose.pose.position.x
		y = current_pose.pose.pose.position.y
		self.robot_pose[0] = x
		self.robot_pose[1] = y
		
	def goal_callback(self,goal_pose):
		x =  goal_pose.pose.pose.position.x
		y =  goal_pose.pose.pose.position.y
		print x
		print y
		self.goal_pose[0] = x
		self.goal_pose[1] = y

	def req_x(self):
		x = abs(self.goal_pose[0] - self.robot_pose[0])
		#print x
		return x
	
	def req_y(self):
		y = (self.goal_pose[1] - self.robot_pose[1])
		#print y		
		return y

	def action(self,vel):
		cmd_vel = Twist()
		cmd_vel.linear.x = vel[0]
	        cmd_vel.linear.y = vel[1]
		cmd_vel.angular.z = 0
		self.cmd_vel_pub.publish(cmd_vel)
	
	def odom_callback(self,odom):
		self.track = odom
		odomx = self.track.pose.pose.position.x
		odomy = self.track.pose.pose.position.y
	def callx(self):
		return self.track.pose.pose.position.x
	def cally(self):
		return self.track.pose.pose.position.y		
	
	
if  __name__ == '__main__':
	
	mb = MoveBase()
	initial_odomx = mb.callx()
	#print "initial_odomx:", initial_odomx
	initial_odomy = mb.cally()
	#print "initial_odomy:", initial_odomy
	x = mb.req_x()
	#print x
	y = mb.req_y()
	startx = 0
	starty = 0
	vel = numpy.matrix([[0],[0],[0]])
	while (not rospy.is_shutdown()):
		current_odomx = mb.callx()
		current_odomy = mb.cally()
		debug = abs(current_odomx - initial_odomx)
		if (startx == 0):	
			while (debug < x) :
				vel = numpy.matrix([[0.1],[0],[0]])
				mb.action(vel)
				current_odomx = mb.callx()
				current_odomy = mb.cally()
				debug = abs(current_odomx - initial_odomx)
				print debug
			startx = 999
		if (starty == 0):
			while (abs(current_odomy - initial_odomy) < y) :
				vel = numpy.matrix([[0],[0.1],[0]])
				mb.action(vel)
				debug = abs(current_odomy - initial_odomy)
				#print debug		
			starty = 999
			
		vel = numpy.matrix([[0],[0],[0]])
		mb.action(vel)	
			
		
		
		
		
		
			
		
	
						
