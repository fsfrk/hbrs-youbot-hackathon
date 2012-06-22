

import roslib; roslib.load_manifest('raw_move_spiral') 
import rospy
import re
import math
import time
import numpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def talker():
    pub = rospy.Publisher('current_pose', PoseWithCovarianceStamped)
    rospy.init_node('current_pose')
    while not rospy.is_shutdown():
        goal = PoseWithCovarianceStamped()
	goal.pose.pose.position.x = 0
	goal.pose.pose.position.y = 0
        rospy.loginfo(goal)
        pub.publish(goal)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
	
			
		
		
		
		
		
			
		
	
						
