import rospy
from geometry_msgs.msg import Point, Twist
from math import pi
import numpy as np

class followTarget:

	def __init__(self):
		rospy.init_node('followTarget', anonymous = True)
		self.targetDistance = 0.3
		
		# Kp for angle (from dry runs)
		self.Kpa = 1
		# Kp for distance (from dry runs)
		self.Kpd = 100

		# subscribing to objectPoint and publishing /cmd_vel 
		self.sub_point = rospy.Subscriber("point", Point, self.object_point_callback)
		self.pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

	def object_point_callback(self, point):
		
		# defining errors
		angle_error = point.x
		dist_error = point.y - self.targetDistance
		
		# defining Twist()
		twistMsg = Twist()
		twistMsg.angular.z = angle_error * self.Kpa

		# 0.543 radians = 61.1 degrees
		if abs(angle_error) > 0.543:
			angle_error = 0
		if abs(twistMsg.angular.z) > 0.3:
			twistMsg.angular.z = 0.3 * np.sign(twistMsg.angular.z)

		# if angle error is less than half the fov in x direction		
		if angle_error < 31.1 * pi/180:
			twistMsg.linear.x = dist_error * self.Kpd
			if abs(twistMsg.linear.x) > 10:
				twistMsg.linear.x = 0.03 * np.sign(twistMsg.linear.x)	
			else:
				twistMsg.linear.x = 0
				twistMsg.angular.z = 0

		# publishing velocity messages
		self.pub_velocity.publish(twistMsg)

if __name__ == '__main__':
	followTarget = followTarget()
	rospy.spin()
