import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from math import pi
import numpy as np

class targetRange:

	def __init__(self):
		self.laserScan = LaserScan()
		self.fov_h = 62.2
		rospy.init_node('target_range', anonymous = True)
		
		# subscribing to image location and Lidar messages and publishing values to chaseObject
		self.sub_image_location = rospy.Subscriber("imageLocation", Point, self.image_location_callback)
		self.sub_lidar_scan = rospy.Subscriber('/scan', LaserScan, self.lidar_scan_callback)
		self.pub_polar = rospy.Publisher("point", Point, queue_size = 1)

	def image_location_callback(self, point):
		# used the camerav2_320X240_5fps.launch
		x = point.x * self.fov_h / 320
		x = -x + self.fov_h / 2
		z = point.z * self.fov_h / 320

		# converting all values to radians
		x = x * pi/180
		r = z * pi/180
		
		# defining initial and final bounds
		initialBound = round((x - r - self.laserScan.angle_min)/self.laserScan.angle_increment)
		finalBound = round((x + r - self.laserScan.angle_min)/self.laserScan.angle_increment)
		
		# reading LIDAR values
		ranges = np.array(self.laserScan.ranges)
		if initialBound < 0 and finalBound > 0:
			object_ranges = np.concatenate((ranges[359-initialBound:], ranges[0:finalBound]))
		else:
			object_ranges = ranges[initialBound:finalBound]
		
		# creating a mask to eliminate values which are not required
		mask = np.bitwise_and(object_ranges > self.laserScan.range_min, object_ranges < self.laserScan.range_max)
		if object_ranges[mask].size > 0:
			min_dist = object_ranges[mask].min()
		else:
			min_dist = 0.3

		msg = Point()
		msg.x = x
		msg.y = min_dist

		#publishing the values
		self.pub_polar.publish(msg)

	def lidar_scan_callback(self, laserScan):
		self.laserScan = laserScan

if __name__ == '__main__':
	targetRange = targetRange()
	rospy.spin()
	