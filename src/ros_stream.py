#! /usr/bin/env python

import sys, os, math
import rospy
from sensor_msgs.msg import NavSatFix

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/pygarmin")
import garmin


class GarminToRos:

	def __init__(self):

		# Ros Parameters
		self.pub_topic = rospy.get_param("garmin_gps/pub_topic", "/fix")
		self.frame_id = rospy.get_param("garmin_gps/frame_id", "gps")
		self.pub_rate = rospy.get_param("garmin_gps/pub_rate", 1.0)
		dev_path = rospy.get_param("garmin_gps/dev_path", "/dev/sensors/garmin_gps")

		# Create a 'physical layer' connection using serial port
		phys = garmin.SerialLink(dev_path)

		# Create a Garmin object using this connection
		gps = garmin.Garmin(phys)
		
		# Turn on Position Velocity Time
		gps.pvtOn()

		# Publishers and subscribers
		self._pub_fix = rospy.Publisher(self.pub_topic, NavSatFix, queue_size=1)

		self.fix = NavSatFix()
		rate = rospy.Rate(self.pub_rate)

		while not rospy.is_shutdown():
			data = gps.getPvt()
			lat = data.rlat * 180 / math.pi
			lon = data.rlon * 180 / math.pi

			if abs(int(lat * 1e10) - int(self.fix.latitude * 1e10)) > 0 and abs(int(lon * 1e10) - int(self.fix.latitude * 1e10)) > 0 and abs(int(data.alt * 1e10) - int(self.fix.altitude * 1e10)) > 0:
				self.fix.status.status = 1
			else:
				self.fix.status.status = 0

			self.fix.header.stamp = rospy.get_rostime()
			self.fix.header.frame_id = self.frame_id
			self.fix.status.service = 1
			self.fix.latitude = lat
			self.fix.longitude = lon
			self.fix.altitude = data.alt
			self.fix.position_covariance = [0] * 9
			self.fix.position_covariance_type = 0
			# Unused gps data fields - fix, epe, eph, epv, tow, east, north, up, msl_height, leap_secs, wn_days

			self._pub_fix.publish(self.fix)

			# getPvt is called again and not used because apparently the data is in a different format when called an even number of times.  The next call will provide data I can currently understand.
			gps.getPvt()
			rate.sleep()

if __name__=='__main__':
	rospy.init_node('garmin_gps')

	try:
		_GarminToRos = GarminToRos()

	except rospy.ROSInterruptException: 
		pass