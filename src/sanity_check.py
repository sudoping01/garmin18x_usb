#!/usr/bin/env python

import rospy
from math import pi, cos, sin, asin, atan2, sqrt
from garmin18x_usb.msg import BaseToRover
from sensor_msgs.msg import NavSatFix

class GpsCorrectionTest:

	def __init__(self):
		self.sub_topic = rospy.get_param("gps_correction_test/sub_topic", "fix")
		self.pub_topic = rospy.get_param("gps_correction_test/pub_topic", "gps_correction_test/rover_data")
		self.base_lat = rospy.get_param("gps_correction_base/base_lat", 30.385178399728332)
		self.base_lon = rospy.get_param("gps_correction_base/base_long", -97.72850900888446)
		self.base_alt = rospy.get_param("gps_correction_base/base_alt", 243.0)
		self.pub_rate = rospy.get_param("gps_correction_rover/pub_rate", 1.0)

		rospy.Subscriber(self.sub_topic, NavSatFix, self.update_data)
		self._pub_fix = rospy.Publisher(self.pub_topic, BaseToRover, queue_size=1)

		self.rlat1 = self.base_lat * pi / 180
		self.rlon1 = self.base_lon * pi / 180

		self.rover = BaseToRover()

		while not rospy.is_shutdown():
			pass

	def update_data(self, fix_in):
		if fix_in.status.status == 1:
			rlat2 = fix_in.latitude * pi / 180
			rlon2 = fix_in.longitude * pi / 180
			dlat =  rlat2 - self.rlat1
			dlon =  rlon2 - self.rlon1
			a = sin(dlat / 2)**2 + cos(self.rlat1) * cos(rlat2) * sin(dlon/2)**2
			c = 2 * asin(sqrt(a))
			self.rover.distance = c * 6371
			self.rover.bearing = atan2( sin(rlon2 - self.rlon1) * cos(rlat2), cos(self.rlat1) * sin(rlat2) - sin(self.rlat1) * cos(rlat2) * cos(rlon2 - self.rlon1))
			self.rover.rel_alt = fix_in.altitude - self.base_alt
			# Unused gps data fields - fix, epe, eph, epv, tow, east, north, up, msl_height, leap_secs, wn_days

			# getPvt is called again and not used because apparently the data is in a different format when called an even number of times.  The next call will provide data I can currently understand.
			print(self.rover)
			self._pub_fix.publish(self.rover)
		else:
			print("Rover is not reporting a fix")

if __name__=='__main__':
	rospy.init_node('gps_correction_test')

	try:
		_GpsCorrectionTest = GpsCorrectionTest()
	except rospy.ROSInterruptException: 
		pass