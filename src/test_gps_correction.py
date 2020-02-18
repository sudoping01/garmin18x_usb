#!/usr/bin/env python

import rospy, math
from sensor_msgs.msg import NavSatFix

class DistHeading:

	def __init__(self):
		self.correction_sub_topic = rospy.get_param("gps_correction_rover/correction_sub_topic", "fix/correction")	
		self.pub_topic = rospy.get_param("gps_correction_rover", "fix")
		self.pub_rate = rospy.get_param("gps_correction_rover/pub_rate", 1.0)
		dev_path = rospy.get_param("gps_correction_rover/dev_path", "/dev/sensors/garmin_gps")

		# Create a 'physical layer' connection using serial port
		phys = garmin.SerialLink(dev_path)

		# Create a Garmin object using this connection
		self.gps = garmin.Garmin(phys)
		
		# Turn on Position Velocity Time
		self.gps.pvtOn()

		rospy.Subscriber(self.correction_sub_topic, GpsCorrection, self.correct_fix)
		self._pub_fix = rospy.Publisher(self.pub_topic, NavSatFix, queue_size=1)
		self.fix = NavSatFix()

		rate = rospy.Rate(self.pub_rate)
		
		while not rospy.is_shutdown():
			pass

	def correct_fix(self, correction):
		data = self.gps.getPvt()
		lat = data.rlat * 180 / math.pi
		lon = data.rlon * 180 / math.pi

		print("Tow: " + str(data.tow) + " Ctow:" + str(correction.tow))
		print("CLat: " + str(lat + correction.lat))
		print("CLon: " + str(lon + correction.lon))

		if abs(int(lat * 1e10) - int(self.fix.latitude * 1e10)) > 0 and abs(int(lon * 1e10) - int(self.fix.latitude * 1e10)) > 0 and abs(int(data.alt * 1e10) - int(self.fix.altitude * 1e10)) > 0:
			self.fix.status.status = 1
		else:
			self.fix.status.status = 0

		self.fix.header.stamp = rospy.get_rostime()
		self.fix.header.frame_id = "gps"
		self.fix.status.service = 1
		self.fix.latitude = lat + correction.lat
		self.fix.longitude = lon + correction.lon
		self.fix.altitude = data.alt + correction.alt
		self.fix.position_covariance = [0] * 9
		self.fix.position_covariance_type = 0
		# Unused gps data fields - fix, epe, eph, epv, tow, east, north, up, msl_height, leap_secs, wn_days

		# getPvt is called again and not used because apparently the data is in a different format when called an even number of times.  The next call will provide data I can currently understand.
		self.gps.getPvt()
		self._pub_fix.publish(self.fix)

if __name__=='__main__':
	rospy.init_node('gps_correction_checker')

	try:
		_DistHeading = DistHeading()
	except rospy.ROSInterruptException: 
		pass