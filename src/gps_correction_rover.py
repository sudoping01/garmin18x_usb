#!/usr/bin/env python

import sys, os, math
import rospy
from garmin18x_usb.msg import GpsCorrection
from sensor_msgs.msg import NavSatFix

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/pygarmin")
import garmin

class GpsRover:

	def __init__(self):
		self.correction_sub_topic = rospy.get_param("gps_correction_rover/correction_sub_topic", "fix/correction")	
		self.pub_topic = rospy.get_param("gps_correction_rover/pub_topic", "fix")
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
		
		if int(data.tow) == int(correction.tow):
			lat = data.rlat * 180 / math.pi
			lon = data.rlon * 180 / math.pi

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
			self._pub_fix.publish(self.fix)

		else:
			rospy.logerr("GPS Correction received but the Time of Week was not the same as the on board GPS! Not publishing a corrected gps fix.")
			rospy.logerr("TOW Onboard: " + str(data.tow))
			rospy.logerr("TOW Correction: "+ str(correction.tow))

		# getPvt is called again and not used because apparently the data is in a different format when called an even number of times.  The next call will provide data I can currently understand.
		self.gps.getPvt()

if __name__=='__main__':
	rospy.init_node('gps_correction_rover')

	try:
		_GpsRover = GpsRover()
	except rospy.ROSInterruptException: 
		pass