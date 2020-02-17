#!/usr/bin/env python

import sys, os, math
import rospy
from garmin18x_usb.msg import GpsCorrection
# from sensor_msgs.msg import NavSatFix

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/pygarmin")
import garmin

class FixCorrection:

	def __init__(self):

		#Ros Parameters
		self.pub_topic = rospy.get_param("gps_correction_base/pub_topic", "fix/correction")
		self.base_lat = rospy.get_param("gps_correction_base/base_lat", 30.385178399728332)
		self.base_lon = rospy.get_param("gps_correction_base/base_long", -97.72850900888446)
		self.base_alt = rospy.get_param("gps_correction_base/base_alt", 243.0)
		self.pub_rate = rospy.get_param("gps_correction_base/pub_rate", 1.0)
		dev_path = rospy.get_param("gps_correction_base/dev_path", "/dev/sensors/garmin_gps")

		# Create a 'physical layer' connection using serial port
		phys = garmin.SerialLink(dev_path)

		# Create a Garmin object using this connection
		gps = garmin.Garmin(phys)
		
		# Turn on Position Velocity Time
		gps.pvtOn()

		# Publisher and Msgs
		self._pub_correction = rospy.Publisher(self.pub_topic, GpsCorrection, queue_size=1)
		self.current_correction = GpsCorrection()

		rate = rospy.Rate(self.pub_rate)
		while not rospy.is_shutdown():

			data = gps.getPvt()
			lat = data.rlat * 180 / math.pi
			lon = data.rlon * 180 / math.pi

			# Correction applied and published
			self.current_correction.lat = lat - self.base_lat
			self.current_correction.lon = lon - self.base_lon
			self.current_correction.alt = data.alt - self.base_alt
			self.current_correction.tow = data.tow
			self._pub_correction.publish(self.current_correction)

			# getPvt is called again and not used because apparently the data is in a different format when called an even number of times.  The next call will provide data I can currently understand.
			gps.getPvt()
			rate.sleep()

if __name__=='__main__':
	rospy.init_node('gps_correction_base')

	try:
		_GpsCorrection = FixCorrection()
	except rospy.ROSInterruptException: 
		pass