#!/usr/bin/env python

import sys
import os
import math
import rospy
from garmin18x_usb.msg import GpsCorrection
from sensor_msgs.msg import NavSatFix

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/pygarmin")
from pygarmin import garmin, link

class GpsRover:

    def __init__(self):
        self.correction_sub_topic = rospy.get_param("gps_correction_rover/correction_sub_topic", "fix/correction")
        self.pub_topic = rospy.get_param("gps_correction_rover/pub_topic", "fix")
        self.pub_rate = rospy.get_param("gps_correction_rover/pub_rate", 1.0)
        dev_path = rospy.get_param("gps_correction_rover/dev_path", "/dev/sensors/garmin_gps")

        # Create a 'physical layer' connection using serial port
        phys = link.SerialLink(dev_path)

        # Create a Garmin object using this connection
        self.gps = garmin.Garmin(phys)
        
        # Turn on Position Velocity Time
        self.gps.pvt_on()

        rospy.Subscriber(self.correction_sub_topic, GpsCorrection, self.correct_fix)
        self._pub_fix = rospy.Publisher(self.pub_topic, NavSatFix, queue_size=1)
        self.fix = NavSatFix()

        rate = rospy.Rate(self.pub_rate)
        
        while not rospy.is_shutdown():
            rate.sleep()

    def correct_fix(self, correction):
        data = self.gps.get_pvt()
        
        if int(data.tow) == int(correction.tow):
            posn = getattr(data, 'posn', [None, None])
            lat = posn[0]
            lon = posn[1]
            
            if lat is not None and lon is not None:
                lat_deg = lat * 180 / math.pi
                lon_deg = lon * 180 / math.pi

                # Check if the current fix is different from the last published fix
                if abs(int(lat_deg * 1e10) - int(self.fix.latitude * 1e10)) > 0 and \
                   abs(int(lon_deg * 1e10) - int(self.fix.longitude * 1e10)) > 0 and \
                   abs(int(getattr(data, 'alt', 0) * 1e10) - int(self.fix.altitude * 1e10)) > 0:
                    self.fix.status.status = 1  # Status: 1 indicates that the GPS fix is valid
                else:
                    self.fix.status.status = 0  # Status: 0 indicates that the GPS fix is not valid

                self.fix.header.stamp = rospy.get_rostime()
                self.fix.header.frame_id = "gps"
                self.fix.status.service = 1
                self.fix.latitude = lat_deg + correction.lat
                self.fix.longitude = lon_deg + correction.lon
                self.fix.altitude = getattr(data, 'alt', 0) + correction.alt
                self.fix.position_covariance = [0] * 9
                self.fix.position_covariance_type = 0
                self._pub_fix.publish(self.fix)

            else:
                rospy.logerr("Latitude or Longitude is None.")
        else:
            rospy.logerr("GPS Correction received but the Time of Week was not the same as the onboard GPS! Not publishing a corrected GPS fix.")
            rospy.logerr(f"TOW Onboard: {data.tow}")
            rospy.logerr(f"TOW Correction: {correction.tow}")

        # get_pvt is called again and not used because the data is in a different format when called an even number of times. 
        self.gps.get_pvt()

if __name__ == '__main__':
    rospy.init_node('gps_correction_rover')

    try:
        GpsRover()
    except rospy.ROSInterruptException:
        pass
