#!/usr/bin/env python

import sys
import os
import math
import rospy
from garmin18x_usb.msg import GpsCorrection
# from sensor_msgs.msg import NavSatFix

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/pygarmin")
from pygarmin import garmin, link

class FixCorrection:

    def __init__(self):
        # ROS Parameters
        self.pub_topic = rospy.get_param("gps_correction_base/pub_topic", "fix/correction")
        self.base_lat = rospy.get_param("gps_correction_base/base_lat", 30.385178399728332)
        self.base_lon = rospy.get_param("gps_correction_base/base_long", -97.72850900888446)
        self.base_alt = rospy.get_param("gps_correction_base/base_alt", 243.0)
        self.pub_rate = rospy.get_param("gps_correction_base/pub_rate", 1.0)
        dev_path = rospy.get_param("gps_correction_base/dev_path", "/dev/sensors/garmin_gps")

        # Create a 'physical layer' connection using serial port
        phys = link.SerialLink(dev_path)

        # Create a Garmin object using this connection
        gps = garmin.Garmin(phys)
        
        # Turn on Position Velocity Time
        gps.pvt_on()

        # Publisher and Messages
        self._pub_correction = rospy.Publisher(self.pub_topic, GpsCorrection, queue_size=1)
        self.current_correction = GpsCorrection()

        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            try:
                data = gps.get_pvt()
                
                # Latitude and Longitude in degrees
                posn = getattr(data, 'posn', [None, None])
                lat = posn[0]
                lon = posn[1]

                # Check if lat/lon is available
                if lat is not None and lon is not None:
                    lat_deg = lat * 180 / math.pi
                    lon_deg = lon * 180 / math.pi

                    # Correction applied and published
                    self.current_correction.lat = lat_deg - self.base_lat
                    self.current_correction.lon = lon_deg - self.base_lon
                else:
                    self.current_correction.lat = 0.0
                    self.current_correction.lon = 0.0

                self.current_correction.alt = getattr(data, 'alt', 0.0) - self.base_alt
                self.current_correction.tow = getattr(data, 'tow', 0.0)
                self._pub_correction.publish(self.current_correction)

            except Exception as e:
                rospy.logerr(f"Error getting GPS data: {e}")

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gps_correction_base')

    try:
        FixCorrection()
    except rospy.ROSInterruptException:
        pass
