#!/usr/bin/env python

import sys
import os
import math
import rospy
from sensor_msgs.msg import NavSatFix

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/pygarmin")
from pygarmin import garmin, link

class GarminToRos:

    def __init__(self):

        # Ros Parameters
        self.pub_topic = rospy.get_param("garmin_gps/pub_topic", "/fix")
        self.frame_id = rospy.get_param("garmin_gps/frame_id", "gps")
        self.pub_rate = rospy.get_param("garmin_gps/pub_rate", 1.0)
        dev_path = rospy.get_param("garmin_gps/dev_path", "/dev/sensors/garmin_gps")

        # Create a 'physical layer' connection using serial port
        phys = link.SerialLink(dev_path)

        # Create a Garmin object using this connection
        gps = garmin.Garmin(phys)
        
        # Turn on Position Velocity Time
        gps.pvt_on()

        # Publisher for NavSatFix messages
        self._pub_fix = rospy.Publisher(self.pub_topic, NavSatFix, queue_size=1)

        # Initialize NavSatFix message
        self.fix = NavSatFix()

        rate = rospy.Rate(self.pub_rate)

        while not rospy.is_shutdown():
            try:
                data = gps.get_pvt()

                # Extract latitude and longitude in degrees
                posn = getattr(data, 'posn', [None, None])
                lat = posn[0]
                lon = posn[1]

                if lat is not None and lon is not None:
                    lat_deg = lat * 180 / math.pi
                    lon_deg = lon * 180 / math.pi

                    # Check if there is a significant change in position
                    if (abs(int(lat_deg * 1e10) - int(self.fix.latitude * 1e10)) > 0 or 
                        abs(int(lon_deg * 1e10) - int(self.fix.longitude * 1e10)) > 0 or
                        abs(int(getattr(data, 'alt', 0) * 1e10) - int(self.fix.altitude * 1e10)) > 0):
                        self.fix.status.status = 1  # Status: 1 indicates that the GPS fix is valid
                    else:
                        self.fix.status.status = 0  # Status: 0 indicates that the GPS fix is not valid

                    # Update NavSatFix message
                    self.fix.header.stamp = rospy.get_rostime()
                    self.fix.header.frame_id = self.frame_id
                    self.fix.status.service = 1
                    self.fix.latitude = lat_deg
                    self.fix.longitude = lon_deg
                    self.fix.altitude = getattr(data, 'alt', 0)
                    self.fix.position_covariance = [0] * 9
                    self.fix.position_covariance_type = 0

                    # Publish the updated GPS fix
                    self._pub_fix.publish(self.fix)
                else:
                    rospy.logerr("Latitude or Longitude is None.")

            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('garmin_gps')

    try:
        GarminToRos()
    except rospy.ROSInterruptException:
        pass
