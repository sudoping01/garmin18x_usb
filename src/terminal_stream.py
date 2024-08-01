##!/usr/bin/env python

import sys
import os
import math
from time import sleep

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/pygarmin")
from pygarmin import garmin, link

# Create a 'physical layer' connection using serial port
phys = link.SerialLink("/dev/sensors/garmin_gps")

# Create a Garmin object using this connection
gps = garmin.Garmin(phys)

# Turn on Position Velocity Time
gps.pvt_on()

# Clear the screen for fresh output
sys.stdout.write("\n" * 20)

while True:
    try:
        data = gps.get_pvt()

        # Clear previous data
        sys.stdout.write("\033[F" * 15)

        # Extract and display GPS data
        sys.stdout.write(f"Fix: {getattr(data, 'fix', 'N/A')}\n")
        sys.stdout.write(f"Alt: {getattr(data, 'alt', 'N/A')}\n")

        # Latitude and Longitude in degrees
        posn = getattr(data, 'posn', [None, None])
        lat = posn[0]
        lon = posn[1]
        if lat is not None and lon is not None:
            lat_deg = lat * 180 / math.pi
            lon_deg = lon * 180 / math.pi
            sys.stdout.write(f"Lat: {lat_deg}\n")
            sys.stdout.write(f"Lon: {lon_deg}\n")
        else:
            sys.stdout.write("Lat: N/A\nLon: N/A\n")

        # Additional data (update if necessary)
        sys.stdout.write("...........\n")
        sys.stdout.write(f"Epe: {getattr(data, 'epe', 'N/A')}\n")
        sys.stdout.write(f"Eph: {getattr(data, 'eph', 'N/A')}\n")
        sys.stdout.write(f"Epv: {getattr(data, 'epv', 'N/A')}\n")
        sys.stdout.write(f"Tow: {getattr(data, 'tow', 'N/A')}\n")
        sys.stdout.write(f"East: {getattr(data, 'east', 'N/A')}\n")
        sys.stdout.write(f"North: {getattr(data, 'north', 'N/A')}\n")
        sys.stdout.write(f"Up: {getattr(data, 'up', 'N/A')}\n")
        sys.stdout.write(f"Msl_ht: {getattr(data, 'msl_height', 'N/A')}\n")
        sys.stdout.write(f"Lp_secs: {getattr(data, 'leap_secs', 'N/A')}\n")
        sys.stdout.write(f"Wn_dy: {getattr(data, 'wn_days', 'N/A')}\n")
        sys.stdout.flush()

    except Exception as e:
        print(f"Unexpected error: {e}")

    sleep(1.0)
