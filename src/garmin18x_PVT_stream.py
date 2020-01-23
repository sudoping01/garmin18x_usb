#! /usr/bin/env python

import sys, os, math
from time import sleep

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/forked_repo")
import garmin

# Create a 'physical layer' connection using serial port
phys = garmin.SerialLink("/dev/sensors/garmin_gps")

# Create a Garmin object using this connection
gps = garmin.Garmin(phys)

# Turn on Position Velocity Time
gps.pvtOn()

sys.stdout.write("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")

while True:
	data = gps.getPvt()
	sys.stdout.write("\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F")
	sys.stdout.write("Fix: " + str(data.fix))
	sys.stdout.write("\nAlt: " + str(data.alt))
	sys.stdout.write("\nLat: " + str(data.rlat * 180 / math.pi))
	sys.stdout.write("\nLon: " + str(data.rlon * 180 / math.pi))
	sys.stdout.write("\n..........." )
	sys.stdout.write("\nEpe: " + str(data.epe))
	sys.stdout.write("\nEph: " + str(data.eph))
	sys.stdout.write("\nEpv: " + str(data.epv))
	sys.stdout.write("\nTow: " + str(data.tow))
	sys.stdout.write("\nEast: " + str(data.east))
	sys.stdout.write("\nNorth: " + str(data.north))
	sys.stdout.write("\nUp: " + str(data.up))
	sys.stdout.write("\nMsl_ht: " + str(data.msl_height))
	sys.stdout.write("\nLp_secs: " + str(data.leap_secs))
	sys.stdout.write("\nWn_dy: " + str(data.wn_days))

	sys.stdout.flush()
	gps.getPvt()
	sleep(1.0)
