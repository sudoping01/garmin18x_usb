## Summary

The goal of this repo is to provide code that relaibly connects to and extracts usable data from a USB Garmin 18x GPS.

Currently supports Ubuntu 18.04 running ROS Melodic

---------

## Setup:

1. Clone the repo recursively into wherever you want. 
`gitclone --recursive https://github.com/chriswsuarez/garmin18x_usb.git`
2. Ensure the garmin_gps driver is not blacklisted in modprobe. Comment out any line that says `blacklist garmin_gps` in the following file path.
`sudo nano /etc/modprobe.d/blacklist.conf`
4. To ensure the garmin gps always comes up with the proper /dev path, copy the file 51-garmin.rules from the setup folder to the path /etc/udev/rules.d/: 
`cp setup/51-garmin.rules /etc/udev/rules.d/`
4. Once the udev rule is in place either restart the computer or reload the udev rules.  This can be done in the terminal window, however it has not been successful for me, so I just restart after adding udev rules.
`sudo restart`
5. Plug in the USB Garmin 18x and run whichever script you desire. 
`./src/ros_stream.py` or `./src/terminal_stream.py`

---------

## ros_stream.py

This script grabs the raw GPS PVT data and publishes a ros topic of type NavSatFix.  It uses rosparam set in `config/ros_stream.yaml` to determine which topic to publish on which dev path to look for the garmin gps and which frame_id the data is associated with.  The publish rate can also be changed however in limited testing it has not been able to publish faster than 1 Hz.  It can publish less frequently.

`launch/ros_stream.launch` gives an example of how to run the script and load the rosparams from the yaml file in one command: `roslaunch garmin18x_usb ros_stream.launch`

---------

## terminal_stream.py

This simple PVT stream script reports the raw GPS PVT data onto the terminal window.  It is a good script to reference when developing future code to get live data from the gps.

---------

## PVT data:

Some of the Garmin receivers support a PVT mode as part of the Garmin mode. If you are using a computer program that supports this then you can remain in Garmin mode even while running your real time mapping application. You set your unit to Garmin mode and then select this solution from the menus in the application. Delorme mapping products support this mode. This is an advantage in that you don't need to switch modes and you can leave your interface at 9600 baud which makes the real time response a bit faster. The update interval is 1 second and this mode does not require handshaking nor does it support retransmission of data. The following data is typically included as part of the pvt structure in the D800 message:

    alt - Altitude above WGS-84 ellipsoid
    epe - total predicted error (2 sigma meters)
    eph - horizontal position error
    epv - vertical position error
    fix - type of position fix
    tow - time of week (seconds)
    posn - lat/lon (radians)
    east - velocity east (meters/sec)
    north - velocity north (meters/sec)
    up - velocity up (meters/sec)
    msl_height - height of WGS-84 ellipsoid above MSL (meters)
    leap_seconds - difference between gps time and UTC (seconds)
    wn_days - week number days
