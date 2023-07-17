#!/usr/bin/env python3
import utm
from serial import Serial
import rospy
from gps_driver.msg import gps_msg
import sys

# lat 42.33709
#lon -71.090029

def driver():

    argument = rospy.myargv(argv=sys.argv)
    number = argument[1]
    port1 = Serial(number, 4800, timeout=100.00)

    msg = gps_msg()
    msg.Header.frame_id = 'GPS1_Frame'
    # seq = -1
    rospy.init_node('driver', anonymous=True)
    pub = rospy.Publisher('/gps', gps_msg, queue_size=10)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        data = str(port1.readline())

        if 'GPGGA' in data:

            values = data.split(',')
            # print(values[2], values[4])
            utc, latitude, longitude, hdop, altitude = values[1], values[2], values[4],values[8], values[9]
            if latitude != "" and longitude != "" and hdop != "":
                latitude = float(latitude[:2])+(float(latitude[2:])/60)
                longitude = (float(longitude[:3])+(float(longitude[3:])/60))*-1

                a = utm.from_latlon(latitude, longitude)
                print(utc[7:])
                print(utc)
                seconds = float(utc[:2])*3600 + float(utc[2:4])*60 + float(utc[4:6])
                dec_seconds = float(utc[7:])*10e6

                msg.Header.seq += 1
                msg.Header.stamp.secs = int(seconds)
                msg.Header.stamp.nsecs = int(dec_seconds)
                msg.Latitude = latitude
                msg.Longitude = longitude
                msg.Altitude = float(altitude)
                msg.HDOP = float(hdop)
                msg.UTM_northing = a[0]
                msg.UTM_easting = a[1]
                msg.Zone = a[2]
                msg.Letter = a[3]
                msg.UTC = float(utc)    

                print(msg)

                pub.publish(msg)
                rate.sleep()
            else:
                pass

if  __name__ == "__main__":
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
        
