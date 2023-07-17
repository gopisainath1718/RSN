#!/usr/bin/env python3
import utm
from serial import Serial
import rospy
from gnss_driver.msg import gps_msg
import sys

#lat 42.338545 long -71.086209
i = 0
def driver():
    argument = rospy.myargv(argv=sys.argv)
    number = argument[1]
    port1 = Serial(number, 4800, timeout=100.00)

    msg = gps_msg()
    msg.Header.frame_id = 'GPS1_Frame'
    # seq = -1
    rospy.init_node('driver', anonymous=True)
    pub = rospy.Publisher('/gps', gps_msg, queue_size=100)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        data = str(port1.readline())
        if 'GNGGA' in data:
            values = data.split(',')
            utc, latitude, longitude, hdop, altitude, fix_quality = values[1], values[2], values[4],values[8], values[9], values[6]
            if latitude != "" and longitude != "" and hdop != "":
                latitude = float(latitude[:2])+(float(latitude[2:])/60)
                longitude = (float(longitude[:3])+(float(longitude[3:])/60))*-1

                a = utm.from_latlon(latitude, longitude)

                utc_hrs = float(utc)//10000
                utc_mint = (float(utc)-(utc_hrs*10000))//100
                utc_sec = (float(utc) - (utc_hrs*10000) - (utc_mint*100))
                utc_final_secs = float((utc_hrs*3600 + utc_mint*60 + utc_sec))
                utc_final_nsecs = int((utc_final_secs * (10*7)) % (10*7))

                msg.Header.seq += 1
                msg.Header.stamp.secs = int(utc_final_secs)
                msg.Header.stamp.nsecs = int(utc_final_nsecs)
                msg.Latitude = latitude
                msg.Longitude = longitude
                msg.Altitude = float(altitude)
                msg.HDOP = float(hdop)
                msg.UTM_easting = a[0]
                msg.UTM_northing = a[1]
                msg.Zone = a[2]
                msg.Letter = a[3]
                msg.UTC = float(utc)  
                msg.fix_quality = int(fix_quality)  
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
        