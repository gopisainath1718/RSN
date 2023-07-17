#!/usr/bin/env python3
from __future__ import print_function
from serial import Serial
import rospy
from imu_driver.msg import Vectornav
from imu_driver.srv import conversion
import sys

def driver():
    argument = rospy.myargv(argv=sys.argv)
    number = argument[1]
    port = Serial(number, 115200, timeout=10.0)
    port.write(b"$VNWRG,07,40*XX")
    msg = Vectornav()
    

    rospy.init_node('driver', anonymous=True)
    pub = rospy.Publisher('/imu', Vectornav, queue_size=100)
    rate = rospy.Rate(40) 
    while not rospy.is_shutdown():
        # print("hi")
        data = port.readline()
        if b'VNYMR' in data:
            values = data.split(b',')
            yaw, pitch, roll, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y = [float(values[i]) for i in range(1, len(values) - 1)]
            gyro_z = float(values[-1].decode("utf-8").split('*')[0])

            rospy.wait_for_service('conversion')
            try:
                convert = rospy.ServiceProxy('conversion', conversion)
                response = convert(roll,pitch,yaw)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            msg.imu.header.frame_id = 'imu1_frame'
            msg.imu.header.seq +=1
            msg.imu.header.stamp.secs=rospy.get_rostime().secs
            msg.imu.header.stamp.nsecs=rospy.get_rostime().nsecs
                        
            msg.imu.orientation.x= float(response.x)
            msg.imu.orientation.y= float(response.y)
            msg.imu.orientation.z= float(response.z)
            msg.imu.orientation.w= float(response.w)

            msg.imu.angular_velocity.x= gyro_x
            msg.imu.angular_velocity.y= gyro_y
            msg.imu.angular_velocity.z= gyro_z
            
            msg.imu.linear_acceleration.x= accel_x
            msg.imu.linear_acceleration.y= accel_y
            msg.imu.linear_acceleration.z= accel_z
            
            msg.mag_field.header.frame_id = 'imu1_frame'
            msg.mag_field.header.seq +=1
            msg.mag_field.header.stamp.secs=rospy.get_rostime().secs
            msg.mag_field.header.stamp.nsecs=rospy.get_rostime().nsecs
            msg.mag_field.magnetic_field.x= mag_x
            msg.mag_field.magnetic_field.y= mag_y
            msg.mag_field.magnetic_field.z= mag_z

            msg.Header.frame_id = 'imu1_frame'
            msg.Header.seq +=1
            msg.Header.stamp.secs=rospy.get_rostime().secs
            msg.Header.stamp.nsecs=rospy.get_rostime().nsecs
            msg.raw_string= str(data.decode("utf-8"))

            print(msg)
            pub.publish(msg)
            # print(float(values[10]))
            # print(float(values[11]))
            # print(float(values[12][0:9]))
            rate.sleep()

        

if  __name__ == "__main__":
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
        