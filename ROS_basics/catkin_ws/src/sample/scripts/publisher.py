#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('topic', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = "you are awesome..! %s" % rospy.get_time()
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
