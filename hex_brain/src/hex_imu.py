#!/usr/bin/env python

# temporary placeholder, will be replaced with UM7


# publishes [rpy]

import rospy

from geometry_msgs.msg import Vector3Stamped




def run():
    rospy.init_node('hex_imu', anonymous=True)

    pub = rospy.Publisher('imu/rpy', Vector3Stamped, queue_size=10)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        current_rpy = Vector3Stamped()
        current_rpy.header.stamp = rospy.Time.now()
        current_rpy.vector.x = 0
        current_rpy.vector.y = 0
        current_rpy.vector.z = 0

        pub.publish(current_rpy)


        r.sleep()








if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass