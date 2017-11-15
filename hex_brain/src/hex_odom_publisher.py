#!/usr/bin/env python




# subscribes to [rpy] from UM7
# subscribes to [legs_ground_contact] from Dynamixel
# updates /base_link orientation in /odom based on [rpy]
# checks coordinates of all tips of the legs in contact with ground to find the lowest one
# updates /base_link Z-coordinate in /odom so that the lowest leg tip is at /odom's Z=0
# publishes tf and odom

import rospy
import tf2_ros

from nav_msgs.msg import Odometry

from hex_msg.msg import LegStates
from geometry_msgs.msg import Vector3Stamped, PointStamped, TransformStamped

from hex_utility import transform_point_stamped

latest_leg_states = LegStates()
latest_rpy = Vector3Stamped()

current_xyz = 0, 0, 0


def gotLegStates(received_leg_states):
    global latest_leg_states
    latest_leg_states = received_leg_states


def gotRPY(received_rpy):
    global latest_rpy
    latest_rpy = received_rpy


def legTipZInTempOdom(leg_name, tf_buffer):
    """

    :type tf_buffer: tf2_ros.Buffer
    """
    try:
        leg_tip = PointStamped()
        leg_tip.header.frame_id = leg_name + "_tarsus3"
        leg_tip.header.stamp = rospy.Time(0)
        leg_tip.point.x = 0.055
        leg_tip.point.y = 0.0
        leg_tip.point.z = 0.0

        p = transform_point_stamped(tf_buffer, "temp_odom", leg_tip)
        return p.point.z

    except:
        return 0


def run():
    global current_xyz
    global latest_rpy

    rospy.init_node('hex_odom_publisher', anonymous=True)

    rospy.Subscriber("leg_states", LegStates, gotLegStates)
    rospy.Subscriber("imu/rpy", Vector3Stamped, gotRPY)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    odom_transform_broadcaster = tf2_ros.TransformBroadcaster()

    r = rospy.Rate(100)

    while not rospy.is_shutdown():

        pub_time = rospy.Time.now()

        t = TransformStamped()
        t.header.stamp = pub_time
        t.header.frame_id = "base_link"
        t.child_frame_id = "temp_odom"
        t.transform.rotation.x = -latest_rpy.vector.x
        t.transform.rotation.y = -latest_rpy.vector.y
        t.transform.rotation.z = -latest_rpy.vector.z
        t.transform.rotation.w = 1
        odom_transform_broadcaster.sendTransform(t)

        min_z = 1000
        leg_names = ["rf", "rm", "rr", "lf", "lm", "lr"]
        for leg_name in leg_names:
            z = legTipZInTempOdom(leg_name, tf_buffer)

            if min_z > z:
                min_z = z


        t = TransformStamped()
        t.header.stamp = pub_time
        t.header.frame_id = "base_link"
        t.child_frame_id = "odom"
        t.transform.translation.x = current_xyz[0]
        t.transform.translation.y = current_xyz[1]
        t.transform.translation.z = min_z
        t.transform.rotation.x = latest_rpy.vector.x
        t.transform.rotation.y = latest_rpy.vector.y
        t.transform.rotation.z = latest_rpy.vector.z
        t.transform.rotation.w = 1
        odom_transform_broadcaster.sendTransform(t)

        print min_z


        r.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
