#!/usr/bin/env python

# Gets Twist and updates Task Coordinate Systems (TCS) for each leg
#
# TCS is defined like this:
#                           -- Z[TCS] is parallel to Z[base_link_flat]
#                           -- TCS origin is on the line, which is vertical in [base_link_flat] and passes through center of corresponding leg's workspace
#                           -- TCS origin is at Z[base_link_flat] coordinate corresponding to the level at which the leg is expected to land
#                           -- X[TCS] points in the direction opposite to required direction of leg's push when in stance = in direction of swing




# publishes tf: [base_link_flat] -> [{leg_name}_tcs]
# subscribes to [teleop], [posture_control_rate]

import rospy
from geometry_msgs.msg import Point, TwistStamped, TransformStamped, Quaternion
from std_msgs.msg import Float32
import tf2_ros
from hex_utility import transform_point
from tf_conversions import transformations
from math import atan2

posture_control_rate = 0

twist_queue = list()

leg_names = list()
tcs_definitions = dict()  # [x;y;z;rot_z;stamp] in base_link_flat


def initialize(tf_buffer):
    global leg_names, tcs_definitions

    # place TCS to workspace center
    leg_names = rospy.get_param("leg_names")

    for leg_name in leg_names:
        x = rospy.get_param(leg_name + "_center_x")
        y = rospy.get_param(leg_name + "_center_y")
        z = -1 * rospy.get_param(leg_name + "_attachment_z")
        z_rot = 0

        # these coordinates are in leg's frame, need to convert to base_link_flat
        tf = tf_buffer.lookup_transform("base_link_flat", leg_name, rospy.Time(0), rospy.Duration(10))


        pt_leg = Point(x,y,z)
        pt_base_link_flat = transform_point(pt_leg,tf)

        tcs_definitions[leg_name] = [pt_base_link_flat.x, pt_base_link_flat.y, pt_base_link_flat.z, z_rot, rospy.Time.now()]


def got_posture_control_rate(received_posture_control_rate):
    """

    :type received_posuture_control_rate: Float32
    """

    global posture_control_rate
    posture_control_rate = received_posture_control_rate


def got_twist(recevied_twist):
    """

    :type recevied_twist: TwistStamped
    """

    global twist_queue

    if len(twist_queue) > 5:
        twist_queue.pop(0)

    twist_queue.append(recevied_twist)





def process_twist(tf_buffer, tf_broadcaster):
    """

    :type tf_buffer: tf2_ros.Buffer
    :type tf_broadcaster: tf2_ros.TransformBroadcaster
    """
    global twist_queue, posture_control_rate, tcs_definitions, leg_names


    if len(twist_queue) == 0: return

    twist = twist_queue[0]
    assert isinstance(twist, TwistStamped)

    # check if transforms between [base_link] and [base_link_flat] are available at the time of this twist message
    if not tf_buffer.can_transform("base_link", "base_link_flat", twist.header.stamp, rospy.Duration(1)):
        return
    transform_to_base_link = tf_buffer.lookup_transform("base_link", "base_link_flat", twist.header.stamp)
    transform_to_base_link_flat = tf_buffer.lookup_transform("base_link_flat", "base_link", twist.header.stamp)

    # calculate dx, dy, dz, drot_x, drot_y, drot_z for one cycle -- based on speeds in Twist and posture_control_rate
    dx = -1* twist.twist.linear.x / posture_control_rate.data
    dy = -1* twist.twist.linear.y / posture_control_rate.data
    dz = twist.twist.linear.z / posture_control_rate.data

    drot_x = twist.twist.angular.x / posture_control_rate.data
    drot_y = twist.twist.angular.y / posture_control_rate.data
    drot_z = -1 * twist.twist.angular.z / posture_control_rate.data

    # for a leg:
    for leg_name in leg_names:
        # take TCSorigin (in base_link_flat)
        tcs_origin = Point(*tcs_definitions[leg_name][:3])

        # transform to [base_link]
        tcs_origin_in_base_link = transform_point(tcs_origin, transform_to_base_link)

        # get quaternion to rotate by drot_x, drot_y
        xy_rotation = transformations.quaternion_from_euler(drot_x,drot_y,0)

        # apply drot_x, drot_y rotation
        xy_rotation_transform = TransformStamped()
        xy_rotation_transform.transform.rotation = Quaternion(*xy_rotation)
        xy_rotated = transform_point(tcs_origin_in_base_link, xy_rotation_transform)

        # transform to [base_link_flat]
        xy_rotated_in_base_link_flat = transform_point(xy_rotated, transform_to_base_link_flat)

        # get quaternion to rotate by drot_z
        z_rotation = transformations.quaternion_from_euler(0, 0, drot_z)

        # apply drot_z rotation
        z_rotation_transform = TransformStamped()
        z_rotation_transform.transform.rotation = Quaternion(*z_rotation)
        z_rotated = transform_point(xy_rotated_in_base_link_flat, z_rotation_transform)

        # translate by dx, dy, dz -> (x_new, y_new, z_new)
        new_tcs_base = Point(z_rotated.x + dx, z_rotated.y + dy, z_rotated.z + dz)

        # update TCS definition:
        # (x_neutral; y_neutral; z_new; atan2(neutral_y - y_new, neutral_x - x_new))

        x_offset = tcs_definitions[leg_name][0] - new_tcs_base.x
        if abs(x_offset) < 0.0002: x_offset =0
        y_offset = tcs_definitions[leg_name][1] - new_tcs_base.y
        if abs(y_offset)< 0.0002: y_offset =0

        tcs_definitions[leg_name][2] = new_tcs_base.z
        tcs_definitions[leg_name][3] = atan2(y_offset, x_offset)
        tcs_definitions[leg_name][4] = twist.header.stamp

        tcs_quaternion = transformations.quaternion_from_euler(0, 0, tcs_definitions[leg_name][3])

        tcs_transform = tf2_ros.TransformStamped()
        tcs_transform.header.stamp = tcs_definitions[leg_name][4]
        tcs_transform.header.frame_id = "base_link_flat"
        tcs_transform.child_frame_id = leg_name + "_tcs"

        tcs_transform.transform.translation.x = tcs_definitions[leg_name][0]
        tcs_transform.transform.translation.y = tcs_definitions[leg_name][1]
        tcs_transform.transform.translation.z = tcs_definitions[leg_name][2]

        tcs_transform.transform.rotation.x = tcs_quaternion[0]
        tcs_transform.transform.rotation.y = tcs_quaternion[1]
        tcs_transform.transform.rotation.z = tcs_quaternion[2]
        tcs_transform.transform.rotation.w = tcs_quaternion[3]



        tf_broadcaster.sendTransform(tcs_transform)

    twist_queue.pop(0)


def run():
    rospy.init_node('hex_tcs', anonymous=True)



    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    initialize(tf_buffer)



    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rospy.Subscriber("posture_control_rate", Float32, got_posture_control_rate)
    rospy.Subscriber("teleop", TwistStamped, got_twist)


    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        process_twist(tf_buffer, tf_broadcaster)
        r.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
