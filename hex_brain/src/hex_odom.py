#!/usr/bin/env python
# -*- coding: utf-8 -*-



#        [grounded_legs]                        -->     |                       |       -->         [odom]
#        [tf]: [base_link] -> [..._tarsus]      -->     |      (hex_odom)       |       -->         [tf] : [odom] -> [base_link]
#        [pc_text_commands]                     -->     |                       |       -->         [tf] : [odom] -> [base_link_flat]
#                                                                                                   (base_link_flat = base_link without pitch and roll)



# got grounded_legs -> wait for tf at the stamped time -> see where grounded feet are at the stamped time ->
# -> compare to grounded feet positions during the previous cycle -> get transforms based on point clouds (feet positions) comparison ->
# -> publish results

# [pc_text_commands]: reset_ground_level




import itertools

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Twist, Vector3
from hex_msg.msg import StampedBoolArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_conversions import transformations

from hex_utility import get_transform_from_point_clouds, transform_point

grounded_legs_queue = list()  # stores incoming grounded_legs messages while waiting for tf

latest_odom_translation = [0, 0, 0]
latest_odom_rotation = transformations.quaternion_from_euler(0, 0, 0)

prev_grounded_feet_positions = list(itertools.repeat(None, 6))


def got_text_command(text_command):
    global latest_odom_translation, latest_odom_rotation, prev_grounded_feet_positions

    if text_command.data == "reset_ground_level":
        latest_odom_translation = [0, 0, 0]
        latest_odom_rotation = transformations.quaternion_from_euler(0, 0, 0)
        prev_grounded_feet_positions = list(itertools.repeat(None, 6))


def got_grounded_legs(received_grounded_legs):
    """

    :type received_grounded_legs: LegStates
    """

    # add the new received grounded_legs message to a queue
    # make sure that queue never longer than [max_queue_length] messages

    max_queue_length = 20

    if len(grounded_legs_queue) == max_queue_length:
        grounded_legs_queue.pop(0)  # dump the oldest unprocessed message

    grounded_legs_queue.append(received_grounded_legs)


def broadcast_tf(current_odom_translation, current_odom_rotation, tf_broadcaster, pub_time):
    # broadcast tf for odom->base_link
    odom_to_base_transform = tf2_ros.TransformStamped()
    odom_to_base_transform.header.stamp = pub_time
    odom_to_base_transform.header.frame_id = "odom"
    odom_to_base_transform.child_frame_id = "base_link"

    odom_to_base_transform.transform.translation.x = current_odom_translation[0]
    odom_to_base_transform.transform.translation.y = current_odom_translation[1]
    odom_to_base_transform.transform.translation.z = current_odom_translation[2]

    odom_to_base_transform.transform.rotation.x = current_odom_rotation[0]
    odom_to_base_transform.transform.rotation.y = current_odom_rotation[1]
    odom_to_base_transform.transform.rotation.z = current_odom_rotation[2]
    odom_to_base_transform.transform.rotation.w = current_odom_rotation[3]

    tf_broadcaster.sendTransform(odom_to_base_transform)

    # broadcast tf for odom->base_link_flat
    odom_to_base_link_flat_transform = tf2_ros.TransformStamped()
    odom_to_base_link_flat_transform.header.stamp = pub_time
    odom_to_base_link_flat_transform.header.frame_id = "odom"
    odom_to_base_link_flat_transform.child_frame_id = "base_link_flat"

    odom_to_base_link_flat_transform.transform.translation.x = current_odom_translation[0]
    odom_to_base_link_flat_transform.transform.translation.y = current_odom_translation[1]
    odom_to_base_link_flat_transform.transform.translation.z = current_odom_translation[2]

    odom_to_base_link_flat_transform.transform.rotation = Quaternion(*transformations.quaternion_from_euler(0, 0,
                                                                                                            transformations.euler_from_quaternion(
                                                                                                                current_odom_rotation)[2]))

    tf_broadcaster.sendTransform(odom_to_base_link_flat_transform)


def foot_position_in_odom(transform_to_base_link, current_odom_translation, current_odom_rotation, tarsus_length):
    """

    :type point_base_link: Point


    """

    # get foot coordinates in tarsus
    foot_coordinates_in_tarsus = Point()
    foot_coordinates_in_tarsus.x = tarsus_length
    foot_coordinates_in_tarsus.y = 0
    foot_coordinates_in_tarsus.z = 0

    # use the stored definition of odom->base_link relationship to get transform to odom

    transform_from_tarsus_to_odom = TransformStamped()
    transform_from_tarsus_to_odom.transform.translation.x = transform_to_base_link.transform.translation.x + \
                                                            current_odom_translation[0]
    transform_from_tarsus_to_odom.transform.translation.y = transform_to_base_link.transform.translation.y + \
                                                            current_odom_translation[1]
    transform_from_tarsus_to_odom.transform.translation.z = transform_to_base_link.transform.translation.z + \
                                                            current_odom_translation[2]

    transform_from_tarsus_to_odom.transform.rotation = Quaternion(
        *transformations.quaternion_multiply((transform_to_base_link.transform.rotation.x,
                                              transform_to_base_link.transform.rotation.y,
                                              transform_to_base_link.transform.rotation.z,
                                              transform_to_base_link.transform.rotation.w),
                                             transformations.quaternion_inverse(current_odom_rotation)))

    return transform_point(foot_coordinates_in_tarsus, transform_from_tarsus_to_odom)


def run():
    global grounded_legs_queue, latest_odom_translation, latest_odom_rotation, prev_grounded_feet_positions

    rospy.init_node('hex_odom', anonymous=True)

    rospy.Subscriber("pc_text_commands", String, got_text_command)
    rospy.Subscriber("grounded_legs", StampedBoolArray, got_grounded_legs)

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    broadcast_tf(latest_odom_translation, latest_odom_rotation, tf_broadcaster, rospy.Time.now())

    leg_names = rospy.get_param("leg_names")
    tarsus_length = rospy.get_param("tarsus_length")

    processed_grounded_legs = None

    transforms_from_tarsus_to_base_link = list(itertools.repeat(None, 6))

    prev_odometry_time = None  # used to calculate velocity

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        if len(grounded_legs_queue) == 0:
            continue

        # get the oldest leg states message from the queue and try to wait till transforms for all stance legs become available
        processed_grounded_legs = grounded_legs_queue.pop(0)
        processed_time = processed_grounded_legs.stamp

        present_grounded_legs_positions = [None for i in range(6)]

        for i in range(6):
            # we only need transforms for grounded legs
            if not processed_grounded_legs.values[i]:
                continue

            leg_name = leg_names[i]

            # wait for transform to arrive
            while not tf_buffer.can_transform("base_link", leg_name + "_tarsus", processed_time, rospy.Duration(1)):
                continue

            # transform to base_link is available
            transforms_from_tarsus_to_base_link[i] = tf_buffer.lookup_transform("base_link", leg_name + "_tarsus", processed_time)

            # use the stored definition of odom->base_link relationship to get transform to odom
            present_grounded_legs_positions[i] = foot_position_in_odom(transforms_from_tarsus_to_base_link[i],
                                                                       latest_odom_translation, latest_odom_rotation, tarsus_length)

        # now we have odom foot coordinates for all legs that were grounded at [processed time]


        # get cycle duration
        if prev_odometry_time == None:
            cycle_duration = rospy.Duration(1000).to_sec()
        else:
            cycle_duration = (rospy.Time.now() - prev_odometry_time).to_sec()
        prev_odometry_time = rospy.Time.now()

        # which feet were grounded previously and remain grounded at [processed_time]?
        # prepare two point clouds from previous and present positions

        prev_points = list()
        present_points = list()

        for i in range(6):
            if prev_grounded_feet_positions[i] != None and present_grounded_legs_positions[i] != None:
                prev_points.append((prev_grounded_feet_positions[i].x,
                                    prev_grounded_feet_positions[i].y,
                                    prev_grounded_feet_positions[i].z))

                present_points.append((present_grounded_legs_positions[i].x,
                                       present_grounded_legs_positions[i].y,
                                       present_grounded_legs_positions[i].z))

        if len(prev_points) != 0:

            # we are dealing with grounded feet rigidly linked through ground -- so they should ideally not move in relation to each other (if there's no slip)

            # compare point clouds to get translation and rotation between their frames
            push_translation, push_rotation = get_transform_from_point_clouds(prev_points, present_points)

            # we assume that grounded legs are firmly on the ground,
            # so push_translation and push_rotation mean changes in base_link pose in odom caused by grounded feet pushes

            push_trans_x = push_translation[0]
            push_trans_y = push_translation[1]
            push_trans_z = push_translation[2]

            push_rot_x = -push_rotation[0]
            push_rot_y = -push_rotation[1]
            push_rot_z = -push_rotation[2]

            push_translation = [push_trans_x, push_trans_y, push_trans_z]
            push_rotation_quat = transformations.quaternion_from_euler(push_rot_x, push_rot_y, push_rot_z)

            # update odom->base_link definition by applying push translation and rotation
            for i in range(0, 3):
                latest_odom_translation[i] -= push_translation[i]
            latest_odom_rotation = transformations.quaternion_multiply(push_rotation_quat, latest_odom_rotation)

            # odom definition was changed, so present_grounded_legs_positions are no longer correct
            # we need to update them before they can be saved to prev_grounded_feet_positions
            for i in range(0, 6):
                if not present_grounded_legs_positions[i] is None:
                    present_grounded_legs_positions[i] = foot_position_in_odom(transforms_from_tarsus_to_base_link[i],
                                                                               latest_odom_translation,
                                                                               latest_odom_rotation, tarsus_length)

            # broadcast tf (odom->base_link and odom->base_link_flat)
            broadcast_tf(latest_odom_translation, latest_odom_rotation, tf_broadcaster, processed_time)

            # publish odom message
            odom = Odometry()

            odom.header.stamp = processed_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose = Pose(Point(*latest_odom_translation), Quaternion(*latest_odom_rotation))
            odom.twist.twist = Twist(Vector3(*tuple(x / cycle_duration for x in push_translation)),
                                     Vector3(*tuple(x / cycle_duration for x in push_rotation[:3])))

            # publish the message
            odom_pub.publish(odom)

        prev_grounded_feet_positions = present_grounded_legs_positions

        r.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
