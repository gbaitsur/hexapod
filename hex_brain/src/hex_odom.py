#!/usr/bin/env python




# calculates pose in odom based on stance legs movements
# publishes odom



import rospy
import tf2_ros
from geometry_msgs.msg import Point,TransformStamped,Quaternion,PointStamped


from nav_msgs.msg import Odometry

from hex_msg.msg import LegStates
from tf_conversions import transformations

from hex_utility import transform_point, get_transform_from_point_clouds


import message_filters


leg_states_queue = list()


def gotLegStates(received_leg_states):
    """

    :type received_leg_states: LegStates
    """

    # add the new received leg states message to a queue
    # make sure that queue never longer than [max_queue_length] messages

    max_queue_length = 10

    if len(leg_states_queue) > 9:
        leg_states_queue.pop(0) # dump the oldest unprocessed message


    leg_states_queue.append(received_leg_states)








def run():
    global leg_states_queue, current_odom_rotation, current_odom_translation, base_link_to_odom_translation_matrix, base_link_to_odom_rotation_matrix, base_link_to_odom_matrix

    rospy.init_node('hex_odom', anonymous=True)

    rospy.Subscriber("leg_states", LegStates, gotLegStates)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    current_odom_translation = [0,0,0]
    current_odom_rotation = transformations.quaternion_from_euler(0,0,0)

    prev_stance_feet_positions = list((None,None,None,None,None,None))


    leg_names = ["rf","rm","rr","lf","lm","lr"]



    tarsus_length = rospy.get_param("tarsus_length")


    processed_leg_states = None


    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        if len(leg_states_queue) == 0:
            continue

        # get the oldest leg states message from the queue and try to wait till transforms for all stance legs become available
        # if all transforms do not arrive within set timeout -- dump this message and try with the next one
        processed_leg_states = leg_states_queue.pop(0)
        processed_time = processed_leg_states.stamp

        current_stance_feet_positions = list((None,None,None,None,None,None))

        for i in range (0,6):
            # we only need transforms for stance legs
            if not processed_leg_states.stance_legs[i]:
                continue

            leg_name = leg_names[i]
            if not tf_buffer.can_transform("base_link",leg_name + "_tarsus", processed_time, rospy.Duration(1)):
                continue



            # transform is available -- let's get foot coordinates in base_link

            transform_to_base_link = tf_buffer.lookup_transform("base_link",leg_name + "_tarsus",processed_time)

            transform_from_tarsus_to_odom = TransformStamped()




            transform_from_tarsus_to_odom.transform.translation.x = transform_to_base_link.transform.translation.x + current_odom_translation[0]
            transform_from_tarsus_to_odom.transform.translation.y = transform_to_base_link.transform.translation.y + current_odom_translation[1]
            transform_from_tarsus_to_odom.transform.translation.z = transform_to_base_link.transform.translation.z + current_odom_translation[2]


            transform_from_tarsus_to_odom.transform.rotation = Quaternion(*transformations.quaternion_multiply((transform_to_base_link.transform.rotation.x,
                                                                                                    transform_to_base_link.transform.rotation.y,
                                                                                                    transform_to_base_link.transform.rotation.z,
                                                                                                    transform_to_base_link.transform.rotation.w),
                                                                                                    transformations.quaternion_inverse(current_odom_rotation)))









            # get foot coordinates in odom
            foot_coordinates_in_tarsus = Point()
            foot_coordinates_in_tarsus.x = tarsus_length
            foot_coordinates_in_tarsus.y = 0
            foot_coordinates_in_tarsus.z = 0

            foot_coordinates_in_odom = transform_point(foot_coordinates_in_tarsus, transform_from_tarsus_to_odom)


            current_stance_feet_positions[i] = foot_coordinates_in_odom


        # now we have odom foot coordinates for all legs that were in stance at [processed time]=now

        # which feet were in stance previously and remain in stance now?
        # prepare two point clouds from previous and current positions

        prev_points = list()
        current_points = list()

        for i in range(0,6):
            if prev_stance_feet_positions[i]!=None and current_stance_feet_positions[i]!=None:
                prev_points.append((prev_stance_feet_positions[i].x,
                                    prev_stance_feet_positions[i].y,
                                    prev_stance_feet_positions[i].z))

                current_points.append((current_stance_feet_positions[i].x,
                                       current_stance_feet_positions[i].y,
                                       current_stance_feet_positions[i].z))





        if len(prev_points) != 0:

            push_translation, push_rotation = get_transform_from_point_clouds(prev_points, current_points)

            push_trans_x = round(push_translation[0], 4)
            push_trans_y = round(push_translation[1], 4)
            push_trans_z = round(push_translation[2], 4)

            push_rot_x = round(push_rotation[0], 4)
            push_rot_y = round(push_rotation[1], 4)
            push_rot_z = round(push_rotation[2], 4)


            push_translation = [push_trans_x, push_trans_y, push_trans_z]
            push_rotation_quat = transformations.quaternion_from_euler(push_rot_x, push_rot_y, push_rot_z)




            for i in range(0,3):
                current_odom_translation[i] -= push_translation[i]


            current_odom_rotation = transformations.quaternion_multiply(push_rotation_quat,current_odom_rotation)


            odom_to_base_transform = tf2_ros.TransformStamped()
            odom_to_base_transform.header.stamp = processed_time
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

            # since odom definition was updated, we need to update current_stance_feet_positions correspondingly
            # before they can be saved to prev_stance_feet_positions
            push_transform = TransformStamped()
            push_transform.transform.translation.x = -push_trans_x
            push_transform.transform.translation.y = -push_trans_y
            push_transform.transform.translation.z = -push_trans_z

            push_transform.transform.rotation = Quaternion(*transformations.quaternion_from_euler(-push_rot_x, -push_rot_y, -push_rot_z))

            for i in range(0,6):
                if not current_stance_feet_positions[i] is None:
                    current_stance_feet_positions[i] = transform_point(current_stance_feet_positions[i] , push_transform)



        prev_stance_feet_positions = current_stance_feet_positions



        r.sleep()




if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass



