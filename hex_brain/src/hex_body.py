#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

[joint_states]              -->     |                       |
[grounded_legs]             -->     |                       |       -->         [goal_joint_states]
[tf]                        -->     |                       |       -->         [tf] : [base_link_flat] -> [{leg_name}_tcs]
[pc_text_commands]          -->     |                       |
[teleop]                    -->     |      (hex_body)       |





                                              🡹🡻
                                    -------------------------
                                    |      Body object      |
                                    -------------------------
                                       |   - stores body dimensions (gets these from from parameter server)
                                       |
                                       |   - stores time-stamped goal joint angles                          |
                                       |   - stores time-stamped actual joint angles                        |
                                       |   - stores time-stamped goal foot positions in [base_link_flat]    |
                                       |   - stores time-stamped actual foot positions in [base_link_flat]  | are used to monitor errors in controlling
                                       position
                                       |
                                       |   - stores time-stamped grounded/not-grounded flags for each leg
                                       |
                                       |
                                       |   -----------------------------
                                       |---| Inverse kinematics solver | --> joint angles for a leg from goal foot position in [base_link] or [base_link_flat]
                                       |   -----------------------------
                                       |
                                       |   -----------------------------
                                       |---| Forward kinematics solver | --> foot position in [base_link] from angles:  (a) explicitly provided
                                       |   -----------------------------                                                (b) goal at a given time
                                       |                                                                                (c) actual at a given time
                                       |
                                       |   ------------------
                                       |---| Gait generator |  -->  generates foot positions to achieve movements requested by Twist messages in [odom]
                                       |   ------------------




*****************
 Gait generation:
                                                                              ------------------
       [twist]                   | --> Δx, Δy, Δz, Δrot_x, Δrot_y, Δrot_z --> | Gait generator |
       [current cycle frequency] |                                            ------------------


    ------------------
    | Gait generator |
    ------------------

    1) identify stance and swing legs:
         - leg stays in swing till it becomes grounded
         - grounded legs stay in stance until they are commanded to swing based on their restrictedness
           after being commanded to swing, legs must become not-grounded before they can touch down and return to stance




    2) process each stance leg:
         - check restrictedness based on the latest goal angles; if beyond threshold -- change to swing

         - get latest foot goal position in [base_link_flat] -- this position will be somewhat incorrect as tf for [base_link_flat]
           arrives with a delay

         - rotate this position by Δrot_x, Δrot_y, Δrot_z, translate by Δx, Δy, Δz -- in [base_link_flat] frame => (new_goal_position) @ current_time

         - adjust (new_goal_position) with x_correction, y_correction, z_correction

         - (new_goal_position)adjusted -> IK solver -> (new goal angles) @ current_time



    3) process each swing leg:
         - if leg has left the ground after being switched to swing and has already touched down -- switch to stance

         - update leg's Task Coordinate System using the TCS algorithm

         - determine (new_goal_position) using the swing algorithm

         - adjust (new_goal_position) with x_correction, y_correction, z_correction

         - (new_goal_position)adjusted -> IK solver -> (new goal angles) @ current_time



    4) get the latest available actual foot position, compare it to target foot position at that time => get x_correction, y_correction, z_correction







**************
TCS algorithm:








****************
Swing algorithm:








**************
Restrictedness:









"""



import copy
from math import atan2, degrees, radians, sqrt

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
from tf_conversions import transformations

from hex_utility import angle_ab, angle_ac, distance_between, get_point, third_side, transform_point_stamped

from hex_msg.msg import LegStates


class Body(object):
    class Leg(object):
        class Segment(object):
            def __init__(self, name, parent):
                self.name = name
                self.parent = parent

                self.length = rospy.get_param(name + "_length")

                self.current_angle = 0

                self.current_target_angle = 0  # target angle for the current cycle
                self.posture_target_angle = 0  # final target angle to reach required posture

                self.previous_angle = -1000

                self.min_angle = rospy.get_param(name + "_min_angle")
                self.max_angle = rospy.get_param(name + "_max_angle")

                self.angular_velocity = 0



        def __init__(self, name):
            self.name = name
            self.attachment_x = rospy.get_param(self.name + "_attachment_x")
            self.attachment_y = rospy.get_param(self.name + "_attachment_y")
            self.attachment_z = rospy.get_param(self.name + "_attachment_z")
            self.attachment_zrot = rospy.get_param(self.name + "_attachment_zrot")

            self.segments = dict()

            self.in_stance = False
            self.maintain_stance = False

            self.segments["coxa"] = Body.Leg.Segment("coxa", None)
            segment_names = [("femur", "coxa"), ("tibia", "femur"), ("tarsus", "tibia")]
            for segment_name in segment_names:
                self.segments[segment_name[0]] = Body.Leg.Segment(segment_name[0], self.segments[segment_name[1]])



        @property
        def attachment_xyz(self):
            return self.attachment_x, self.attachment_y, self.attachment_z



        def calc_joint_angles(self, target_point, best_effort, tf_buffer):

            """

            :type tf_buffer: tf2_ros.Buffer
            :type target_point: PointStamped
            """

            general_calculation = True

            try:
                point_in_leg_frame = transform_point_stamped(tf_buffer, self.name, target_point)
            except:
                return False

            angle_coxa = 90 - degrees(atan2(point_in_leg_frame.point.x, point_in_leg_frame.point.y))
            angle_femur = 0
            angle_tibia = 0
            femur_to_position_projection = 0

            coxa_segment = self.segments["coxa"]
            coxa_length = coxa_segment.length
            femur_segment = self.segments["femur"]
            femur_length = femur_segment.length
            tibia_segment = self.segments["tibia"]
            tibia_length = tibia_segment.length
            tarsus_segment = self.segments["tarsus"]
            tarsus_length = tarsus_segment.length

            if best_effort:
                tgt_tarsus = copy.deepcopy(point_in_leg_frame)
                tgt_tarsus.point.z += tarsus_segment.length

                femur_node = get_point(self.name + "_coxa", coxa_length, 0, 0)
                femur_node = transform_point_stamped(tf_buffer, self.name, femur_node)

                # check if we can actually reach target_position
                distance = distance_between(femur_node.point, tgt_tarsus.point)

                max_dist = femur_length + tibia_length
                min_dist = third_side(femur_length, tibia_length, 180 - tibia_segment.max_angle)
                dz = femur_node.point.z - tgt_tarsus.point.z

                # are we too far?
                if distance > max_dist:
                    # yep, too far
                    # calculate femur angle to target
                    femur_to_position_projection = sqrt(max_dist ** 2 - dz ** 2)
                    general_calculation = False
                elif distance < min_dist:
                    # we are too close
                    femur_to_position_projectionf = sqrt(min_dist ** 2 - dz ** 2)
                    general_calculation = False
                else:
                    # we are not too close and not too far
                    general_calculation = True

            if general_calculation:
                # works either when not best_effort or when best_effort and distance is ok
                coxa_to_position_projection = sqrt(point_in_leg_frame.point.x ** 2 + point_in_leg_frame.point.y ** 2)
                femur_to_position_projection = coxa_to_position_projection - coxa_length

            distance_to_tarsus_top = sqrt(
                femur_to_position_projection ** 2 + (point_in_leg_frame.point.z + tarsus_length) ** 2)

            if point_in_leg_frame.point.z + tarsus_length == 0:
                angle_down_to_tarsus_top = 0
            else:
                angle_down_to_tarsus_top = angle_ab((point_in_leg_frame.point.z + tarsus_length),
                                                    distance_to_tarsus_top,
                                                    femur_to_position_projection) - 90

            angle_femur = -1 * (angle_ab(femur_length, distance_to_tarsus_top, tibia_length) - angle_down_to_tarsus_top)
            angle_tibia = 180 - angle_ac(femur_length, distance_to_tarsus_top, tibia_length)
            angle_tarsus = 90 - (angle_femur + angle_tibia)

            if coxa_segment.min_angle > angle_coxa:
                if best_effort:
                    angle_coxa = coxa_segment.min_angle
                else:
                    raise Exception(self.name + " coxa angle too low")
            elif coxa_segment.max_angle < angle_coxa:
                if best_effort:
                    angle_coxa = coxa_segment.max_angle
                else:
                    raise Exception(self.name + " coxa angle too high")

            if femur_segment.min_angle > angle_femur:
                if best_effort:
                    angle_femur = femur_segment.min_angle
                else:
                    raise Exception(self.name + " femur angle too low")
            elif femur_segment.max_angle < angle_femur:
                if best_effort:
                    angle_femur = femur_segment.max_angle
                else:
                    raise Exception(self.name + " femur angle too high")

            if tibia_segment.min_angle > angle_tibia:
                if best_effort:
                    angle_tibia = tibia_segment.min_angle
                else:
                    raise Exception(self.name + " tibia angle too low")
            elif tibia_segment.max_angle < angle_tibia:
                if best_effort:
                    angle_tibia = tibia_segment.max_angle
                else:
                    raise Exception(self.name + " tibia angle too high")

            if tarsus_segment.min_angle > angle_tarsus:
                if best_effort:
                    angle_tarsus = tarsus_segment.min_angle
                else:
                    raise Exception(self.name + " tarsus angle too low")
            elif tarsus_segment.max_angle < angle_tarsus:
                if best_effort:
                    angle_tarsus = tarsus_segment.max_angle
                else:
                    raise Exception(self.name + " tarsus angle too high")

            angles = dict()
            angles["coxa"] = angle_coxa
            angles["femur"] = angle_femur
            angles["tibia"] = angle_tibia
            angles["tarsus"] = angle_tarsus

            return angles

    def __init__(self):
        self.legs = dict()

        leg_names = ["rf", "rm", "rr", "lf", "lm", "lr"]
        for leg_name in leg_names:
            self.legs[leg_name] = Body.Leg(leg_name)

        self.prev_assume_posture_call_time = None

        self.is_on = False



    def define_posture(self, posture_descriptor, tf_buffer=None):
        # posture_descriptor may be:
        # String -- to use pre-defined posture from config file: may be defined by exact joint angles or by foot positions in each foot's frame
        #
        #
        # list type #1 -- to set specific joint angles:
        # (leg_name, list((segment_name, angle, angular_velocity)))
        #
        # posture_descriptor = list()
        # for leg in body.legs.values():
        #     segment_postures = list()
        #     for segment in leg.segments.values():
        #         angle = _DEGREES_
        #         angular_velocity = _DEGREES_PER_SECOND_
        #         segment_postures.append((segment.name, angle, angular_velocity))
        #     posture_descriptor.append((leg.name, segment_postures))
        #
        #
        #
        #
        # list type #2 -- to move feet to specific positions:
        # (leg_name, target_point, optional, may be None-> list(segment_name, angular_velocity))
        #
        # if type #2 is used -- tf_buffer is necessary
        #
        # posture_descriptor may include items with list #1 and list #2 formats simultaneously
        #

        if isinstance(posture_descriptor, basestring):
            # use pre-defined posture from config file

            # see what type of posture definition that is
            definition_type = rospy.get_param("posture_" + posture_descriptor + "_type")

            angular_velocity = rospy.get_param("posture_" + posture_descriptor + "_angular_velocity")

            leg_names = ["rf", "rm", "rr", "lf", "lm", "lr"]
            segment_names = ["coxa", "femur", "tibia", "tarsus"]
            for leg_name in leg_names:
                leg = self.legs[leg_name]

                if definition_type == "foot_positions":
                    point_frame = rospy.get_param("posture_" + posture_descriptor + "_" + leg_name + "_frame")
                    point_x = rospy.get_param("posture_" + posture_descriptor + "_" + leg_name + "_x")
                    point_y = rospy.get_param("posture_" + posture_descriptor + "_" + leg_name + "_y")
                    point_z = rospy.get_param("posture_" + posture_descriptor + "_" + leg_name + "_z")

                    target_point = get_point(point_frame, point_x, point_y, point_z)
                    joint_angles = leg.calc_joint_angles(target_point, False, tf_buffer)


                for segment_name in segment_names:
                    segment = leg.segments[segment_name]

                    if definition_type == "angles":
                        segment.angular_velocity = angular_velocity
                        segment.posture_target_angle = rospy.get_param(
                            posture_descriptor + "_" + leg_name + "_" + segment_name + "_angle")

                    elif definition_type == "foot_positions":
                        segment.angular_velocity = angular_velocity

                        for segment_name in joint_angles.keys():
                            segment = leg.segments[segment_name]
                            segment.posture_target_angle = joint_angles[segment_name]




        elif isinstance(posture_descriptor, list):
            for leg_item in posture_descriptor:
                leg_name = leg_item[0]
                leg = self.legs[leg_name]

                if isinstance(leg_item[1], list):
                    # set specific joint angles
                    for segment_item in leg_item[1]:
                        segment_name = segment_item[0]
                        posture_target_angle = segment_item[1]
                        angular_velocity = segment_item[2]

                        segment = leg.segments[segment_name]
                        segment.posture_target_angle = posture_target_angle
                        segment.angular_velocity = angular_velocity


                elif isinstance(leg_item[1], PointStamped):
                    # calculate angles based on target point
                    target_point = leg_item[1]

                    joint_angles = leg.calc_joint_angles(target_point, False, tf_buffer)

                    if leg_item[2] != None:
                        for segment_item in leg_item[2]:
                            segment_name = segment_item[0]
                            angular_velocity = segment_item[1]

                            segment = leg.segments[segment_name]
                            segment.angular_velocity = angular_velocity

                    for segment_name in joint_angles.keys():
                        segment = leg.segments[segment_name]
                        segment.posture_target_angle = joint_angles[segment_name]



    def assume_posture(self):
        in_posture = True  # shows if all legs are at target positions

        if self.prev_assume_posture_call_time == None:
            self.prev_assume_posture_call_time = rospy.get_time() - 0.001

        cycle_duration = rospy.get_time() - self.prev_assume_posture_call_time

        leg_names = ["rf", "rm", "rr", "lf", "lm", "lr"]
        segment_names = ["coxa", "femur", "tibia", "tarsus"]
        for leg_name in leg_names:
            for segment_name in segment_names:

                leg = self.legs[leg_name]
                segment = leg.segments[segment_name]

                degs_per_cycle = segment.angular_velocity * cycle_duration

                dir = 1
                if segment.current_angle > segment.posture_target_angle:
                    dir = -1

                segment.current_target_angle = segment.current_target_angle + dir * min(degs_per_cycle,
                                                                                        abs(segment.current_target_angle - segment.posture_target_angle))

                if abs(segment.current_angle - segment.posture_target_angle) > 1:
                    in_posture = False





        self.prev_assume_posture_call_time = rospy.get_time()

        return in_posture


transform_broadcaster = None

body = Body()

tf_buffer = None


def got_text_command(text_command):
    global tf_buffer

    if text_command.data == "turn_on" and not body.is_on:
        # use actual angles as target angles when initializing
        for leg in body.legs.values():
            for segment in leg.segments.values():
                segment.posture_target_angle = segment.current_angle
                segment.current_target_angle = segment.current_angle

        body.is_on = True

        body.define_posture("initial", tf_buffer=tf_buffer)

    elif text_command.data == "turn_off":
        body.is_on = False

    elif text_command.data == "standup":
        body.define_posture("standup", tf_buffer=tf_buffer)

    elif text_command.data == "initial":
        body.define_posture("initial", tf_buffer=tf_buffer)



def got_joint_states(received_joint_states):
    # type: (JointState) -> received_joint_states
    # type: (JointState) -> current_joint_states


    global current_joint_states
    current_joint_states = received_joint_states

    pub_time = rospy.Time.now()

    t_batch = []

    js = 0
    for joint_name in current_joint_states.name:
        joint_position = current_joint_states.position[js]
        leg_name = joint_name[:2]
        joint_id = joint_name[3:7]

        if joint_id == "coxa":
            segment = body.legs[leg_name].segments["coxa"]
        elif joint_id == "femu":
            segment = body.legs[leg_name].segments["femur"]
        elif joint_id == "tibi":
            segment = body.legs[leg_name].segments["tibia"]
        elif joint_id == "tars":
            segment = body.legs[leg_name].segments["tarsus"]

        t = tf2_ros.TransformStamped()
        t.header.stamp = current_joint_states.header.stamp

        try:
            frame_name = leg_name + "_" + segment.parent.name
        except:
            frame_name = leg_name

        t.header.frame_id = frame_name
        t.child_frame_id = leg_name + "_" + segment.name
        segment.current_angle = degrees(joint_position)
        if segment.name == "coxa":
            quat = transformations.quaternion_from_euler(0, 0, joint_position)

        else:
            quat = transformations.quaternion_from_euler(0, joint_position, 0)

        try:
            x_offset = segment.parent.length
        except:
            x_offset = 0
        t.transform.translation.x = x_offset

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        t_batch.append(t)

        js += 1

    transform_broadcaster.sendTransform(t_batch)


def publish_target_joint_states(target_joint_state_pub):
    global body

    if not body.is_on:
        return

    target_joint_states = JointState()

    for leg in body.legs.values():
        leg_name = leg.name

        joint_name = leg_name + "_coxa_joint"
        target_joint_states.name.append(joint_name)
        target_joint_states.position.append(radians(leg.segments["coxa"].current_target_angle))

        joint_name = leg_name + "_femur_joint"
        target_joint_states.name.append(joint_name)
        target_joint_states.position.append(radians(leg.segments["femur"].current_target_angle))

        joint_name = leg_name + "_tibia_joint"
        target_joint_states.name.append(joint_name)
        target_joint_states.position.append(radians(leg.segments["tibia"].current_target_angle))

        joint_name = leg_name + "_tarsus_joint"
        target_joint_states.name.append(joint_name)
        target_joint_states.position.append(radians(leg.segments["tarsus"].current_target_angle))

    target_joint_states.header.stamp = rospy.Time.now()
    target_joint_state_pub.publish(target_joint_states)



def publish_static_tf():
    static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()

    t_batch = []
    for leg in body.legs.values():
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = leg.name
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = leg.attachment_xyz

        quat = transformations.quaternion_from_euler(0, 0, radians(leg.attachment_zrot))

        t.transform.rotation.x = quat[0]
        t.transform.rotation.z = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        t_batch.append(t)

    static_transform_broadcaster.sendTransform(t_batch)

def gotLegStates(received_leg_states):
    """

    :type received_leg_states: LegStates
    """
    leg_names = ["rf", "rm", "rr", "lf", "lm", "lr"]
    for i in range(0,6):
        leg_name = leg_names[i]
        leg = body.legs[leg_name]
        leg.in_stance = received_leg_states.stance_legs[i]


def run():
    global transform_broadcaster, body, tf_buffer

    rospy.init_node('hex_body', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    transform_broadcaster = tf2_ros.TransformBroadcaster()

    target_joint_state_pub = rospy.Publisher('target_joint_states', JointState, queue_size=3)

    rospy.Subscriber("joint_states", JointState, got_joint_states)

    rospy.Subscriber("pc_text_commands", String, got_text_command)


    rospy.Subscriber("leg_states", LegStates, gotLegStates)

    posture_control_rate_pub = rospy.Publisher('posture_control_rate', Float32, queue_size=1)

    publish_static_tf()



    actual_cycle_durations  = list([0,0,0,0,0])
    prev_time = rospy.Time.now()

    cycle_frequency = 100
    cycle_rate = rospy.Rate(cycle_frequency)
    while not rospy.is_shutdown():
        body.assume_posture()

        publish_target_joint_states(target_joint_state_pub)

        current_duration = (rospy.Time.now() - prev_time).to_sec()
        actual_cycle_durations.pop(0)
        actual_cycle_durations.append(current_duration)

        avg_cycle_duration = sum(actual_cycle_durations) / float(len(actual_cycle_durations))
        prev_time = rospy.Time.now()

        posture_control_rate_pub.publish(1/avg_cycle_duration)


        cycle_rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
