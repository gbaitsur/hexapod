#!/usr/bin/env python

# gets body parameters from robot_description on parameter server
# subscribes to [joint_states] and updates legs to store latest joint angles
# publishes tf based on data in legs


# calculates joint angles to reach target foot position



import rospy
from sensor_msgs.msg import JointState
import tf2_ros

from math import radians, degrees, atan2, sqrt
from geometry_msgs.msg import PointStamped, TransformStamped
from hex_utility import angle_ab, angle_ac, third_side, get_point, distance_between, transform_point_stamped

from tf_conversions import transformations

import copy





class Body(object):
    def __init__(self):
        self.clearance = rospy.get_param("neutral_clearance")



class Leg(object):
    class Segment(object):
        def __init__(self, name, parent):
            self.name = name
            self.parent = parent

            self.length = rospy.get_param(name + "_length")
            self.current_angle = 0
            # self.current_quaternion = tf2_ros.transformations.quaternion_from_euler(0, self.current_angle, 0)

            self.previous_angle = -1000

            self.min_angle = rospy.get_param(name + "_min_angle")
            self.max_angle = rospy.get_param(name + "_max_angle")

    def __init__(self, name):
        self.name = name
        self.attachment_x = rospy.get_param(self.name + "_attachment_x")
        self.attachment_y = rospy.get_param(self.name + "_attachment_y")
        self.attachment_z = rospy.get_param(self.name + "_attachment_z")
        self.attachment_zrot = rospy.get_param(self.name + "_attachment_zrot")
        # self.attachment_quaternion = tf.transformations.quaternion_from_euler(0, 0, radians(self.attachment_zrot))

        self.segments = dict()

        self.segments["coxa"] = Leg.Segment("coxa", None)
        segment_names = [("femur", "coxa"), ("tibia", "femur"), ("tarsus", "tibia")]
        for segment_name in segment_names:
            self.segments[segment_name[0]] = Leg.Segment(segment_name[0], self.segments[segment_name[1]])

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

        return angle_coxa, angle_femur, angle_tibia, angle_tarsus


body= Body()

legs = dict()
transform_broadcaster = None


def got_joint_states(received_joint_states):
    # type: (JointState) -> received_joint_states
    # type: (JointState) -> current_joint_states


    global current_joint_states, legs
    current_joint_states = received_joint_states

    pub_time = rospy.Time.now()

    t_batch = []

    js = 0
    for joint_name in current_joint_states.name:
        joint_position = current_joint_states.position[js]
        leg_name = joint_name[:2]
        joint_id = joint_name[3:7]

        if joint_id == "coxa":
            segment = legs[leg_name].segments["coxa"]
        elif joint_id == "femu":
            segment = legs[leg_name].segments["femur"]
        elif joint_id == "tibi":
            segment = legs[leg_name].segments["tibia"]
        elif joint_id == "tars":
            segment = legs[leg_name].segments["tarsus"]

        t = tf2_ros.TransformStamped()
        t.header.stamp = rospy.Time.now()

        try:
            frame_name = leg_name + "_" + segment.parent.name
        except:
            frame_name = leg_name

        t.header.frame_id = frame_name
        t.child_frame_id = leg_name + "_" + segment.name
        segment.current_angle = joint_position
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

def fill_legs():
    global legs
    leg_names = ["rf", "rm", "rr", "lf", "lm", "lr"]
    for leg_name in leg_names:
        legs[leg_name] = Leg(leg_name)


def publish_target_joint_states(target_joint_state_pub, leg_name, coxa_angle, femur_angle, tibia_angle, tarsus_angle):
    target_joint_states = JointState()
    joint_name = leg_name + "_coxa_joint"
    target_joint_states.name.append(joint_name)
    target_joint_states.position.append(radians(coxa_angle))

    joint_name = leg_name + "_femur_joint"
    target_joint_states.name.append(joint_name)
    target_joint_states.position.append(radians(femur_angle))

    joint_name = leg_name + "_tibia_joint"
    target_joint_states.name.append(joint_name)
    target_joint_states.position.append(radians(tibia_angle))

    joint_name = leg_name + "_tarsus_joint"
    target_joint_states.name.append(joint_name)
    target_joint_states.position.append(radians(tarsus_angle))

    target_joint_states.header.stamp = rospy.Time.now()
    target_joint_state_pub.publish(target_joint_states)




def set_legs_neutral(tf_buffer, legs, target_joint_state_pub):
    global body


    for leg in legs:

        test_point = get_point(leg.name, rospy.get_param("neutral_foot_distance"), 0, -(body.clearance + leg.attachment_z))

        joint_angles = leg.calc_joint_angles(test_point, False, tf_buffer)
        if joint_angles != False:
            publish_target_joint_states(target_joint_state_pub, leg.name, *joint_angles)


def run():
    global legs, transform_broadcaster, body

    fill_legs()

    # test

    rospy.init_node('hex_body', anonymous=True)

    current_joint_states = JointState()

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    transform_broadcaster = tf2_ros.TransformBroadcaster()

    static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()
    t_batch = []
    for leg in legs.values():
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = leg.name
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = leg.attachment_xyz

        quat = transformations.quaternion_from_euler(0, 0,radians(leg.attachment_zrot))

        t.transform.rotation.x = quat[0]
        t.transform.rotation.z = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        t_batch.append(t)

    static_transform_broadcaster.sendTransform(t_batch)


    target_joint_state_pub = rospy.Publisher('target_joint_states', JointState, queue_size=10)

    rospy.Subscriber("joint_states", JointState, got_joint_states)




    mover = -0.001


    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        set_legs_neutral(tf_buffer, legs.values(), target_joint_state_pub)


        if body.clearance > 0.075 or body.clearance < 0.001:
            mover*=-1

        body.clearance += mover


        r.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
