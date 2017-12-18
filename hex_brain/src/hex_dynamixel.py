#!/usr/bin/env python
# -*- coding: utf-8 -*-







#               [pc_text_commands]      -->     |                       |       -->         [joint_states]
#               [goal_joint_states]     -->     |   (hex_dynamixel)     |       -->         [grounded_legs]
#                                               |                       |
#                                                         🡹🡻
#                                               -------------------------
#                                               |      pypot robot      |
#                                               -------------------------
#                                                         🡹🡻
#                                               -------------------------
#                                               |      dynamixels       |
#                                               -------------------------


# [pc_text_commands]: turn_on, turn_off






from math import degrees, radians

import rospy
from hex_msg.msg import StampedBoolArray
from pypot import robot
from sensor_msgs.msg import JointState
from std_msgs.msg import String







hex_robot = None  # pypot robot


def robot_config():


    """
    prepares configuration to initialize pypot robot, reads from parameter server
    """



    leg_names = rospy.get_param("leg_names")
    segment_names = rospy.get_param("segment_names")

    config = dict()

    config["controllers"] = dict()
    controllers = config["controllers"]
    controllers["controller_1"] = dict()
    controller_1 = controllers["controller_1"]

    controller_1["sync_read"] = True
    controller_1["attached_motors"] = leg_names  # motor groups controlled by this controller
    controller_1["port"] = "auto"

    config["motorgroups"] = dict()
    motorgroups = config["motorgroups"]

    config["motors"] = dict()
    motors = config["motors"]

    for segment_name in segment_names:
        motorgroups[segment_name] = list()

    for leg_name in leg_names:
        motorgroups[leg_name] = list()
        for segment_name in segment_names:
            motorgroups[leg_name].append(leg_name + "_" + segment_name)
            motorgroups[segment_name].append(leg_name + "_" + segment_name)

            motors[leg_name + "_" + segment_name] = dict()
            m_desc = motors[leg_name + "_" + segment_name]

            direction = rospy.get_param(leg_name + "_" + segment_name + "_direction")
            if direction == 1:
                m_desc["orientation"] = "direct"
            elif direction == -1:
                m_desc["orientation"] = "indirect"

            m_desc["type"] = rospy.get_param(segment_name + "_servo_type")
            m_desc["id"] = rospy.get_param(leg_name + "_" + segment_name + "_servo_id")

            min_angle = -180
            max_angle = 180

            if m_desc["type"] == "AX-18":
                min_angle = -150
                max_angle = 180

            m_desc["angle_limit"] = list((min_angle, max_angle))

            m_desc["offset"] = rospy.get_param(leg_name + "_" + segment_name + "_angle_offset")

    return config

def got_text_command(text_command):
    if text_command.data == "turn_on":

        for motor in hex_robot.motors:
            motor.goal_position = motor.present_position  # use actual angles as target angles when initializing
            motor.torque_limit = 100
            motor.compliant = False


    elif text_command.data == "turn_off":
        for motor in hex_robot.motors:
            motor.torque_limit = 0
            motor.compliant = True
def got_goal_joint_states(received_goal_joint_states):
    """
    passes received goal positions to servos
    """

    # type: (JointState) -> received_goal_joint_states

    if hex_robot._controllers[0].io == None: # when in simulation mode without actual hardware
        return

    positions = dict()

    js = 0
    for joint_name in received_goal_joint_states.name:
        joint_position = degrees(received_goal_joint_states.position[js])
        leg_name = joint_name[:2]
        joint_id = joint_name[3:-6]

        positions[leg_name + "_" + joint_id] = joint_position

        js += 1

    time_to_reach_position = 0.3

    hex_robot.goto_position(positions, time_to_reach_position, "dummy")


def publish_grounded_legs(publisher, pub_time):


    """
    identifies grounded legs based on femur loads
    publishes results to [grounded_legs] topic
    """

    present_grounded_legs = StampedBoolArray()
    present_grounded_legs.stamp = pub_time

    loads = list()
    for m in hex_robot.femur:
        loads.append(m.present_load)

    # get the highest three loads
    sorted_loads = sorted(loads)
    avg = sum(sorted_loads[:3]) / 3

    # leg is considered to be grounded when its load is more than [grounded_threshold] of the average of three most loaded legs
    grounded_threshold = 0.6


    for i in range(0, 6):
        grounded = True
        if not hex_robot == None:
            if loads[i] > avg * grounded_threshold:
                grounded = False
        present_grounded_legs.values.append(grounded)

        # mark grounded legs with LEDs
        if hex_robot._controllers[0].io is not None:
            if grounded:
                hex_robot._controllers[0].io.switch_led_on([hex_robot.femur[i].id])
            else:
                hex_robot._controllers[0].io.switch_led_off([hex_robot.femur[i].id])

    publisher.publish(present_grounded_legs)
def publish_joint_states(publisher, pub_time):
    present_joint_states = JointState()

    for motor in hex_robot.motors:
        joint_name = motor.name + "_joint"

        if hex_robot._controllers[0].io == None:
            present_position = radians(motor.goal_position)
            present_load = 0
        else:
            present_position = radians(motor.present_position)
            present_load = motor.present_load

        present_joint_states.name.append(joint_name)
        present_joint_states.position.append(present_position)
        present_joint_states.effort.append(present_load)

    present_joint_states.header.stamp = pub_time
    publisher.publish(present_joint_states)


def run():
    global hex_robot

    rospy.init_node('hex_dynamixel', anonymous=True)

    rospy.Subscriber("pc_text_commands", String, got_text_command)
    rospy.Subscriber("goal_joint_states", JointState, got_goal_joint_states, queue_size=1)

    leg_state_pub = rospy.Publisher('grounded_legs', StampedBoolArray, queue_size=10)
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    try:
        hex_robot = robot.from_config(robot_config())
    except:
        hex_robot = robot.from_config(robot_config(), use_dummy_io=True)

    for motor in hex_robot.motors:
        motor.torque_limit = 0
        motor.compliant = True

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub_time = rospy.Time.now()

        publish_joint_states(joint_state_pub, pub_time)
        publish_grounded_legs(leg_state_pub, pub_time)

        r.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
