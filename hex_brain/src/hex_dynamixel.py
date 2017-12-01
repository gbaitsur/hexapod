#!/usr/bin/env python

# reads dynamixels
#
# publishes [leg_states]
# publishes [joint_states]

# subscribes to [target_joint_states], passes targets to dynamixels




from math import degrees, radians

import rospy
from hex_msg.msg import LegStates
from pypot import robot
from sensor_msgs.msg import JointState
from std_msgs.msg import String

hex_robot = None  # pypot robot

is_on = False


class JointAngles(object):
    def __init__(self, leg_name, coxa_angle=0, femur_angle=0, tibia_angle=0, tarsus_angle=0):
        self.leg_name = leg_name
        self.coxa_angle = coxa_angle
        self.femur_angle = femur_angle
        self.tibia_angle = tibia_angle
        self.tarsus_angle = tarsus_angle


# servo_ids = list()
# servo_offsets = list()
# servo_directions = list()
# joint_names = list()
# current_joint_angles = list()
#
# stance_legs = list()


# target_joint_angles = list()
# servo_sequence_dict = dict()








def robot_config():
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

            # min_angle = rospy.get_param(segment_name + "_min_angle")
            # max_angle = rospy.get_param(segment_name + "_max_angle")

            min_angle = -180
            max_angle = 180

            if m_desc["type"] == "AX-18":
                min_angle = -150
                max_angle = 180

            m_desc["angle_limit"] = list((min_angle, max_angle))

            m_desc["offset"] = rospy.get_param(leg_name + "_" + segment_name + "_angle_offset")

    return config

def got_target_joint_states(received_target_joint_states):
    # type: (JointState) -> received_target_joint_states

    if hex_robot == None:
        return

    positions = dict()

    js = 0
    for joint_name in received_target_joint_states.name:
        joint_position = degrees(received_target_joint_states.position[js])
        leg_name = joint_name[:2]
        joint_id = joint_name[3:-6]

        positions[leg_name + "_" + joint_id] = joint_position

        js += 1

    hex_robot.goto_position(positions, 0.3, "dummy")


def publish_leg_states(publisher):
    current_leg_states = LegStates()
    current_leg_states.stamp = rospy.Time.now()

    loads = list()
    for m in hex_robot.femur:
        loads.append(m.present_load)

    # get the highest three loads
    sorted_loads = sorted(loads)
    avg = sum(sorted_loads[:3]) / 3

    # leg is considered to be in stance when its load is within 90% of the average of three most loaded legs

    for i in range(0, 6):
        in_stance = True
        if not hex_robot == None:
            if loads[i] > avg * 0.6:
                in_stance = False
        current_leg_states.stance_legs.append(in_stance)

        if hex_robot._controllers[0].io is not None:
            if in_stance:
                hex_robot._controllers[0].io.switch_led_on([hex_robot.femur[i].id])
            else:
                hex_robot._controllers[0].io.switch_led_off([hex_robot.femur[i].id])

    publisher.publish(current_leg_states)


def publish_joint_states(publisher):
    current_joint_states = JointState()

    for motor in hex_robot.motors:
        joint_name = motor.name + "_joint"

        if hex_robot._controllers[0].io == None:
            joint_angle = radians(motor.goal_position)
        else:
            joint_angle = radians(motor.present_position)

        current_joint_states.name.append(joint_name)
        current_joint_states.position.append(joint_angle)

    current_joint_states.header.stamp = rospy.Time.now()
    publisher.publish(current_joint_states)


def got_text_command(text_command):
    global is_on

    if text_command.data == "turn_on" and not is_on:
        # use actual angles as target angles when initializing
        for motor in hex_robot.motors:
            motor.torque_limit = 100
            motor.compliant = False

        is_on = True


    elif text_command.data == "turn_off":
        for motor in hex_robot.motors:
            motor.torque_limit = 0
            motor.compliant = True

        is_on = False


def run():
    global hex_robot

    rospy.init_node('hex_dynamixel', anonymous=True)

    rospy.Subscriber("target_joint_states", JointState, got_target_joint_states, queue_size=1)

    leg_state_pub = rospy.Publisher('leg_states', LegStates, queue_size=10)
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rospy.Subscriber("pc_text_commands", String, got_text_command)

    try:
        hex_robot = robot.from_config(robot_config())
    except:
        hex_robot = robot.from_config(robot_config(), use_dummy_io=True)

    for motor in hex_robot.motors:
        motor.torque_limit = 0
        motor.compliant = True

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        publish_joint_states(joint_state_pub)
        publish_leg_states(leg_state_pub)

        r.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
