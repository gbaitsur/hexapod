#!/usr/bin/env python

# reads dynamixels
#
# publishes [leg_states]
# publishes [joint_states]

# subscribes to [target_joint_states], passes targets to dynamixels




import rospy
from hex_msg.msg import LegStates
from sensor_msgs.msg import JointState



from math import radians

import pypot.dynamixel


dxl_io = pypot.dynamixel.DxlIO






class JointAngles(object):
    def __init__(self, leg_name, coxa_angle=0, femur_angle=0, tibia_angle=0, tarsus_angle=0):
        self.leg_name = leg_name
        self.coxa_angle = coxa_angle
        self.femur_angle = femur_angle
        self.tibia_angle = tibia_angle
        self.tarsus_angle = tarsus_angle

target_joint_angles = dict()

servo_ids = list()
servo_offsets = list()
servo_directions = list()
joint_names = list()
current_joint_angles = list()


def prepare_servo_lists():
    global servo_ids, joint_names


    leg_names = ("rf", "rm", "rr", "lf", "lm", "lr")
    segment_names = ("coxa", "femur", "tibia", "tarsus")
    for leg_name in leg_names:
        for segment_name in segment_names:
            servo_ids.append(rospy.get_param(leg_name + "_" + segment_name + "_servo_id"))
            joint_names.append (leg_name + "_" + segment_name)
            servo_offsets.append(rospy.get_param(leg_name + "_" + segment_name + "_angle_offset"))
            servo_directions.append(rospy.get_param(leg_name + "_" + segment_name + "_direction"))


def got_target_joint_states(received_target_joint_states):
    # type: (JointState) -> received_target_joint_states
    global target_joint_angles

    js = 0
    for joint_name in received_target_joint_states.name:
        joint_position = received_target_joint_states.position[js]
        leg_name = joint_name[:2]
        joint_id = joint_name[3:7]

        if joint_id == "coxa":
            target_joint_angles[leg_name].coxa_angle = joint_position
        elif joint_id == "femu":
            target_joint_angles[leg_name].femur_angle = joint_position
        elif joint_id == "tibi":
            target_joint_angles[leg_name].tibia_angle = joint_position
        elif joint_id == "tars":
            target_joint_angles[leg_name].tarsus_angle = joint_position

        js += 1



def fill_joint_angles():
    leg_names = ["rf", "rm", "rr", "lf", "lm", "lr"]
    for leg_name in leg_names:
        target_joint_angles[leg_name] = JointAngles(leg_name)


def send_targets_to_dynamixels():
    return

def read_dynamixels():

    global dxl_io, servo_ids, current_joint_angles

    try:
        current_joint_angles = dxl_io.get_present_position(servo_ids)
    except Exception, e:
        rospy.logerr(e.message)
        return False

    return True


def publish_leg_states(publisher):
    current_leg_states = LegStates()
    current_leg_states.stamp = rospy.Time.now()
    current_leg_states.rf_is_landed = True
    current_leg_states.rm_is_landed = True
    current_leg_states.rr_is_landed = True
    current_leg_states.lf_is_landed = True
    current_leg_states.lm_is_landed = True
    current_leg_states.lr_is_landed = True
    publisher.publish(current_leg_states)


def publish_joint_states(publisher):
    global joint_names, current_joint_angles, servo_offsets, servo_directions

    current_joint_states = JointState()


    for i in range(0,24):
        joint_name = joint_names[i] + "_joint"
        joint_angle = radians(servo_directions[i] * current_joint_angles[i] + servo_offsets[i])

        # rospy.loginfo(joint_name + " " + str(servo_directions[i] * current_joint_angles[i] + servo_offsets[i]))

        current_joint_states.name.append(joint_name)
        current_joint_states.position.append(joint_angle)

    current_joint_states.header.stamp = rospy.Time.now()
    publisher.publish(current_joint_states)



def run():
    global dxl_io, servo_list


    fill_joint_angles()

    prepare_servo_lists()


    rospy.init_node('hex_dynamixel', anonymous=True)

    rospy.Subscriber("target_joint_states", JointState, got_target_joint_states)

    leg_state_pub = rospy.Publisher('leg_states', LegStates, queue_size=10)
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    ports = pypot.dynamixel.get_available_ports()
    rospy.loginfo( "Dynamixel found on: " + str(ports))

    dxl_io = pypot.dynamixel.DxlIO(ports[0], use_sync_read=True)

    servo_list = dxl_io.scan()


    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        if read_dynamixels():
            publish_joint_states(joint_state_pub)



        publish_leg_states(leg_state_pub)



        r.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
