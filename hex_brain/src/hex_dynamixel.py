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



class JointAngles(object):
    def __init__(self, leg_name, coxa_angle=0, femur_angle=0, tibia_angle=0, tarsus_angle=0):
        self.leg_name = leg_name
        self.coxa_angle = coxa_angle
        self.femur_angle = femur_angle
        self.tibia_angle = tibia_angle
        self.tarsus_angle = tarsus_angle

target_joint_angles = dict()
current_joint_angles = dict()





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
        current_joint_angles[leg_name] = JointAngles(leg_name)


def send_targets_to_dynamixels():
    return

def read_dynamixels():
    for target_angles in target_joint_angles.values():
        current_joint_angles[target_angles.leg_name] = target_angles


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
    leg_names = ["rf", "rm", "rr", "lf", "lm", "lr"]

    current_joint_states = JointState()

    for leg_name in leg_names:
        joint_angles_list = current_joint_angles[leg_name]
        """:type : JointAngles"""

        joint_angles = [("coxa", joint_angles_list.coxa_angle), ("femur", joint_angles_list.femur_angle), ("tibia", joint_angles_list.tibia_angle), ("tarsus", joint_angles_list.tarsus_angle)]

        for joint_angle in joint_angles:
            joint_name = leg_name + "_" + joint_angle[0] + "_joint"
            current_joint_states.name.append(joint_name)
            current_joint_states.position.append(joint_angle[1])

    current_joint_states.header.stamp = rospy.Time.now()
    publisher.publish(current_joint_states)



def run():
    femur_angle = 30
    tibia_angle = 20
    tarsus_angle = 20
    delta = 0.1

    fill_joint_angles()


    rospy.init_node('hex_dynamixel', anonymous=True)

    rospy.Subscriber("target_joint_states", JointState, got_target_joint_states)

    leg_state_pub = rospy.Publisher('leg_states', LegStates, queue_size=10)
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        read_dynamixels()

        publish_leg_states(leg_state_pub)

        publish_joint_states(joint_state_pub)

        r.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
