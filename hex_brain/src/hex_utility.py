import rospy
from math import sin, cos, radians, degrees, acos, log10, exp, sqrt

from geometry_msgs.msg import PointStamped

import tf2_geometry_msgs


def cosd(degs):
    return cos(radians(degs))


def sind(degs):
    return sin(radians(degs))


def angle_ab(a_length, b_length, c_length):
    val = (a_length ** 2 + b_length ** 2 - c_length ** 2) / (2 * a_length * b_length)
    val = max(-0.999999999999999, min(0.999999999999999, val))
    try:
        return degrees(acos(val))
    except:
        print "angle_ab error, val = " + str(val)


def angle_ac(a_length, b_length, c_length):
    val = (a_length ** 2 + c_length ** 2 - b_length ** 2) / (2 * a_length * c_length)
    val = max(-0.999999999999999, min(0.999999999999999, val))
    try:
        return degrees(acos(val))
    except:
        print "angle_ac error, val = " + str(val)


def angle_bc(a_length, b_length, c_length):
    val = (b_length ** 2 + c_length ** 2 - a_length ** 2) / (2 * b_length * c_length)
    val = max(-0.999999999999999, min(0.999999999999999, val))
    try:
        return degrees(acos(val))
    except:
        print "angle_bc error, val = " + str(val)


def exp_r(parameter, lower_limit, upper_limit, transition_zone):
    # Restrictedness increases exponentially from 0.1 to 1 over transition_zone

    epsilon = -1 * log10(0.1) / transition_zone

    if parameter > (lower_limit + upper_limit) / 2:
        return exp(epsilon * (parameter - upper_limit))
    else:
        return exp(-1 * epsilon * (parameter - lower_limit))


def third_side(side1, side2, angle):
    return sqrt(side1 ** 2 + side2 ** 2 - 2 * side1 * side2 * cosd(angle))


def closest_intersection(circle_center, radius, line_start, line_end):
    """

    :rtype : (x,y)
    :type line_start: (x,y)
    :type line_end: (x,y)
    :type radius: float
    :type circle_center: (x,y)
    """
    dx = line_end[0] - line_start[0]
    dy = line_end[1] - line_start[1]

    a = dx * dx + dy * dy
    b = 2 * (dx * (line_start[0] - circle_center[0]) + dy * (line_start[1] - circle_center[1]))
    c = (line_start[0] - circle_center[0]) * (line_start[0] - circle_center[0]) + (line_start[1] - circle_center[1]) * (
        line_start[1] - circle_center[1]) - radius * radius

    det = b * b - 4 * a * c
    if (a <= 0.0000001) or (det < 0):
        # No real solutions.
        return None
    elif det == 0:
        # One solution.
        t = -b / (2 * a)
        pt1 = line_start[0] + t * dx, line_start[1] + t * dy

        # is this point between line start and line end?
        if min(line_start[0], line_end[0]) <= pt1[0] <= max(line_start[0], line_end[0]) \
                and min(line_start[1], line_end[1]) <= pt1[1] <= max(line_start[1], line_end[1]):
            return pt1
        else:
            return None

    else:
        # Two solutions.
        t = (-b + sqrt(det)) / (2 * a)
        pt1 = line_start[0] + t * dx, line_start[1] + t * dy
        t = (-b - sqrt(det)) / (2 * a)
        pt2 = line_start[0] + t * dx, line_start[1] + t * dy

        pt1_wrong = False
        pt2_wrong = False

    # is this point1 between line start and line end?
    if min(line_start[0], line_end[0]) <= pt1[0] <= max(line_start[0], line_end[0]) \
            and min(line_start[1], line_end[1]) <= pt1[1] <= max(line_start[1], line_end[1]):
        # compare distances
        pass
    else:
        pt1_wrong = True

    # is this point2 between line start and line end?
    if min(line_start[0], line_end[0]) <= pt2[0] <= max(line_start[0], line_end[0]) \
            and min(line_start[1], line_end[1]) <= pt2[1] <= max(line_start[1], line_end[1]):
        # compare distances
        pass
    else:
        pt2_wrong = True

    if pt1_wrong and pt2_wrong:
        return None

    # which one is closer to line start?

    d1 = ((line_start[0] - pt1[0]) ** 2 + (line_start[1] - pt1[1]) ** 2) ** 0.5
    d2 = ((line_start[0] - pt2[0]) ** 2 + (line_start[1] - pt2[1]) ** 2) ** 0.5

    if d1 < d2:
        return pt1
    else:
        return pt2


def get_point(frame_id, x, y, z, time=rospy.Time(0)):
    """

    :rtype: PointStamped
    """
    pt = PointStamped()

    pt.header.stamp = time
    pt.header.frame_id = frame_id
    pt.point.x, pt.point.y, pt.point.z = x, y, z

    return pt


def distance_between(pt1, pt2):
    dx = pt1.x - pt2.x
    dy = pt1.y - pt2.y
    dz = pt1.z - pt2.z

    return sqrt(dx ** 2 + dy ** 2 + dz ** 2)


def transform_point_stamped(transform_buffer, target_frame_name, point_stamped):
    """

    :type point_stamped: PointStamped
    :type transform_buffer: tf2_ros.Buffer
    
    """

    transform = transform_buffer.lookup_transform(target_frame_name,
                                                  point_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1))

    return tf2_geometry_msgs.do_transform_point(point_stamped, transform)
