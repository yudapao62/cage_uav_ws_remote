"""Small helpers that keep marker construction consistent across nodes."""
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def point(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


def color(red, green, blue, alpha=1.0):
    return ColorRGBA(r=red, g=green, b=blue, a=alpha)


def marker(frame_id, namespace, marker_id, marker_type, stamp):
    result = Marker()
    result.header.frame_id = frame_id
    result.header.stamp = stamp
    result.ns = namespace
    result.id = marker_id
    result.type = marker_type
    result.action = Marker.ADD
    result.pose.orientation.w = 1.0
    result.lifetime.secs = 0
    return result


def text_marker(frame_id, namespace, marker_id, stamp, position, text, height, rgba):
    result = marker(frame_id, namespace, marker_id, Marker.TEXT_VIEW_FACING, stamp)
    result.pose.position = position
    result.scale.z = height
    result.color = rgba
    result.text = text
    return result
