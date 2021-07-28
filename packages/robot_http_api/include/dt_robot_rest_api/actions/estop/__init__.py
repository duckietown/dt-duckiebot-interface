import rospy
from flask import Blueprint
from duckietown_msgs.msg import BoolStamped
from dt_robot_rest_api.utils import response_ok


estop_bp = Blueprint('estop', __name__)


def _estop_cb(msg):
    global _estop_value
    _estop_value = msg.data


_estop_value = False
_pub = rospy.Publisher("~estop", BoolStamped, queue_size=1)
_sub = rospy.Subscriber("~estop", BoolStamped, _estop_cb, queue_size=1)


def _estop(value: bool):
    _pub.publish(BoolStamped(data=value))
    return response_ok({})


@estop_bp.route('/estop/on')
def _estop_on():
    # return current API car_estop
    return _estop(True)


@estop_bp.route('/estop/status')
def _estop_status():
    # return current API car_estop
    return response_ok({'engaged': _estop_value})


@estop_bp.route('/estop/off')
def _estop_off():
    # return current API car_estop
    return _estop(False)
