import math
import rospy
from flask import Blueprint
from duckietown_msgs.msg import WheelsCmdStamped
from dt_robot_rest_api.utils import response_ok


car_bp = Blueprint('car', __name__)


def _wheels_cmd_cb(msg):
    global _car_is_moving
    _car_is_moving = math.fabs(msg.vel_left) + math.fabs(msg.vel_right) > 0


_car_is_moving = False
_pub = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, _wheels_cmd_cb, queue_size=1)


@car_bp.route('/car/status')
def _carstatus():
    # return current API car_car
    return response_ok({'engaged': _car_is_moving})
