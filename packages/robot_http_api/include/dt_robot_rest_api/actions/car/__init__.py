import math
import time

import rospy
from flask import Blueprint
from duckietown_msgs.msg import WheelsCmdStamped
from dt_robot_rest_api.utils import response_ok


car_bp = Blueprint('car', __name__)


def _wheels_cmd_cb(msg):
    global _last_moving_msg_time
    _car_is_moving = math.fabs(msg.vel_left) + math.fabs(msg.vel_right) > 0
    if _car_is_moving:
        _last_moving_msg_time = time.time()


_last_moving_msg_time = 0
_last_moving_msg_time_thr = 1
_pub = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, _wheels_cmd_cb, queue_size=1)


@car_bp.route('/car/status')
def _carstatus():
    # return current API car_car
    return response_ok({
        # this means that the robot is considered NOT moving after
        # at least `_last_moving_msg_time_thr` sec with no non-zero
        # velocity messages received
        'engaged': (time.time() - _last_moving_msg_time) < _last_moving_msg_time_thr
    })
