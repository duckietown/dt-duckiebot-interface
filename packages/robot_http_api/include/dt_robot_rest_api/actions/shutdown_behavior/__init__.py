import rospy
import traceback
from flask import Blueprint
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from dt_robot_rest_api.utils import response_ok, response_error


shutdown_behavior_bp = Blueprint("shutdown_behavior", __name__)

# this service will be called, when the API is triggered
_srv_proxy = rospy.ServiceProxy("~shutdown_behavior", Trigger)
_trigger = TriggerRequest()


@shutdown_behavior_bp.route("/shutdown_behavior")
def _show_shutdown_behavior():
    try:
        resp: TriggerResponse = _srv_proxy(_trigger)
        if resp.success:
            return response_ok({})
        else:
            return response_error(message=resp.message)
    except Exception as e:
        traceback.print_exc()
        return response_error(message=f"Failed to run shutdown behaviors. Reason: {e}")
