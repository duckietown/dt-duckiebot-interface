import os
import glob
import socket
from flask import Blueprint, send_file
from dt_robot_rest_api.utils import response_ok, response_not_found


logs_bp = Blueprint("logs", __name__)

_ROBOT_HOSTNAME = socket.gethostname()
_ROS_LOGS_DIR = "/tmp/log/latest"


# @logs_bp.route("/logs/search/<nodename>")
# def search_ros_node_logs(nodename):
#     file_pattern = _ROS_LOGS_DIR + '/' + f"{_ROBOT_HOSTNAME}-{nodename}-*.log"
#     log_files = glob.glob(file_pattern)
#     # for log_f in log_files:
#     #     print("Matching log:", log_f)

#     if len(log_files) > 0:
#         f_name = log_files[0]
#     else:
#         f_name = "Not Found!"

#     ret_data = {
#         "n_logs_found": len(log_files),
#         "first_log_file_name": f_name,
#     }

#     return response_ok(
#         {
#             "result_data": ret_data,
#         },
#     )


# TODO: would there be multiple log files for the same node?
@logs_bp.route("/logs/download/<nodename>")
def download_ros_node_logs(nodename):
    file_pattern = _ROS_LOGS_DIR + '/' + f"{_ROBOT_HOSTNAME}-{nodename}-*.log"
    log_files = glob.glob(file_pattern)

    if len(log_files) > 0:
        return send_file(log_files[0])
    else:
        return response_not_found()