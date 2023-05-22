import glob
import os
import socket
import traceback
from datetime import datetime

from flask import Blueprint, send_file, send_from_directory
from dt_robot_rest_api.utils import response_ok, response_not_found


logs_bp = Blueprint("logs", __name__)

_ROBOT_HOSTNAME = socket.gethostname()
_ROS_LOGS_DIR = "/tmp/log/latest"


@logs_bp.route("/logs/list")
def search_ros_node_logs():
    file_pattern = os.path.join(_ROS_LOGS_DIR, f"{_ROBOT_HOSTNAME}-*.log")
    log_files = glob.glob(file_pattern)
    # for log_f in log_files:
    #     print("Matching log:", log_f)
    log_files = [os.path.basename(f_path) for f_path in log_files]

    ret_data = {
        "n_logs_found": len(log_files),
        "lst_log_file_names": log_files,
        "search_pattern": file_pattern
    }

    return response_ok(
        {
            "result_data": ret_data,
        },
    )


@logs_bp.route("/logs/download/<filename>")
def download_ros_node_logs(filename):
    file_path = os.path.join(_ROS_LOGS_DIR, filename)
    if os.path.exists(file_path):
        f_base, f_ext = os.path.splitext(filename)
        # timestamp
        current_time = datetime.now().replace(microsecond=0)
        formatted_time = current_time.isoformat().replace(':', '').replace('-', '').replace('.', '')
        # new filename when downloading
        new_file_name = f"{f_base}-TS{formatted_time}{f_ext}"
        # print("Sending file as attachment", new_file_name)
        try:
            return send_file(
                file_path,
                as_attachment=True,
                download_name=new_file_name,
            )
        except Exception as e:
            traceback.print_exc()
            return response_not_found()
    else:
        return response_not_found()
