import glob
import os
import traceback
from datetime import datetime

from flask import Blueprint, send_file
from dt_robot_rest_api.utils import (
    response_ok, response_not_found, response_error,
)
from dt_device_utils import get_device_hostname


logs_bp = Blueprint("logs", __name__)

_ROBOT_HOSTNAME = get_device_hostname()
_ROS_LOGS_DIR = "/tmp/log/latest"


@logs_bp.route("/logs/list")
def search_ros_node_logs():
    """Returns a mapping between ros nodes and their log file paths"""
    try:
        file_pattern = os.path.join(_ROS_LOGS_DIR, f"{_ROBOT_HOSTNAME}-*.log")
        log_files = glob.glob(file_pattern)

        node_names = [os.path.basename(f_path).split('-')[1] for f_path in log_files]
        node_to_log = dict(zip(node_names, log_files))

        ret_data = {
            "count": len(log_files),
            "logs": node_to_log,
            "pattern": file_pattern
        }

        return response_ok(ret_data)
    except Exception as e:
        traceback.print_exc()
        return response_error()


@logs_bp.route("/logs/download/<nodename>")
def download_ros_node_logs(nodename):
    file_pattern = os.path.join(_ROS_LOGS_DIR, f"{_ROBOT_HOSTNAME}-{nodename}-*.log")
    log_files = glob.glob(file_pattern)

    # lex order's last log file, in case there are rolling/truncated logs
    file_path = sorted(log_files, reverse=True)[0]

    if os.path.exists(file_path):
        # prepare to rename file when downloading
        f_base, f_ext = os.path.splitext(os.path.basename(file_path))
        # timestamp
        current_time = datetime.now().replace(microsecond=0)
        formatted_time = current_time.isoformat().replace(':', '').replace('-', '').replace('.', '')
        # new filename when downloading
        new_file_name = f"{f_base}-TS{formatted_time}{f_ext}"
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
