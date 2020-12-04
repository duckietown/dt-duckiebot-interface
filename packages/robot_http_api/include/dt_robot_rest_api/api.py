import logging

from flask import Flask
from flask_cors import CORS

from dt_robot_utils import get_robot_type


class RobotRestAPI(Flask):

    def __init__(self, debug=False):
        # create Flask app
        super(RobotRestAPI, self).__init__(__name__)
        self.http_port = None
        # apply CORS settings
        CORS(self)
        # configure logging
        self._logger = logging.getLogger('werkzeug')
        self._logger.setLevel(logging.DEBUG if debug else logging.WARNING)

    def setup(self):
        """
        Make sure that this function is always called after rospy.init_node, some actions
        spin up publishers/subscribers.
        """
        if self.http_port is not None:
            return
        # ---
        from .robot import blueprints, HTTP_PORT
        for blueprint in blueprints:
            self.register_blueprint(blueprint)
        self.http_port = HTTP_PORT

    def run(self, *args, **kwargs):
        self.setup()
        # ---
        if self.http_port is None:
            self._logger.error(
                "No HTTP_PORT configured yet. "
                "The current robot type might not be supported yet. Exiting.")
            return
        self._logger.info(f'Running HTTP server for robot type `{get_robot_type().name}` '
                          f'on TCP port {self.http_port}.')
        super(RobotRestAPI, self).run(*args, port=self.http_port, **kwargs)
