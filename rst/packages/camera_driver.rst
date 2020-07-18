
camera_driver package
*********************


Contents
^^^^^^^^

* `camera_driver package`_

    * `CameraNode`_

The *camera_driver* package handles everything camera-related on a
Duckiebot. It has a single node *CameraNode* that acquires images
using `Picamera <https://picamera.readthedocs.io/>`_ and then
publishes them. The same node is also responsible for publishing
``CameraInfo`` messages.


CameraNode
==========
