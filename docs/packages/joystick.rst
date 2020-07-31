joystick package
================

The joystick package is a wrapper around the `ROS joy <https://wiki.ros.org/joy>`_ node.

It accepts the following parameters:

Configuration:

   - dev (:obj:`str`): Linux joystick device from which to read joystick events, default is `/dev/input/js0`
     
   - deadzone (:obj:`float`): Amount by which the joystick has to move before it is considered to be off-center.
     This parameter is specified relative to an axis normalized between -1 and 1. Thus, 0.1 means that
     the joystick has to move 10% of the way to the edge of an axis's range before that axis will output
     a non-zero value. Linux does its own deadzone processing, so in many cases this value can be set to
     zero, default is 0.01
     
   - autorepeat_rate (:obj:`float`): Rate in Hz at which a joystick that has a non-changing state will resend the
     previously sent message, disabled if 0.0, default is 2.0
     
   - coalesce_interval (:obj:`float`): Axis events that are received within coalesce_interval (seconds) of
     each other are sent out in a single ROS message. Since the kernel sends each axis motion as
     a separate event, coalescing greatly reduces the rate at which messages are sent. This option
     can also be used to limit the rate of outgoing messages. Button events are always sent out immediately
     to avoid missing button presses, default is 0.02




