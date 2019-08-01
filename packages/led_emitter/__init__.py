"""
    The `led_emitter` package blablabla

    .. contents::

    Quick start
    -----------

    To test the LED emitter run the following command::

        $ roslaunch led_emitter led_emitter_node.launch veh:=![robot name]

    This launches the LED emitter node. Note that the either the coordination node or the led_joy_mapper node should be launched in order to be able to use the led_emitter_node.

    Nodes
    -----

    .. automodule:: led_emitter.src

"""

from .src import *