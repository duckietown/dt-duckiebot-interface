led\_emitter package
====================

.. contents::

The `led_emitter` package blablabla


Quick start
-----------

To test the LED emitter run the following command::

     roslaunch led_emitter led_emitter_node.launch veh:=![robot name]


This launches the LED emitter node. Note that the either the coordination node or the
JoyMapperNode node should be launched in order to be able to use the LEDEmitterNode.

LEDEmitterNode
--------------

.. autoclass:: nodes.LEDEmitterNode

Included libraries
------------------

rgb_led
^^^^^^^

.. automodule:: rgb_led
