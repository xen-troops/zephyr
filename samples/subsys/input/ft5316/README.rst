.. zephyr:code-sample:: ft5316
   :name: ft5316
   :relevant-api: input_events

   Print input events from touch panel.

Overview
********

The ft5316 sample prints input event using the :ref:`input` APIs from FT5x16
family touch panels.

Requirements
************

The samples works on any board with an input,i2c and ft5336 drivers defined
in the board devicetree.

Building and Running
********************

Build and flash as follows, changing ``rz_a2m`` for your board:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/input/ft5316
   :board: rz_a2m
   :goals: build flash
   :compact:

After starting, the sample will print any input event in the console.
