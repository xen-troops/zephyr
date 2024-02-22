.. _snippet-rz-g3s-canfd0-test:

RZ/G3S CAN-FD CH0 Test
#########################################

.. code-block:: bash

   west build -p always -b rz_g3s -S rz-g3s-canfd0-test tests/drivers/can/api

Overview
********

This snippet enables RZ/G3S CAN-FD channel 0 functionality on RZ/G3S SMARC Evaluation Board,
so it can be used with CAN tests and samples.
