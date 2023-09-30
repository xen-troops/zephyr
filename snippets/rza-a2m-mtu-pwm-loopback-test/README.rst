.. _snippet-rza-a2m-mtu-pwm-loopback-test:

RZ/A2M MTU PWM Loopback Test (rza-a2m-mtu-pwm-loopback-test)
#########################################

.. code-block:: console

   west build -p always -b rz_a2m -S rza-a2m-mtu-pwm-loopback-test tests/drivers/pwm/pwm_loopback

Overview
********

This snippet overwrite `pwms` property in node `pwm_loopback_0`
from `tests/drivers/pwm/pwm_loopback` test in order to run this test with the MTU timers.
To run the test, you need to connect pins CN17-21 and CN17-11 together.
