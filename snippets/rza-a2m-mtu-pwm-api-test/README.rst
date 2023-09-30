.. _snippet-rza-a2m-mtu-pwm-api-test:

RZ/A2M MTU PWM API Test (rza-a2m-mtu-pwm-api-test)
#########################################

.. code-block:: console

   west build -p always -b rz_a2m -S rza-a2m-mtu-pwm-api-test tests/drivers/pwm/pwm_api

Overview
********

This snippet overwrite `pwm-0` alias property for `tests/drivers/pwm/pwm_loopback`
test in order to run this test with the MTU timer(s).
