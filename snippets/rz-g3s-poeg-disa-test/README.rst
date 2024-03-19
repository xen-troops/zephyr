.. _snippet-rz-g3s-poeg-disa-test:

RZ/G3S CAN-FD CH0 Test
#########################################

.. code-block:: bash

   west build -p always -b rz_g3s -S rz-g3s-poeg-disa-test tests/drivers/pwm/pwm_ab

Overview
********

This snippet enables RZ/G3S enables Disable request from POEG on to GPT on the
same level of output pins.
