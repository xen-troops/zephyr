.. _snippet-rz-g3s-gtm-timer-test:

RZ/G3S GTM OS Timer Test
#########################################

.. code-block:: bash

   west build -p always -b rz_g3s -S rz-g3s-gtm-timer-test tests/kernel/tickless/tickless_concept

Overview
********

This snippet enables RZ/G3S GTM channel 0 functionality on RZ/G3S SMARC Evaluation Board,
so it can be used as Zephyr OS Timer for tests and samples.
