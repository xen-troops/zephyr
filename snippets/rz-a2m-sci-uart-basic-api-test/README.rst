.. _snippet-rza-a2m-mtu-pwm-api-test:

RZ/A2M UART basic API Test for SCI uart device (rz-a2m-sci-uart-basic-api-test)
#########################################

.. code-block:: console

   west build -p always -b rz_a2m -S rz-a2m-sci-uart-basic-api-test tests/drivers/uart/uart_basic_api

Overview
********

This snippet setup default console to `sci0` for `tests/drivers/uart/uart_basic_api`
test in order to run this test with `sci0` device as "device under test".
