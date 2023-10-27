.. _ina219:

Sample for parallel reading of two INA219 current sensors
#########################################################

Overview
********

This sample demonstrates the ability to parallel read data from two INA219 sensors
located on the same I2C bus from two threads and output them to the console.
It also allows to identify possible collisions during parallel access to I2C
from different threads.
Reading occurs every 100 ms in each thread, while the led0 LED in the first thread
and the led1 LED in the second are toggled.
The calibration/configuration parameters can be set in the devicetree file.

References
**********

 - `2 INA219 sensors <https://www.ti.com/product/INA219>`_

Wiring
******

Using address pins, set the address to 0x40 for the first sensor (default)
and 0x41 for the second. Connect both sensors to the I2C3 bus.
The supply voltage of the INA219 can be in the 3V to 5.5V range.
The common mode voltage of the measured bus can be in the 0V to 26V range.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/ina219_2sensors
   :board: rz_a2m
   :goals: build flash

Sample Output
=============
When monitoring a 3 V bus on first sensor and 0 V on second, you should get
a similar output as below:

.. code-block:: console

        ID      Bus[V]  Pow[W]  Curr[A]
        0       3       0       0
        1       0       0       0

If a collision occurs while accessing the I2C bus at the same time, you will receive the following message:

.. code-block:: console

    Could not fetch sensor data.

Terminal output is reduced as much as possible because it slows down the flow and can cause problems with the sample.
