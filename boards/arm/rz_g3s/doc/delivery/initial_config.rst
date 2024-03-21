.. _rz_g3s_con:

RZ/G3S-EVKIT board connection
-----------------------------

Views of the Renesas RZ/G3S SMARC Evaluation Board Kit board ``rz_g3s``:

.. figure:: ../img/rzg3s_con.jpg
   :align: center
   :height: 250px

The connection diagram of RZ/G3S-EVKIT and PC is shown below:

.. figure:: ../img/rzg3s_con_diag.jpg
   :align: center

* USB Type-C Main Power - connected to USB Type-C power supply
* micro-USB SER3_UART - shell be connected to PC.
  It's used as console for SW running on Cortex-A55 System Core.
* PMOD1_3A SER0_UART (3.3V) - shell be connected to PC using UART-USB adapter, like YP-05.
  It's used as console for SW running on Cortex-M33 System Core.

.. figure:: ../img/rzg3s_ser0.jpg
   :align: center
   :height: 250px

* 4-pin connector SER1_UART (1.8V) - shell be connected to PC using UART-USB adapter, like MCS-73LV.
  It's used as console for SW running on Cortex-M33_FPU System Core.

.. image:: ../img/SER1.jpg
   :height: 250px
   :align: center

* The G3S SMARC Module JTAG interface - shell be connected to PC through SEGGER JLink probe

.. image:: ../img/rzg3s_jlink.jpg
   :height: 250px
   :align: center

.. _Linux minicom terminal:

Linux minicom terminal
``````````````````````
The **minicom** is a terminal program for Linux and other unix-like systems. Use below command to connect to the dedicated serial device:

.. code-block:: bash

    sudo minicom -D /dev/<tty dev>

When console port of RZ/G3S-EVKIT is connected to PC it will create TTY **/dev/ttyX** device.
The device name depends on UART-USB converter HW and usually named as **/dev/ttyUSBN**, where **N** - the number assigned by Linux and it depends on connection or re-connection of (plug/unplug UCN cable) UART-USB converter HW to PC.

For example:

* connect micro-USB SER3_UART to PC - the **/dev/ttyUSB0** serial device will be created (Cortex-A55)
* connect PMOD1_3A SER0_UART to PC - the **/dev/ttyUSB1** serial device will be created (Cortex-M33)

Run below two commands in different terminal windows to access RZ G3S Cortex-A55 and Cortex-M33 console:

.. code-block:: bash

    sudo minicom -D /dev/ttyUSB0
    sudo minicom -D /dev/ttyUSB1

RZ/G3S-EVKIT board default HW configuration
-------------------------------------------

This section describes default RZ/G3S-EVKIT board configuration used for basic Zephyr testing.
It enables eMMC boot by default.

G3S SMARC Module:

* DIP Switch **SW_CONFIG**: **1:OFF 2:OFF 3:ON 4:OFF 5:OFF 6:OFF**
* DIP Switch **GPIO4_SEL**: **(2-3, 5-6)**

The RZ SMARC Carrier II:

* DIP switch **SW_MODE** (Boot Mode): **1:ON 2:OFF 3:OFF**. Module eMMC flash E.g. eMMC
* DIP switch **SW_MODE** (Power): **4:ON** (15V, 2A, 30W, VBUS_SEL=3)
* DIP switch **SW_OPT_MUX**: **1:ON 2:ON 3:OFF 4:ON**
* DIP switch **SW_GPIO_CAN_PMOD** (CAN Standby): **(2-3, 5-6)**
* JP **PMOD_PWR_SEL** (PMOD Power):  should be configured to supply 3.3V
* DIP switch **SW_PMOD0_PWR_SLP** (PMOD Power/Sleep Control): **(2-3, 5-6)**
* DIP switch **SW_GPIO_OPT_SEL** (GPIO Options): **(2-3, 5-6)**
* DIP switch **SW_M2_DIS** (M.2 Card Control Signals): **1:ON 2:ON 3:ON 4:ON**
* DIP switch **SW_PCIE_MUX**: **1:ON 2:OFF 3:OFF 4:ON**

RZ/G3S-EVKIT boot sequence
--------------------------

TODO
