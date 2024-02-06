.. _rz_g3s:

#################################
Renesas RZ G3S Release Notes v0.1
#################################

.. contents::
   :depth: 5

Overview
********
This evaluation board kit is ideal for evaluating RZ/G3S. The RZ/G3S Evaluation Board Kit consists
of a module board (SOM) and a carrier board. The module board complies with the SMARC v2.1 standard.

.. include:: delivery/delivery_scope.rst

.. include:: delivery/getting_source_code.rst

.. include:: delivery/initial_config.rst

Hardware
********

The RZ G3S includes:

* Device: RZ/G3S R9A08G045S33GBG

  * Cortex-A55 Single, Cortex-M33 x2
  * BGA 359-pin, 14mmSq body, 0.5mm pitch

* Module Board Functions

  * LPDDR4 SDRAM: 1GB × 1pc
  * QSPI flash memory: 128Mb × 1pc
  * eMMC memory: 64GB × 1pc
  * PMIC power supply RAA215300A2GNP#HA3 implemented
  * microSD card x2
  * I3C connector
  * JTAG connector
  * ADC x8 channels
  * Current monitor (USB Micro B)

* Carrier Board Functions

  * Gigabit Ethernet x2
  * USB2.0 x2ch (OTG x1ch, Host x1ch)
  * CAN-FD x2
  * microSD card x1
  * Mono speaker, Stereo headphone, Mic., and Aux..
  * PMOD x2
  * USB-Type C for power input
  * PCIe Gen2 4-lane slot (G3S supports only 1-lane)
  * M.2 Key E
  * M.2 Key B and SIM card
  * Coin cell battery holder (3.0V support)

Connections and IOs
===================

RZ G3S board
------------

Views of the Renesas RZ/G3S SMARC Evaluation Board Kit board ``rz_g3s``:

|

.. figure:: img/rzg3s.jpg
   :align: center

Supported Features
==================

The Renesas ``rz_g3s`` board configuration supports the following
hardware features:

+-----------+------------------------------+--------------------------------+
| Interface | Driver/components            | Support level                  |
+===========+==============================+================================+
| PINCTRL   | pinctrl                      |                                |
+-----------+------------------------------+--------------------------------+
| CLOCK     | clock_control                |                                |
+-----------+------------------------------+--------------------------------+
| UART      | uart                         | serial port-polling            |
+-----------+------------------------------+--------------------------------+
| SPI       | spi                          | 8/16 bit transfers             |
+-----------+------------------------------+--------------------------------+
| I2C       | i2c                          |                                |
+-----------+------------------------------+--------------------------------+
| Watchdog  | wdt                          |                                |
+-----------+------------------------------+--------------------------------+

Other hardware features have not been enabled yet for this board.

The default configuration can be found in the defconfig file:

        ``boards/arm/rz_g3s/rz_g3s_defconfig``

Programming and Debugging
*************************

Applications for the ``rz_g3s`` boards can be built in the usual way as documented
in :ref:`build_an_application`.

Currently is only possible to load and execute a Zephyr application binary on
this board from the internal SRAM, using ``JLink`` debugger.

Here is an example for the :ref:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rz_g3s
   :goals: build

These are the memory mapping for A55 and M33:

+------------+-----------------------+------------------------+-----------------------+
| Region     | Cortex-A55            | Cortex-M33             | Cortex-M33-FPU        |
+============+=======================+========================+=======================+
| SRAM       | 0x00020000-0x00022FFF | 0x00023000-0x0005FFFF  | 0x10000000-0x1FFEFFFF |
+------------+-----------------------+------------------------+-----------------------+
|            |                       |                        |                       |
+------------+-----------------------+------------------------+-----------------------+

Flashing
========

The flash on board is not supported by Zephyr at this time.

Supported Features Details
**************************

.. include:: cortex-m/mpu.rst

.. include:: cortex-m/fpu.rst

.. include:: pinctrl/pinctrl.rst

.. include:: icu/interrupts.rst

.. include:: uart/uart.rst

.. include:: gpio/gpio.rst

.. include:: rspi/rspi.rst

.. include:: i2c/i2c.rst

.. include:: wdt/wdt.rst
