.. _rz_g3s:
.. _rz_g3s_fpu:

#################################
RZ/G3S SMARC Evaluation Board Kit
#################################

.. only:: html

  .. contents::
     :depth: 5

.. _Overview:

Overview
********

The Renesas Evaluation Board Kit for RZ/G3S MPU (RZ/G3S-EVKIT) consists
of a module board (SOM) and a carrier board. The module board complies with the SMARC v2.1 standard.

.. toctree::
   :maxdepth: 4

   delivery/delivery_scope.rst
   delivery/getting_source_code.rst
   delivery/yocto_build.rst
   delivery/hello_world_start.rst

Hardware
********

The Renesas RZ/G3S MPU documentation can be found at:

`RZ/G3S MPU <https://www.renesas.com/us/en/products/microcontrollers-microprocessors/rz-mpus/rzg3s-general-purpose-microprocessors-single-core-arm-cortex-a55-11-ghz-cpu-and-dual-core-cortex-m33-250>`_

The Renesas Evaluation Board Kit for RZ/G3S MPU (RZ/G3S-EVKIT) documentation can be found at:

`Evaluation Board Kit for RZ/G3S MPU <https://www.renesas.com/us/en/products/microcontrollers-microprocessors/rz-mpus/rzg3s-evkit-evaluation-board-kit-rzg3s-mpu>`_

`RZG3S SMARC Module Board User's Manual Hardware <https://www.renesas.com/us/en/document/mat/rzg3s-smarc-module-board-users-manual-hardware>`_

`RZ SMARC Series Carrier Board II User's Manual Hardware <https://www.renesas.com/us/en/document/mat/rz-smarc-series-carrier-board-ii-users-manual-hardware?r=25458596>`_

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

.. toctree::
   :maxdepth: 4

   delivery/initial_config.rst

Supported Features
==================

The Renesas ``rz_g3s`` board configuration supports the following
hardware features:

+----------+------------------+--------------------+
| Interface| Driver/components| Support level      |
+==========+==================+====================+
| PINCTRL  | pinctrl          |                    |
+----------+------------------+--------------------+
| CLOCK    | clock_control    |                    |
+----------+------------------+--------------------+
| gpio     | gpio             |                    |
+----------+------------------+--------------------+
| UART     | uart             | serial port-polling|
+----------+------------------+--------------------+
| rSPI     | spi              | 8/16 bit transfers |
+----------+------------------+--------------------+
| I2C      | i2c              |                    |
+----------+------------------+--------------------+
| Watchdog | wdt              |                    |
+----------+------------------+--------------------+
| ADC      | adc              |                    |
+----------+------------------+--------------------+
| CAN-FD   | can              |                    |
+----------+------------------+--------------------+
| DMAC     | dma              |                    |
+----------+------------------+--------------------+
| GTM      | timer            | GTM as OS Timer    |
+----------+------------------+--------------------+
| GTM      | counter          | GTM as Counter     |
+----------+------------------+--------------------+
| GPT      | pwm              |                    |
+----------+------------------+--------------------+
| POEG     | pwm              | POEG for GPT       |
+----------+------------------+--------------------+

Other hardware features have not been enabled yet for this board.

The default configuration can be found in the defconfig file:

.. code-block:: text

    boards/arm/rz_g3s/rz_g3s_defconfig

.. _rz_g3s_prog_debug:

Programming and Debugging
*************************

.. note::

    Ensure all prerequisite steps, described in :ref:`Overview` section, are done and RZ/G3S-EVKIT board has corresponding BL2 bootloader
    (ARM trusted firmware TF-A with **PLAT_M33_BOOT_SUPPORT** option enabled) flashed on eMMC(xSPI) and BL2 TF-A is booted properly.

Applications for the ``rz_g3s`` board can be built in the usual way as documented
in :ref:`build_an_application`.

The generating of binaries for RZ G3S Cortex-M33 and Cortex-M33_FPU system cores is
supported by using different board names:

* ``rz_g3s`` for Cortex-M33
* ``rz_g3s_fpu`` for Cortex-M33_FPU

These are the memory mapping for A55 and M33:

+----------+-----------------------+-----------------------+
| Region   | Cortex-M33            | Cortex-M33-FPU        |
+==========+=======================+=======================+
| SRAM code| 0x00023000-0x0005FC00 | 0x00063000-0x0009FC00 |
+----------+-----------------------+-----------------------+
| DDR      | 0x60000000-0x60FFFFFF | 0x61000000-0x61FFFFFF |
+----------+-----------------------+-----------------------+

Console
=======

There are 2 UART ports supported from Zephyr on RZ/G3S SMARC Evaluation Board Kit board,
which assigned to Cortex-M33 System Cores as following:

* Cortex-M33: SER0 from PMOD1_3A
* Cortex-M33_FPU: SER1 from SER1_UART

Refer to the section :ref:`rz_g3s_con` for more information about board connection and console setup.

.. _Debugging:

Debugging
=========

It is possible to load and execute a Zephyr application binary on
this board on one of the Cortex-M33/Cortex-M33_FPU System Cores from
the internal SRAM, using ``JLink`` debugger (:ref:`jlink-debug-host-tools`).

.. note::

    Currently it's required Renesas BL2 TF-A to be started on Cortex-A55 System Core
    before starting Zephyr. As BL2 TF-A configures clocks and Cortex-A33 before starting it.

+----------------+--------------------+
|                | JLink device id    |
+================+====================+
| Cortex-M33     | R9A08G045S33_M33_0 |
+----------------+--------------------+
| Cortex-M33_FPU | R9A08G045S33_M33_1 |
+----------------+--------------------+

Here is an example for the :ref:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rz_g3s
   :goals: debug

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rz_g3s_fpu
   :goals: debug

Flashing
========

.. note::

    The flashing using west environment is not fully supported.

Zephyr application can be flashed to eMMC or qSPI storage and then loaded by
Renesas BL2 TF-A running on Cortex-A55 System Core.
and staring binary at Cortex-M33 System Core.

.. note::

    Flashing is supported only for Cortex-M33 System Core.
    Zephyr application can be started on Cortex-M33_FPU System Core only by using debugger.

More information about flashing to eMMC(xSPI) can be found in Renesas document:
`Linux Start-up Guide for RZ/G3S Board Support Package`_.

The Zephyr application binary has to be converted to Motorolla S-record `SREC`_ format
which is generated automatically in Zephyr application build directory (`*zephyr.srec*`).

.. _SREC: https://en.wikipedia.org/wiki/SREC_(file_format)

Follow `Linux Start-up Guide for RZ/G3S Board Support Package`_:

* section "4.2.3 Settings" for enabling "SCIF Download mode"
* section "4.3 Download Flash Writer to RAM" (FlashWriter-smarc-rzg3s.mot)

.. _Linux Start-up Guide for RZ/G3S Board Support Package: https://www.renesas.com/us/en/document/mas/linux-start-guide-rzg3s-board-support-package-v100


.. _Flashing on eMMC:

Flashing on eMMC
----------------

* Download and start **Flash Writer** as described above.
* Use **Flash Writer EM_W** command to flash Zephyr binary
* Input when asked:

.. code-block:: console

    Select area(0-2)>1
    Please Input Start Address in sector :1000
    Please Input Program Start Address : 23000

* then send Zephyr **srec** file from terminal (use ''ascii'' mode)
* reboot the board in the **eMMC Boot Mode**

.. code-block:: console

    >EM_W
    EM_W Start --------------
    ---------------------------------------------------------
    Please select,eMMC Partition Area.
    0:User Partition Area : 62160896 KBytes
     eMMC Sector Cnt : H'0 - H'0768FFFF
    1:Boot Partition 1 : 32256 KBytes
     eMMC Sector Cnt : H'0 - H'0000FBFF
    2:Boot Partition 2 : 32256 KBytes
     eMMC Sector Cnt : H'0 - H'0000FBFF
    ---------------------------------------------------------
     Select area(0-2)>1
    -- Boot Partition 1 Program -----------------------------
    Please Input Start Address in sector :1000
    Please Input Program Start Address : 23000
    Work RAM (H'00020000-H'000FFFFF) Clear....
    please send ! ('.' & CR stop load)

.. _Flashing on qSPI:

Flashing on qSPI
----------------

Zephyr binary has to be converted to **srec** format.

* Download and start **Flash Writer** as described above
* Use **Flash Writer XLS2** command to flash Zephyr binary
* Input when asked:

.. code-block:: console

    ===== Please Input Program Top Address ============
      Please Input : H'23000
    ===== Please Input Qspi Save Address ===
      Please Input : H'200000

* then send Zephyr **srec** file from terminal (use ''ascii'' mode)
* reboot the board in the **qSPI Boot Mode**

.. code-block:: console

     -- Load Program to SRAM ---------------

    Flash writer for RZ/G3S Series V0.60 Jan.26,2023
     Product Code : RZ/G3S
    >XLS2
    ===== Qspi writing of RZ/G2 Board Command =============
    Load Program to Spiflash
    Writes to any of SPI address.
    Program size & Qspi Save Address
    ===== Please Input Program Top Address ============
      Please Input : H'23000

    ===== Please Input Qspi Save Address ===
      Please Input : H'200000
    please send ! ('.' & CR stop load)
    I Flash memory...
    Erase Completed
    Write to SPI Flash memory.
    ======= Qspi  Save Information  =================
     SpiFlashMemory Stat Address : H'00200000
     SpiFlashMemory End Address  : H'002098E6
    ===========================================================

Supported Features Details
**************************

.. toctree::
   :maxdepth: 4

   cortex-m/fpu.rst
   cortex-m/mpu.rst
   cortex-m/systick.rst
   cortex-m/sau.rst
   cpg/cpg.rst
   pinctrl/pinctrl.rst
   icu/interrupts.rst
   uart/uart.rst
   gpio/gpio.rst
   rspi/rspi.rst
   i2c/i2c.rst
   wdt/wdt.rst
   adc/adc.rst
   canfd/canfd.rst
   ostm_gtm/ostm_gtm.rst
   openamp/mhu.rst
   openamp/openamp.rst
   pwm/GPT.rst
   dma/dma.rst
