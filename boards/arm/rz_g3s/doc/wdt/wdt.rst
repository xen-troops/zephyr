Watchdog Timer (WDT)
====================

WDT overview
------------

The Renesas RZ G3S Soc has 3 channels of watchdog timer and generates a reset request signal when the counter value is
not rewritten and overflows due to system runaway.

* WDT CH0 - WDT to check the operation of Cortex-A55-CPU
* WDT CH1 - WDT to check the operation of Cortex-M33 CPU
* WDT CH2 - WDT to check the operation of Cortex-M33_FPU CPU

Each WDT channel can reset only specific CPU Core or cause System (SoC) reset.

Refer to "Watchdog Timer (WDT)" section in "Renesas RZ/G3S Group User’s Manual: Hardware"

WDT driver overview
-------------------

Zephyr RZ/G3S Renesas WDT driver provides Zephyr :ref:`watchdog_api` System interface implementation.
Zephyr RZ/G3S Renesas WDT driver supports following Watchdog behavior flags:

* Reset: none - WDT_FLAG_RESET_NONE;
* Reset: CPU core - WDT_FLAG_RESET_CPU_CORE;
* Reset: SoC - WDT_FLAG_RESET_SOC.

The WATCHDOG subsystem is **not** enabled by default in ``rz_g3s_defconfig``. To enable Zephyr
WDT functionality and RZ G3S WDT driver below Kconfig options have to be enabled:

.. code-block:: text

    CONFIG_WATCHDOG=y
    /* automatically enabled */
    CONFIG_WDT_RZG=y

The RZ G3S WDT driver code can be found at:

.. code-block:: text

    drivers/watchdog/wdt_rzg.c

WDT testing
-----------

samples/drivers/watchdog
````````````````````````

Zephyr RZ/G3S WDT driver can be tested by using **samples/drivers/watchdog** sample application.
Use below command to build WDT **samples/drivers/watchdog** sample application:

.. code-block:: bash

    west build -b rz_g3s -p always samples/drivers/watchdog

The details about the **samples/drivers/watchdog** test can be found in the following link:
`Zephyr Project Watchdog Sample
<https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/drivers/watchdog/README.rst>`_

The **samples/drivers/watchdog** sample application will produce below console output when executed:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-231-g585ed1539f97 ***
    Watchdog sample application
    Attempting to test pre-reset callback
    [00:00:00.010,000] <dbg> wdt_rzg: wdt_rzg_install_timeout: watchdog@42800400: configuring reset SOC mode
    Feeding watchdog 5 times
    Feeding watchdog...
    Feeding watchdog...
    Feeding watchdog...
    Feeding watchdog...
    Feeding watchdog...
    Waiting for reset...
    Handled things..ready to reset

After this, the SoC should reset.

tests/drivers/watchdog/wdt_basic_api
````````````````````````````````````

.. note::

    This test is not working from JLink debugger (west debug) and has to be run by flashing on xSPI/eMMC.
    This is because test code stores some data in the memory and expects it no to be corrupted after reset.
    In case of running WDT test from JLink debugger system will be rebooted by JLink which causes Cortex-A55
    to start and TF-A to reinitialize memory which leads to test data corruption.

Zephyr RZ/G3S WDT driver can be tested by using **wdt_basic_api** test application.
Use below command to build WDT **wdt_basic_api** test application:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/watchdog/wdt_basic_api

It runs several tests: running WDT without a callback and waiting for the Cortex-M33 to reset,
running WDT with a callback and waiting for it to fire and Cortex-M33 to reset,
setting an invalid timeout and waiting for the WDT API to generate an error.

Only Cortex-M33 System Core will be reset during this test.

The **wdt_basic_api** will application will produce below console output when executed:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-231-g585ed1539f97 ***
    Running TESTSUITE wdt_basic_test_suite
    ===================================================================
    START - test_wdt
    Testcase: test_wdt_no_callback
    Waiting to restart MCU
    Running TESTSUITE wdt_basic_test_suite
    ===================================================================
    START - test_wdt
    Testcase: test_wdt_no_callback
    Testcase passed
    Testcase: test_wdt_callback_1
    Waiting to restart MCU
    Running TESTSUITE wdt_basic_test_suite
    ===================================================================
    START - test_wdt
    Testcase: test_wdt_callback_1
    Testcase passed
    Testcase: test_wdt_bad_window_max
    E: watchdog@42800400: invalid timeout val(s) min 0 max 0
    PASS - test_wdt in 0.013 seconds
    ===================================================================
    TESTSUITE wdt_basic_test_suite succeeded
    ------ TESTSUITE SUMMARY START ------
    SUITE PASS - 100.00% [wdt_basic_test_suite]: pass = 1, fail = 0, skip = 0, total = 1 duration = 0.013 ss
    - PASS - [wdt_basic_test_suite.test_wdt] duration = 0.013 seconds
    ------ TESTSUITE SUMMARY END ------
    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

.. raw:: latex

    \newpage
