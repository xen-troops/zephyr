Watchdog support
================
Zephyr provides API for the watchdog timer. This can be checked using the following sample:

.. code-block:: bash

    west build -b rz_g3s -p always samples/drivers/watchdog/

This sample is part of the Zephyr samples collection.
The details about the test work can be found in the following link:
`Zephyr Project Watchdog Sample
<https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/drivers/watchdog/README.rst>`_

The result will be the following:

::

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

Also Zephyr provides test for the watchdog timer. It can be built using the following command:

.. code-block:: bash

    west build -b rz_g3s -p tests/drivers/watchdog/wdt_basic_api/

It runs several tests: running WDT without a callback and waiting for the SoC to reset,
running WDT with a callback and waiting for it to fire, setting an invalid timeout
and waiting for the WDT API to generate an error.

The result will be the following:

::

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
