General Timer (GTM/OSTM)
========================

GTM overview
------------

The Renesas RZ G3S SoC has the 8 General Timers (GTM/OSTM).

The General timer has the following features.

    * Two operating modes

        * Interval timer mode
        * Free-running comparison mode

    * Choice between startup of DMA by compare match and generation of interrupt

In Zephyr the RZ G3S General Timers can be enabled to serve as:

* Zephyr OS timer
* Zephyr Counter

Refer to "General Timer (GTM)" section in "Renesas RZ/G3S Group User’s Manual: Hardware"

GTM as Zephyr OS timer
----------------------

When GTM is enabled as Zephyr OS timer it serves function of Zephyr Kernel System clock as described
in :ref:`kernel_timing` documentation.

On Renesas RZ G3S platform the GTM is **not** enabled as Zephyr OS timer by default.
To enable GTM as Zephyr OS timer the **"compatible"** property of the selected GTM has to be updated to include **"renesas,ostm-timer"** and Cortex-M **Systyck** has to be disabled. The below DT example shows enabling of GTM0 as Zephyr OS timer:

.. code-block:: dts

    &gtm0 {
        compatible = "renesas,ostm-r9a08g045", "renesas,ostm-timer", "renesas,ostm";
        status = "okay";
    };

    &systick {
        status = "disabled";
    };

The Zephyr snippet **rz-g3s-gtm-timer-test** can be used for testing purposes and already contains DT changes for enabling GTM0 as Zephyr OS timer:

.. code-block:: text

    snippets/rz-g3s-gtm-timer-test/README.rst
    snippets/rz-g3s-gtm-timer-test/rz-g3s-gtm-timer-test.overlay
    snippets/rz-g3s-gtm-timer-test/snippet.yml

The GTM OS timer driver will be enabled in Kconfig once above DT changes are applied.

.. code-block:: text

    CONFIG_RZ_OS_TIMER=y
    /* automatically enabled */
    CONFIG_RZ_OS_TIMER=y

The GTM OS timer driver code can be found at:

.. code-block:: text

    drivers/timer/rz_os_timer.c
    drivers/timer/Kconfig.rz

GTM as Zephyr OS timer
----------------------

tests/kernel/tickless/tickless_concept
``````````````````````````````````````

Zephyr RZ/G3S GTM as Zephyr OS timer driver can be tested by using **tests/kernel/tickless/tickless_concept** test.
To build **tests/kernel/tickless/tickless_concept** test run command:

.. code-block:: bash

   west build -p always -b rz_g3s -S rz-g3s-gtm-timer-test tests/kernel/tickless/tickless_concept

Console output:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-331-gb7a06954094c ***
    Running TESTSUITE tickless_concept
    ===================================================================
    START - test_tickless_slice
    elapsed slice 110, expected: <100, 110>
    elapsed slice 100, expected: <100, 110>
    elapsed slice 100, expected: <100, 110>
    elapsed slice 100, expected: <100, 110>
     PASS - test_tickless_slice in 0.611 seconds
    ===================================================================
    START - test_tickless_sysclock
    time 2920, 3130
    time 3140, 3340
     PASS - test_tickless_sysclock in 0.426 seconds
    ===================================================================
    TESTSUITE tickless_concept succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [tickless_concept]: pass = 2, fail = 0, skip = 0, total = 2 duration = 1.037
    ses
     - PASS - [tickless_concept.test_tickless_slice] duration = 0.611 seconds
     - PASS - [tickless_concept.test_tickless_sysclock] duration = 0.426 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

.. raw:: latex

    \newpage
