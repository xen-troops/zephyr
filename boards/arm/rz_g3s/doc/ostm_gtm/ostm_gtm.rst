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
        compatible = "renesas,ostm-r9a08g045-timer", "renesas,ostm-timer";
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

GTM as Zephyr OS timer testing
------------------------------

.. toctree::
   :maxdepth: 4

   timer_tickless_test.rst

GTM as Zephyr Counter
---------------------

Zephyr RZ/G3S Renesas GTM/OSTM Counter driver provides Zephyr :ref:`counter_api` System interface implementation.

GTM/OSTM counter can't support both alarms and changing top value in free-running mode,
so this driver allows to use either alarms in free-running (counting up) mode, or
either changing top (CMP) value in interval mode (counting down).

Therefore:

 * driver starts with counter in free-running mode with alarms support by default;
 * set top value is rejected if alarm is active;
 * set top value causes counter to switch to interval mode and disables alarms;
 * set top to RZ_GTM_TIMER_TOP_VALUE (0xFFFFFFFF) switches counter back to free-running mode and enables alarms;

Over all it could be practical to use GTM Counter instance as either free-running with alarms or as
interval counter with supporting of changing top value, but not both.

The Zephyr Counter subsystem is **not** enabled by default in ``rz_g3s_defconfig``. To enable Zephyr
Counter functionality and RZ G3S GTM/OSTM Counter driver below Kconfig options have to be enabled:

.. code-block:: text

    CONFIG_COUNTER=y
    /* automatically enabled */
    CONFIG_COUNTER_RZ_GTM_COUNTER=y

The RZ G3S GTM/OSTM Counter driver code can be found at:

.. code-block:: text

    drivers/counter/counter_rz_gtm.c

GTM Counter testing
-------------------

.. toctree::
   :maxdepth: 4

   counter_api_test.rst

.. raw:: latex

    \newpage
