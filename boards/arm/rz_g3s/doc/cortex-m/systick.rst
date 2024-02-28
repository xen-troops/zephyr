Cortex-M33 System timer (SysTick)
=================================

SysTick overview
----------------

The Renesas RZ G3S SoC has the SysTick timers in the Cortex-M33/Cortex-M33_FPU.

* There is one Secure SysTick timer and one Non-secure SysTick timer.
* The count cycle of SysTick timers is common to Secure and Non-secure.

Refer to Arm® Cortex®-M33 Processor Revision: r0p4 Technical Reference Manual
for more information.

SysTick driver overview
-----------------------

The ARM SysTick serves function of Zephyr Kernel System clock as described
in :ref:`kernel_timing` documentation.

Zephyr has generic support for the ARM SysTick which is enabled for
the Renesas RZ G3S Soc by declaring ``CONFIG_CPU_CORTEX_M_HAS_SYSTICK`` Kconfig option by default.
This also enables below Kconfig options by default.

.. code-block:: text

    CONFIG_CPU_CORTEX_M_HAS_SYSTICK=y
    CONFIG_CORTEX_M_SYSTICK=y

The generic Zephyr ARM Cortex-M SYSTICK timer driver code can be found at:

.. code-block:: text

    drivers/timer/cortex_m_systick.c
    drivers/timer/Kconfig.cortex_m_systick

.. raw:: latex

    \newpage
