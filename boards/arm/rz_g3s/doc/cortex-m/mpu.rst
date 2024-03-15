Cortex-M33 Memory Protection Unit (MPU)
=======================================

MPU overview
------------

The Renesas RZ G3S SoC Cortex-M33/Cortex-M33_FPU (r0p4) have MPU built in.
The MPU improves system reliability by defining the memory attributes for
different memory regions.

It provides up to 16 different regions. The MPU doesn't support overlapped
memory regions.

When the Security Extension is included, there can be two MPUs, one Secure
and one Non-secure. Each MPU can define memory attributes independently.

Refer to Arm® Cortex®-M33 Processor Revision: r0p4 Technical Reference Manual
for more information.

MPU default configuration
-------------------------

The MPU is enabled by default in ``rz_g3s_defconfig``.

.. code-block:: text

    CONFIG_ARM_MPU=y

At boot time Zephyr configures following static memory regions and regions defined in DT:

.. raw:: latex

    \begin{scriptsize}

+----------------+--------------------------------------------------------------+
| Region         |                                                              |
+================+==============================================================+
| NULL           | Optional. Any, used for null pointer dereference check       |
|                |                                                              |
|                | Enabled in Kconfig                                           |
+----------------+--------------------------------------------------------------+
| vector         | EXEC, P_RO_U_RO_Msk, NON_SHAREABLE_Msk                       |
+----------------+--------------------------------------------------------------+
| SRAM_TEXT      | EXEC, P_RO_U_RO_Msk, NON_SHAREABLE_Msk                       |
+----------------+--------------------------------------------------------------+
| SRAM_RODATA    | NOT_EXEC, P_RW_U_NA_Msk, NON_SHAREABLE_Msk                   |
+----------------+--------------------------------------------------------------+
| DEVICE         | Optional. NOT_EXEC, P_RW_U_NA_Msk, NON_SHAREABLE_Msk,NOCACHE |
|                |                                                              |
|                | Enabled if defined(CONFIG_MPU_DISABLE_BACKGROUND_MAP)        |
+----------------+--------------------------------------------------------------+

.. raw:: latex

    \end{scriptsize}

More regions can be added if specified properly in DT by using standard
Zephyr DT memory **"zephyr,memory-region"** bindigs.

Refer to Zephyr :ref:`mem_mgmt_api` documentation.

MPU testing
-------------

samples/arch/mpu/mpu_test
`````````````````````````

To build **mpu_test** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always samples/arch/mpu/mpu_test

The **mpu_test** is not automatic test and required entering commands from
console and reboot after each test:

.. code-block:: text

    mpu read
    mpu write
    mpu run

The below is console output of test execution:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-230-ga80246d0863d ***
    uart:~$ mpu read

    [00:00:05.979,000] <err> os: ***** BUS FAULT *****
    [00:00:05.979,000] <err> os:   Precise data bus error
    [00:00:05.979,000] <err> os:   BFAR Address: 0x24060000
    [00:00:05.979,000] <err> os: r0/a1:  0x0002efe4  r1/a2:  0x00000008  r2/a3:  0x20061980
    [00:00:05.979,000] <err> os: r3/a4:  0x24060000 r12/ip:  0x0003026a r14/lr:  0x000263eb
    [00:00:05.979,000] <err> os:  xpsr:  0x21000000
    [00:00:05.979,000] <err> os: Faulting instruction address (r15/pc): 0x00023c8a
    [00:00:05.979,000] <err> os: >>> ZEPHYR FATAL ERROR 25: Unknown error on CPU 0
    [00:00:05.979,000] <err> os: Current thread: 0x20060668 (shell_uart)
    [00:00:06.048,000] <err> os: Halting system

    *** Booting Zephyr OS build v3.5.0-rc2-230-ga80246d0863d ***
    uart:~$ mpu write
    write address: 0
    [00:00:05.917,000] <err> os: ***** MPU FAULT *****
    [00:00:05.917,000] <err> os:   Data Access Violation
    [00:00:05.917,000] <err> os:   MMFAR Address: 0x27000
    [00:00:05.917,000] <err> os: r0/a1:  0x00000000  r1/a2:  0x0000000e  r2/a3:  0x0badc0de
    [00:00:05.917,000] <err> os: r3/a4:  0x00027000 r12/ip:  0x00000001 r14/lr:  0x00023c6f
    [00:00:05.918,000] <err> os:  xpsr:  0x61000000
    [00:00:05.918,000] <err> os: Faulting instruction address (r15/pc): 0x00023c76
    [00:00:05.918,000] <err> os: >>> ZEPHYR FATAL ERROR 19: Unknown error on CPU 0
    [00:00:05.918,000] <err> os: Current thread: 0x20060668 (shell_uart)
    [00:00:05.987,000] <err> os: Halting system

    *** Booting Zephyr OS build v3.5.0-rc2-230-ga80246d0863d ***
    uart:~$ mpu run

    [00:00:13.740,000] <err> os: ***** MPU FAULT *****
    [00:00:13.740,000] <err> os:   Instruction Access Violation
    [00:00:13.740,000] <err> os: r0/a1:  0x0002efe4  r1/a2:  0x00000001  r2/a3:  0x20061980
    [00:00:13.740,000] <err> os: r3/a4:  0x20060000 r12/ip:  0x0003029d r14/lr:  0x00023c57
    [00:00:13.740,000] <err> os:  xpsr:  0x60000000
    [00:00:13.740,000] <err> os: Faulting instruction address (r15/pc): 0x20060000
    [00:00:13.740,000] <err> os: >>> ZEPHYR FATAL ERROR 20: Unknown error on CPU 0
    [00:00:13.740,000] <err> os: Current thread: 0x20060668 (shell_uart)
    [00:00:13.803,000] <err> os: Halting system

.. raw:: latex

    \newpage
