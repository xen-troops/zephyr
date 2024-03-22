Appendix A. west xSPI flash (experimental)
------------------------------------------

.. note::

    The default Renesas flashing process is based on using **FlashWriter**.
    The usage of **west flash** (JLink flash) is considered as experimental and can be used on your own risk only.

The **west flash** for Renesas RZ G3S uses capability of JLink SW and probe to write memory mapped flash memory, which, in this case,
is xSPI memory area. The JLink supports only Cortex-M33 and so the Cortex-M33/Cortex-M33_FPU Address Space shell be used while working with Jlink.

To flash Zephyr application binary **west flash** uses **0x80200000** base address which is compatible with TF-A initial bootloader.

**The minimal version of SEGGER JLink SW which can perform flashing of xSPI memory is v7.96.**

The command:

.. code-block:: bash

    west flash

The output in case of success:

.. code-block:: console

    -- west flash: rebuilding
    ninja: no work to do.
    -- west flash: using runner jlink
    -- runners.jlink: JLink version: 7.96
    -- runners.jlink: Flashing file: /home/xtrs/zephyrproject/zephyr/build/zephyr/zephyr.bin
