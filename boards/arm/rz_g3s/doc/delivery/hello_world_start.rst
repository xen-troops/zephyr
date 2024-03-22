Starting hello_world application
================================

:ref:`hello_world` is a simple sample that can be started on RZ/G3S board and prints
"Hello World" to the console.

This sample is useful to validate board configuration, such as DIP Switch settings and
Loaders.

.. _Starting on CM33 core using Jlink:

Starting on CM33 core using Jlink
---------------------------------

Before starting :ref:`hello_world` sample on the board the following preparations should be made:

- Confirm that all switches are set according to :ref:`rz_g3s_con` section.
- Turn on the board.
- Connect to the CA55 and CM33 console (PMOD1_3A SER0_UART) using minicom (:ref:`Linux minicom terminal`).
- Build Yocto images and flash the loaders to the eMMC (:ref:`Building yocto image`).
- Select eMMC boot mode in SW mode Switch and Reboot.

After all needed preparations please build :ref:`hello_world` sample using instructions
from :ref:`Debugging` and use the following command:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rz_g3s
   :goals: build

Start the debug session using command:

.. code-block:: bash

    west debug

The result will be the following:

.. code-block:: console

    Copyright (C) 2022 Free Software Foundation, Inc.
    License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
    This is free software: you are free to change and redistribute it.
    There is NO WARRANTY, to the extent permitted by law.
    Type "show copying" and "show warranty" for details.
    This GDB was configured as "--host=x86_64-build_pc-linux-gnu --target=arm-zephyr-eabi".
    Type "show configuration" for configuration details.
    For bug reporting instructions, please see:
    <https://github.com/zephyrproject-rtos/sdk-ng/issues>.
    Find the GDB manual and other documentation resources online at:
        <http://www.gnu.org/software/gdb/documentation/>.

    For help, type "help".
    Type "apropos word" to search for commands related to "word"...
    Reading symbols from build/rz_g3s/hello_world/zephyr/zephyr.elf...
    Remote debugging using :2331
    ...
    Resetting target
    --Type <RET> for more, q to quit, c to continue without paging--
    Loading section rom_start, size 0x7e0 lma 0x23000
    Loading section text, size 0x3990 lma 0x237e0
    Loading section .ARM.exidx, size 0x10 lma 0x27170
    Loading section initlevel, size 0x68 lma 0x27180
    Loading section device_area, size 0xb4 lma 0x271e8
    Loading section rodata, size 0xca4 lma 0x2729c
    Loading section datas, size 0x74 lma 0x27f40
    Loading section sw_isr_table, size 0xf40 lma 0x27fb4
    Loading section device_states, size 0x12 lma 0x28ef4
    Loading section .last_section, size 0x4 lma 0x2aadc
    Start address 0x00023f3c, load size 24330
    Transfer rate: 19 KB/sec, 2433 bytes/write.
    (gdb)

Then type `c` and hit enter.

Afterwards, please check the PMOD1_3A SER0_UART console. You should see the following output:

.. code-block:: console

   *** Booting Zephyr OS build v3.5.0-rc2-370-g5f6ce67b2d9f ***
   Hello World! rz_g3s

Starting on CM33_FPU core using Jlink
-------------------------------------

``west debug`` command can be used to start :ref:`hello_world` sample.

Please follow recommendations from :ref:`Starting on CM33 core using Jlink` with the following changes:

* Please connect to SER1_UART instead of PMOD1_3A SER0_UART using minicom (:ref:`rz_g3s_con`)

After all needed preparations please build :ref:`hello_world` sample using instructions
from :ref:`Debugging` and use the following command:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rz_g3s_fpu
   :goals: build

After that please follow the steps to start debugging using ``west debug`` from :ref:`Starting on CM33 core using Jlink`

All results will be the same with one difference. The following output will appear on SER1_UART console:

.. code-block:: console

   *** Booting Zephyr OS build v3.5.0-rc2-370-g5f6ce67b2d9f ***
   Hello World! rz_g3s_fpu

Flashing to SPI
----------------

:ref:`hello_world` sample can be flashed to the target board SPI flash so it will be started on each reboot.

Please follow the instructions to flash :ref:`hello_world` sample to the target board:

- Confirm that all switches are set according to :ref:`rz_g3s_con` section.
- Turn on the board.
- Connect to the CA55 and CM33 console (PMOD1_3A SER0_UART) using minicom (:ref:`Linux minicom terminal`).
- Build Yocto images and flash the loaders to the SPI (:ref:`Building yocto image`).
- Select Select SPI boot mode in SW mode Switch and Reboot.

After all needed preparations please build :ref:`hello_world` sample using instructions
from :ref:`Debugging` and use the following command:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rz_g3s
   :goals: build

As the result `zephyr.srec` file will be created in `build/zephyr` directory. Srec name is the same as `zephyr.bin` file
so in most cases it will have name `zephyr.srec`

`zephyr.srec` file can be flashed to the target board. Please follow instructions in :ref:`Flashing on qSPI`.

After flashing please Reboot the board and the following output will appear on PMOD1_3A SER0_UART console:

.. code-block:: console

   *** Booting Zephyr OS build v3.5.0-rc2-370-g5f6ce67b2d9f ***
   Hello World! rz_g3s

Flash to eMMC
-------------

:ref:`hello_world` sample can be flashed to the target board eMMC flash so it will be started on each reboot.

Please follow the instructions to flash :ref:`hello_world` sample to the target board:

- Confirm that all switches are set according to :ref:`rz_g3s_con` section.
- Turn on the board.
- Connect to the CA55 and CM33 console (PMOD1_3A SER0_UART) using minicom (:ref:`Linux minicom terminal`).
- Build Yocto images and flash the loaders to the eMMC (:ref:`Building yocto image`).
- Select Select eMMC boot mode in SW mode Switch and Reboot.

After all needed preparations please build :ref:`hello_world` sample using instructions
from :ref:`Debugging` and use the following command:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rz_g3s
   :goals: build

As the result `zephyr.srec` file will be created in `build/zephyr` directory. Srec name is the same as `zephyr.bin` file
so in most cases it will have name `zephyr.srec`

`zephyr.srec` file can be flashed to the target board. Please follow instructions in :ref:`Flashing on eMMC`.

After flashing please Reboot the board and the following output will appear on PMOD1_3A SER0_UART console:

.. code-block:: console

   *** Booting Zephyr OS build v3.5.0-rc2-370-g5f6ce67b2d9f ***
   Hello World! rz_g3s

.. raw:: latex

    \newpage
