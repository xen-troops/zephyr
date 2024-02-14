Direct Memory Access (DMA) Controller Support
=============================================

DMA overview
------------

The direct memory access controller can be used in place of the CPU to perform high-speed transfers between external
memory, on-chip memory, memory-mapped external devices, and on-chip peripheral modules.
This module has controllers that handles secure and non-secure access.
Do not assign the same DMA transfer request to secure access and non- secure access, respectively.

DMA driver overview
-------------------

Zephyr RZ/G3S Renesas DMA driver provides Zephyr :ref:`dma_api` System interface implementation.

The DMA subsystem is **not** enabled by default in ``rz_g3s_defconfig``. To enable Zephyr
DMA functionality and RZ G3S DMA driver below Kconfig options have to be enabled:

.. code-block:: text

    CONFIG_DMA=y
    /* automatically enabled */
    CONFIG_DMA_RZA2=y

The RZ G3S DMA driver code can be found at:

.. code-block:: text

    drivers/dma/dma_rza2.c

DMA testing
-----------

tests/drivers/dma/chan_blen_transfer
````````````````````````````````````

Zephyr RZ/G3S DMA driver can be tested by using **tests/drivers/dma/chan_blen_transfer** test.
To build **tests/drivers/dma/chan_blen_transfer** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/dma/chan_blen_transfer

Console output:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-364-g2f7499219d0b ***
    Running TESTSUITE dma_m2m
    ===================================================================
    START - test_test_dma0_m2m_chan0_burst16
    Preparing DMA Controller: Name=dma@41800000, Chan_ID=0, BURST_LEN=2
    Starting the transfer
    DMA transfer done
    It is harder to be kind than to be wise........
     PASS - test_test_dma0_m2m_chan0_burst16 in 2.014 seconds
    ===================================================================
    START - test_test_dma0_m2m_chan0_burst8
    Preparing DMA Controller: Name=dma@41800000, Chan_ID=0, BURST_LEN=1
    Starting the transfer
    DMA transfer done
    It is harder to be kind than to be wise........
     PASS - test_test_dma0_m2m_chan0_burst8 in 2.014 seconds
    ===================================================================
    START - test_test_dma0_m2m_chan1_burst16
    Preparing DMA Controller: Name=dma@41800000, Chan_ID=1, BURST_LEN=2
    Starting the transfer
    DMA transfer done
    It is harder to be kind than to be wise........
     PASS - test_test_dma0_m2m_chan1_burst16 in 2.014 seconds
    ===================================================================
    START - test_test_dma0_m2m_chan1_burst8
    Preparing DMA Controller: Name=dma@41800000, Chan_ID=1, BURST_LEN=1
    Starting the transfer
    DMA transfer done
    It is harder to be kind than to be wise........
     PASS - test_test_dma0_m2m_chan1_burst8 in 2.014 seconds
    ===================================================================
    TESTSUITE dma_m2m succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [dma_m2m]: pass = 4, fail = 0, skip = 0, total = 4 duration = 8.056 seconds
     - PASS - [dma_m2m.test_test_dma0_m2m_chan0_burst16] duration = 2.014 seconds
     - PASS - [dma_m2m.test_test_dma0_m2m_chan0_burst8] duration = 2.014 seconds
     - PASS - [dma_m2m.test_test_dma0_m2m_chan1_burst16] duration = 2.014 seconds
     - PASS - [dma_m2m.test_test_dma0_m2m_chan1_burst8] duration = 2.014 seconds

    ------ TESTSUITE SUMMARY END ------

tests/drivers/dma/loop_transfer
```````````````````````````````

Zephyr RZ/G3S DMA driver can be tested by using **tests/drivers/dma/loop_transfer** test.
To build **tests/drivers/dma/loop_transfer** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/dma/loop_transfer

Console output:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-364-g2f7499219d0b ***
    Running TESTSUITE dma_m2m_loop
    ===================================================================
    START - test_test_dma0_m2m_loop
    DMA memory to memory transfer started
    Preparing DMA Controller: dma@41800000
    Starting the transfer on channel 0 and waiting for 1 second
    Each RX buffer should contain the full TX buffer string.
    RX data Loop 0
    RX data Loop 1
    RX data Loop 2
    RX data Loop 3
    Finished DMA: dma@41800000
     PASS - test_test_dma0_m2m_loop in 0.309 seconds
    ===================================================================
    START - test_test_dma0_m2m_loop_repeated_start_stop
    DMA memory to memory transfer started
    Preparing DMA Controller
    Starting the transfer on channel 1 and waiting for 1 second
    Each RX buffer should contain the full TX buffer string.
    RX data Loop 0
    RX data Loop 1
    RX data Loop 2
    RX data Loop 3
    Finished: DMA
     PASS - test_test_dma0_m2m_loop_repeated_start_stop in 0.304 seconds
    ===================================================================
    START - test_test_dma0_m2m_loop_suspend_resume
    DMA memory to memory transfer started
    Preparing DMA Controller: dma@41800000
    Starting the transfer on channel 2 and waiting for 1 second
    suspended after 0 transfers occurred
    resuming after 0 transfers occurred
    Resumed transfers
    Transfer count 4
    Each RX buffer should contain the full TX buffer string.
    RX data Loop 0
    RX data Loop 1
    RX data Loop 2
    RX data Loop 3
    Finished DMA: dma@41800000
     PASS - test_test_dma0_m2m_loop_suspend_resume in 0.568 seconds
    ===================================================================
    TESTSUITE dma_m2m_loop succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [dma_m2m_loop]: pass = 3, fail = 0, skip = 0, total = 3 duration = 1.181 seconds
     - PASS - [dma_m2m_loop.test_test_dma0_m2m_loop] duration = 0.309 seconds
     - PASS - [dma_m2m_loop.test_test_dma0_m2m_loop_repeated_start_stop] duration = 0.304 seconds
     - PASS - [dma_m2m_loop.test_test_dma0_m2m_loop_suspend_resume] duration = 0.568 seconds

    ------ TESTSUITE SUMMARY END ------

tests/drivers/dma/scatter_gather
````````````````````````````````

Zephyr RZ/G3S DMA driver can be tested by using **tests/drivers/dma/scatter_gather** test.
To build **tests/drivers/dma/scatter_gather** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/dma/scatter_gather

Console output:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-364-gd807a730bca8 ***
    Running TESTSUITE dma_m2m_sg
    ===================================================================
    START - test_dma_m2m_sg
    DMA memory to memory transfer started
    Preparing DMA Controller
    dma block 0 block_size 8192, source addr 200590a0, dest addr 200510a0
    set next block pointer to 0x2005b2f4
    dma block 1 block_size 8192, source addr 200590a0, dest addr 200530a0
    set next block pointer to 0x2005b314
    dma block 2 block_size 8192, source addr 200590a0, dest addr 200550a0
    set next block pointer to 0x2005b334
    dma block 3 block_size 8192, source addr 200590a0, dest addr 200570a0
    Configuring the scatter-gather transfer on channel 0
    Starting the transfer on channel 0 and waiting completion
    giving xfer_sem
    Verify RX buffer should contain the full TX buffer string.
    rx_data[0]
    rx_data[1]
    rx_data[2]
    rx_data[3]
    Finished: DMA Scatter-Gather
     PASS - test_dma_m2m_sg in 0.101 seconds
    ===================================================================
    TESTSUITE dma_m2m_sg succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [dma_m2m_sg]: pass = 1, fail = 0, skip = 0, total = 1 duration = 0.101 seconds
     - PASS - [dma_m2m_sg.test_dma_m2m_sg] duration = 0.101 seconds

    ------ TESTSUITE SUMMARY END ------

.. raw:: latex

    \newpage
