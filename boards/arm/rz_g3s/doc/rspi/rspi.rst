Renesas Serial Peripheral Interface (rSPI)
==========================================

rSPI overview
-------------

The Renesas RZ G3S Soc includes three independent Renesas serial peripheral interfaces (rSPI).
The rSPI module is capable of full-duplex synchronous serial communication.

Refer to "Renesas Serial Peripheral Interface" section in "Renesas RZ/G3S Group User’s Manual: Hardware"

rSPI driver overview
--------------------

Zephyr RZ/G3S Renesas Serial Peripheral Interface (rSPI) driver supports:

• 8/16bit transfers

Zephyr RZ/G3S Renesas rSPI driver provides Zephyr :ref:`spi_api` System interface implementation.

The SPI subsystem is **not** enabled by default in ``rz_g3s_defconfig``. To enable Zephyr
SPI functionality and RZ G3S rSPI driver below Kconfig options have to be enabled:

.. code-block:: text

    CONFIG_SPI=y
    /* automatically enabled */
    CONFIG_SPI_RZA2M=y

The RZ G3S rSPI driver code can be found at:

.. code-block:: text

    drivers/spi/spi_rza2m.c

rSPI testing
------------

tests/drivers/spi/spi_loopback
``````````````````````````````

Zephyr RZ/G3S Renesas Serial Peripheral Interface (rSPI) driver can be tested by
using generic **spi_loopback** test. Use below command to build SPI **spi_loopback** sample application:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/spi/spi_loopback

Once **spi_loopback** is loaded it will provide console output showing the test execution process:

.. code-block:: console

    I: spi@400aa000:"Init done fck:100000000Hz"
    *** Booting Zephyr OS build v3.5.0-rc2-364-g2f7499219d0b ***
    Running TESTSUITE spi_loopback
    ===================================================================
    START - test_spi_loopback
    I: SPI test on buffers TX/RX 0x20051260/0x20051240, frame size = 8, DMA enabled (without CONFIG_NOCACHE_MEMORY)
    I: SPI test slow config
    I: Start complete multiple
    I: Passed
    I: Start complete loop
    I: Passed
    I: Start null tx
    I: Passed
    I: Start half start
    I: Passed
    I: Start half end
    I: Passed
    I: Start every 4
    I: Passed
    I: Start rx bigger than tx
    I: Passed
    I: Start async call
    I: Passed
    I: SPI test fast config
    I: Start complete multiple
    I: Passed
    I: Start complete loop
    I: Passed
    I: Start null tx
    I: Passed
    I: Start half start
    I: Passed
    I: Start half end
    I: Passed
    I: Start every 4
    I: Passed
    I: Start rx bigger than tx
    I: Passed
    I: Start async call
    I: Passed
    I: Start complete loop
    I: Passed
    I: Start complete loop
    I: Passed
    I: All tx/rx passed
     PASS - test_spi_loopback in 0.078 seconds
    ===================================================================
    TESTSUITE spi_loopback succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [spi_loopback]: pass = 1, fail = 0, skip = 0, total = 1 duration = 0.078 seconds
     - PASS - [spi_loopback.test_spi_loopback] duration = 0.078 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

.. raw:: latex

    \newpage
