rSPI Support
============
Zephyr RZ/G3S Renesas Serial Peripheral Interface (rSPI) driver supports:

• 8/16bit transfers

Async and DMA transfers are not supported at the moment.

Zephyr Kconfig options needed for RZ/G3S SPI driver testing:

.. code-block::

    CONFIG_SPI=y

Zephyr RZ/G3S Renesas Serial Peripheral Interface (rSPI) driver can be tested by
using generic spi loopback test. Use below command to build SPI spi loopback sample application:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/spi/spi_loopback

Once spi loopback is loaded it will provide console output showing the test execution process:

.. code-block:: bash

    I: spi@400aa000:"Init done fck:100000000Hz"
    *** Booting Zephyr OS build v3.5.0-rc2-228-g35075fc31e86 ***
    Running TESTSUITE spi_loopback
    ===================================================================
    START - test_spi_loopback
    I: SPI test on buffers TX/RX 0x2cce0/0x2ccc0, frame size = 8
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
    I: Start complete loop
    I: Passed
    I: Start complete loop
    I: Passed
    I: All tx/rx passed
     PASS - test_spi_loopback in 0.072 seconds
    ===================================================================
    TESTSUITE spi_loopback succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [spi_loopback]: pass = 1, fail = 0, skip = 0, total = 1 dus
     - PASS - [spi_loopback.test_spi_loopback] duration = 0.072 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL
