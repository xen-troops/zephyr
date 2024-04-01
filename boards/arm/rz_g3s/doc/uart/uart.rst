UART Serial Communications Interface with FIFO (SCIFA)
======================================================

UART overview
-------------

The Renesas RZ G3S Soc has five channels of serial communication interface (SCIFA)
with FIFO that support both asynchronous and clock synchronous serial communication.
The SCIFA has 16-stage FIFO buffers for transmission and reception,
respectively, for each channel to perform efficient high-speed continuous communication.

Refer to "Serial Communications Interface with FIFO (SCIFA)" section
in "Renesas RZ/G3S Group User’s Manual: Hardware"

There are 2 UART ports supported from Zephyr on RZ/G3S SMARC Evaluation Board Kit board:

* SER0 from PMOD1_3A which is currently used to access Console of M33 cores;
* SER1 from SER1_UART which can be use as M33_FPU console.

.. _ser0_usage:

SER0 usage
``````````

.. figure:: ../img/rzg3s_ser0.jpg
   :align: center
   :height: 300px

SER0 can be used with USB/UART adapter which support 3.3V level.
SER0 connector corresponds to SCIF ch1 peripheral on the board.
You can configure that peripheral in device-tree in the following way:

.. code-block:: dts

    &scif1 {
            current-speed = <115200>;
            pinctrl-0 = <&scif1_pins>;
            pinctrl-names = "default";
            status = "okay";
    };

.. _ser1_usage:

SER1 usage
``````````
.. image:: ../img/SER1.jpg
   :height: 250px
   :align: center

SER1 can be used with USB/UART adapter which support 1.8V level. Connect
appropriate adapter according to the picture. SER1 connector corresponds to
SCIF ch3 peripheral on the board. You can configure that peripheral in
device-tree overlay file in the following way:

.. code-block:: dts

    &scif3 {
            current-speed = <115200>;
            pinctrl-0 = <&scif3_pins>;
            pinctrl-names = "default";
            status = "okay";
    };

UART driver overview
--------------------

The Renesas SCIF driver provides Zephyr :ref:`uart_api` System interface implementation.

The Zephyr SERIAL subsystem and serial console are enabled by default in ``rz_g3s_defconfig``,
which automatically enables RZ G3S SCIF driver if corresponding DT node is enabled.

.. code-block:: text

    # Enable UART driver
    CONFIG_SERIAL=y
    # Enable console
    CONFIG_CONSOLE=y
    CONFIG_UART_CONSOLE=y
    /* automatically enabled */
    CONFIG_UART_SCIF=y

The RZ G3S SCIF driver code can be found at

.. code-block:: text

    drivers/serial/uart_scif.c

UART testing
------------

tests/drivers/uart/uart_basic_api
`````````````````````````````````

To build **uart_basic_api** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/uart/uart_basic_api

The **uart_basic_api** test will produce below console output when executed:

.. code-block:: console

   *** Booting Zephyr OS build zephyr-v3.3.0-10465-gb8ad06c6248d ***
   Running TESTSUITE uart_basic_api
   ===================================================================
   START - test_uart_config_get
   This is a configure_g PASS - test_uart_config_get in 0.003 seconds
   ===================================================================
   START - test_uart_co PASS - test_uart_configure in 0.001 seconds
   ===================================================================
   START - test_uart_fifo_fill
   This is a FIFO test.
    PASS - test_uart_fifo_fill in 0.501 seconds
   ===================================================================
   START - test_uart_fifo_read
   Please send characters to serial console
    PASS - test_uart_fifo_read in 4.425 seconds
   ===================================================================
   START - test_uart_poll_in
   Please send characters to serial console
    PASS - test_uart_poll_in in 1.853 seconds
   ===================================================================
   START - test_uart_poll_out
   This is a POLL test.
    PASS - test_uart_poll_out in 0.002 seconds
   ===================================================================
   TESTSUITE uart_basic_api succeeded
   Running TESTSUITE uart_basic_api_pending
   ===================================================================
   START - test_uart_pending
   Please send characters to serial console
   w PASS - test_uart_pending in 0.801 seconds
   ===================================================================
   TESTSUITE uart_basic_api_pending succeeded

   ------ TESTSUITE SUMMARY START ------

   SUITE PASS - 100.00% [uart_basic_api]: pass = 6, fail = 0, skip = 0, total = 6 duration = 6.785 seconds
    - PASS - [uart_basic_api.test_uart_config_get] duration = 0.003 seconds
    - PASS - [uart_basic_api.test_uart_configure] duration = 0.001 seconds
    - PASS - [uart_basic_api.test_uart_fifo_fill] duration = 0.501 seconds
    - PASS - [uart_basic_api.test_uart_fifo_read] duration = 4.425 seconds
    - PASS - [uart_basic_api.test_uart_poll_in] duration = 1.853 seconds
    - PASS - [uart_baesic_api.test_uart_poll_out] duration = 0.002 seconds

   SUITE PASS - 100.00% [uart_basic_api_pending]: pass = 1, fail = 0, skip = 0, total = 1 duration = 0.801 s
    - PASS - [uart_basic_api_pending.test_uart_pending] duration = 0.801 seconds

   ------ TESTSUITE SUMMARY END ------

   ===================================================================
   PROJECT EXECUTION SUCCESSFUL

tests/drivers/uart/uart_async_api
`````````````````````````````````
Async api uses DMA for communication. This test is using SCIF loopback test mode to run the **uart_async_api** test.
To build **uart_async_api** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/uart/uart_async_api

The **uart_async_api** test will produce below console output when executed:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-372-ge216e1e5896c ***
    Running TESTSUITE uart_async_chain_read
    ===================================================================
    START - test_chained_read
     SKIP - test_chained_read in 0.002 seconds
    ===================================================================
    TESTSUITE uart_async_chain_read succeeded
    Running TESTSUITE uart_async_chain_write
    ===================================================================
    START - test_chained_write
     PASS - test_chained_write in 0.004 seconds
    ===================================================================
    TESTSUITE uart_async_chain_write succeeded
    Running TESTSUITE uart_async_double_buf
    ===================================================================
    START - test_double_buffer
     SKIP - test_double_buffer in 0.002 seconds
    ===================================================================
    TESTSUITE uart_async_double_buf succeeded
    Running TESTSUITE uart_async_long_buf
    ===================================================================
    START - test_long_buffers
     SKIP - test_long_buffers in 0.002 seconds
    ===================================================================
    TESTSUITE uart_async_long_buf succeeded
    Running TESTSUITE uart_async_multi_rx
    ===================================================================
    START - test_multiple_rx_enable
     SKIP - test_multiple_rx_enable in 0.002 seconds
    ===================================================================
    TESTSUITE uart_async_multi_rx succeeded
    Running TESTSUITE uart_async_read_abort
    ===================================================================
    START - test_read_abort
     PASS - test_read_abort in 1.121 seconds
    ===================================================================
    TESTSUITE uart_async_read_abort succeeded
    Running TESTSUITE uart_async_single_read
    ===================================================================
    START - test_single_read
     PASS - test_single_read in 0.354 seconds
    ===================================================================
    TESTSUITE uart_async_single_read succeeded
    Running TESTSUITE uart_async_timeout
    ===================================================================
    START - test_forever_timeout
     PASS - test_forever_timeout in 3.003 seconds
    ===================================================================
    TESTSUITE uart_async_timeout succeeded
    Running TESTSUITE uart_async_write_abort
    ===================================================================
    START - test_write_abort
     SKIP - test_write_abort in 0.002 seconds
    ===================================================================
    TESTSUITE uart_async_write_abort succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE SKIP -   0.00% [uart_async_chain_read]: pass = 0, fail = 0, skip = 1, total = 1 duration = 0.002 ss
     - SKIP - [uart_async_chain_read.test_chained_read] duration = 0.002 seconds

    SUITE PASS - 100.00% [uart_async_chain_write]: pass = 1, fail = 0, skip = 0, total = 1 duration = 0.004 s
     - PASS - [uart_async_chain_write.test_chained_write] duration = 0.004 seconds

    SUITE SKIP -   0.00% [uart_async_double_buf]: pass = 0, fail = 0, skip = 1, total = 1 duration = 0.002 ss
     - SKIP - [uart_async_double_buf.test_double_buffer] duration = 0.002 seconds

    SUITE SKIP -   0.00% [uart_async_long_buf]: pass = 0, fail = 0, skip = 1, total = 1 duration = 0.002 secs
     - SKIP - [uart_async_long_buf.test_long_buffers] duration = 0.002 seconds

    SUITE SKIP -   0.00% [uart_async_multi_rx]: pass = 0, fail = 0, skip = 1, total = 1 duration = 0.002 secs
     - SKIP - [uart_async_multi_rx.test_multiple_rx_enable] duration = 0.002 seconds

    SUITE PASS - 100.00% [uart_async_read_abort]: pass = 1, fail = 0, skip = 0, total = 1 duration = 1.121 ss
     - PASS - [uart_async_read_abort.test_read_abort] duration = 1.121 seconds

    SUITE PASS - 100.00% [uart_async_single_read]: pass = 1, fail = 0, skip = 0, total = 1 duration = 0.354 s
     - PASS - [uart_async_single_read.test_single_read] duration = 0.354 seconds

    SUITE PASS - 100.00% [uart_async_timeout]: pass = 1, fail = 0, skip = 0, total = 1 duration = 3.003 secos
     - PASS - [uart_async_timeout.test_forever_timeout] duration = 3.003 seconds

    SUITE SKIP -   0.00% [uart_async_write_abort]: pass = 0, fail = 0, skip = 1, total = 1 duration = 0.002 s
     - SKIP - [uart_async_write_abort.test_write_abort] duration = 0.002 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

.. raw:: latex

    \newpage
