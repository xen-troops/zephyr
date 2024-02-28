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

SER0 usage
``````````

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

Limitations:
````````````

* No DMA support implemented.
* uart_async_api test should be implemented once UART will support DMA.

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

.. raw:: latex

    \newpage
