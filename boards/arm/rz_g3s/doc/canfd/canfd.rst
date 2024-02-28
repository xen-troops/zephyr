CAN-FD Interface (CAN-FD)
=========================

CAN-FD overview
---------------

Renesas RZ G3S CAN-FD controller has 2-channels that complies with ISO 11898-1 (2015) Standards.
It transmits and receives both formats of messages: the standard identifier (11 bits)
and extended ID (29 bits). The following major HW features are supported.

* CAN-FD mode
* Nominal bit rate: Max. 1 Mbps
* Data bit rate: Max. 8 Mbps
* CAN-FD ISO 11898-1 (CD2015) compliant;
* number of receive rules (0 to 128) for each channel;
* transmit buffers up to 32 per channel;
* receive buffers 128 total, up to 64 buffers per channel;
* Transmitter Delay Compensation;
* RX/TX message timestamping;
* auto-bus recovery;
* loopback, listen-only and one-shot test modes

Refer to "CAN-FD Interface (CAN-FD)" section in "Renesas RZ/G3S Group User’s Manual: Hardware"

CAN-FD driver overview
----------------------

The RZ G3S CAN-FD driver driver provides Zephyr :ref:`can_api` System interface implementation
and supports below features:

* 2 channels supported;
* number of receive AFL rules (0 to 128) for each channel;
* transmit buffers up to 32 per channel;
* Transmission Priority by CAN Frame ID;
* receive buffers 128 total, up to 64 buffers per channel;
* CAN-FD and Classic frames processing;
* loopback, listen-only and one-shot test modes;
* Transmitter Delay Compensation;
* RX message timestamping;
* auto bus recovery;
* Zephyr CAN statistic and RZ G3S CAN-FD driver statistic;
* Frame filering by ID, IDE, RTR/Data with masking in HW;
* Frame filering by FDF in SW.

Unsupported HW  features:

* Gateway function
* Message buffer number Transmission Priority
* DLC filtering
* Mirror mode
* Common FIFO is not used
* TXQ is not used
* TX message timestamping;
* Timestamp source selection
* Interval transmission function
* Global test modes
* Power management
* Bus traffic measurement

The CAN-FD subsystem is **not** enabled by default in rz_g3s_defconfig. To enable Zephyr
CAN functionality and RZ G3S CAN-FD driver below Kconfig options has to be enabled:

.. code-block:: text

    CONFIG_CAN=y
    CONFIG_CAN_FD_MODE=y
    /*  RZ G3S CAN-FD driver automatically enabled if corresponding DT node is enabled */
    CONFIG_CAN_RZGS_FD=y
    /* optional */
    CONFIG_CAN_AUTO_BUS_OFF_RECOVERY=y
    /* optional */
    ONFIG_STATS=y
    /* optional */
    CONFIG_CAN_STATS=y
    /* optional */
    CONFIG_CAN_RX_TIMESTAMP=y

The RZ G3S CAN-FD driver code can be found at:

.. code-block:: text

    drivers/can/can_rzg3s_fd.c
    drivers/can/can_rzg3s_fd.h
    drivers/can/Kconfig.rzg3s

The RZ G3S CAN-FD driver Kconfig parameters are listed below:

.. raw:: latex

	\begin{scriptsize}

+---------------------------+--------+------------------------------------------------------------------------+
| option                    | default| comment                                                                |
+===========================+========+========================================================================+
| CONFIG_CAN_MAX_FILTER     | 32     | Number of AFL entries per each CAN-FD channel.                         |
|                           |        | All CAN-FD channels will use the same value.                           |
+---------------------------+--------+------------------------------------------------------------------------+
| CONFIG_CAN_RZGS_FD_MAX_RX | 32     | Maximum number of RX FIFO buffers. Used to configure CFDRFCCn.RFDC.    |
|                           |        | All CAN-FD channels will use the same value.                           |
+---------------------------+--------+------------------------------------------------------------------------+
| CONFIG_CAN_RZGS_FD_MAX_RX | 64     | Maximum size of RX Payload Data Size. Used to configure CFDRFCCn.RFPLS.|
| _DATA_SIZE                |        | All CAN-FD channels will use the same value.                           |
+---------------------------+--------+------------------------------------------------------------------------+
| CONFIG_CAN_RZGS_FD_MAX_TX | 32     | Maximum number of TX message buffers per each CAN-FD channel.          |
|                           |        | All CAN-FD channels will use the same value.                           |
+---------------------------+--------+------------------------------------------------------------------------+

.. raw:: latex

	\end{scriptsize}


The RZ G3S CAN-FD DT node is defined in (dts/arm/renesas/rz/r9a08g045.dtsi) as:

.. code-block:: dts

	canfd: can@400c0000 {
		compatible = "renesas,r9a08g045-canfd", "renesas,rzg3s-canfd";
		reg = <0x400c0000 DT_SIZE_K(128)>;
		interrupts = <373 0>, <374 0>;
		interrupt-names = "g_err", "g_recc";
		clocks = <&cpg CPG_MOD R9A08G045_CANFD_CLK_RAM>,
				 <&cpg CPG_MOD R9A08G045_CANFD_PCLK>,
				 <&can_clk>;
		clock-names = "fck", "canfd", "can_clk";
		resets = <&cpg_rctl R9A08G045_CANFD_RSTP_N>,
				 <&cpg_rctl R9A08G045_CANFD_RSTC_N>;
		reset-names = "rstp_n", "rstc_n";
		#address-cells = <1>;
		#size-cells = <0>;
		status = "disabled";

		canfd0: channel@0 {
			reg = <0x0>;
			interrupts = <375 0>, <377 0>, <379 0>;
			interrupt-names = "ch_rec", "ch_err", "ch_trx";
			status = "disabled";
		};

		canfd1: channel@1 {
			reg = <0x1>;
			interrupts = <376 0>, <378 0>, <380 0>;
			interrupt-names = "ch_rec", "ch_err", "ch_trx";
			status = "disabled";
		};
	};

Each RZ G3S CAN-FD channel implemented as Zephyr CAN device and can be used independently.
The RZ G3S CAN-FD has to be enabled and configured in board DTS files to become available.
The RZ G3S CAN-FD configuration example can be found at:

.. code-block:: text

	boards/arm/rz_g3s/rz_g3s_canfd.dtsi
	snippets/rz-g3s-canfd0-test/rz-g3s-canfd0-test.overlay
	snippets/rz-g3s-canfd0-test/rz-g3s-canfd1-test.overlay

The RZ G3S CAN-FD channel's DT parameters are listed below:

.. raw:: latex

	\begin{scriptsize}

+-----------------------------------+----------------+---------------------------------------------------------------------------+
| Name                              | Default        | Comment                                                                   |
+===================================+================+===========================================================================+
| reg                               | 0|1            | Should be 0 for channel0 and 1 for channel1                               |
+-----------------------------------+----------------+---------------------------------------------------------------------------+
| bus-speed                         | board specific | Nominal Initial bitrate in bit/s                                          |
+-----------------------------------+----------------+---------------------------------------------------------------------------+
| sample-point                      | board specific | Initial sample point in per mille (e.g. 875 equals 87.5%).                |
|                                   |                | Used to calculate and configure Nominal Bit Rate Configuration (CFDCnNCFG)|
|                                   |                | parameters.                                                               |
+-----------------------------------+----------------+---------------------------------------------------------------------------+
| bus-speed-data                    | board specific | InitialInitial data phase bitrate in bit/s.                               |
+-----------------------------------+----------------+---------------------------------------------------------------------------+
| sample-point-data                 | board specific | Initial data phase sample point in per mille (e.g. 875 equals 87.5%).     |
|                                   |                | Used to calculate and configure Data Bit Rate Configuration (CFDCnDCFG)   |
|                                   |                | parameters.                                                               |
+-----------------------------------+----------------+---------------------------------------------------------------------------+
| phys                              | board specific,| Actively controlled CAN transceiver.                                      |
|                                   | optional       |                                                                           |
+-----------------------------------+----------------+---------------------------------------------------------------------------+
| can-transceiver                   | board specific,| Passive CAN transceiver. The child node must be named "can-transceiver".  |
|                                   | optional       |                                                                           |
+-----------------------------------+----------------+---------------------------------------------------------------------------+
| tx-delay-comp-offset              | 0,             | Transceiver Delay Compensation Offset, used as CFDCnFDCFG.TDCO.           |
|                                   | board specific,|                                                                           |
|                                   | optional       | Transceiver Delay Compensation is enabled if this parameter != 0.         |
+-----------------------------------+----------------+---------------------------------------------------------------------------+
| renesas,tx-delay-comp-offset-only | false,         | Only Transceiver Delay Compensation Offset is used when specified.        |
|                                   | board specific,| Used to set CFDCnFDCFG.TDCOC=1.                                           |
|                                   | optional       | Valid when tx-delay-comp-offset != 0.                                     |
+-----------------------------------+----------------+---------------------------------------------------------------------------+

.. raw:: latex

	\end{scriptsize}

CAN-FD TX path
``````````````
The RZ G3S CAN-FD driver uses TX Message Buffers (TMB) for sending CAN Frames.
For each TX CAN Frame the optional callback is saved and called from TX ("ch_trx") IRQ on TX Frame sending completion.
The CAN Frames are sent using CAN ID priority as priority, the ID priority complies with the CAN bus arbitration rule
(as specified in ISO 11898-1 specification).

.. code-block:: text

     Zephyr    | RZ G3S CAN-FD driver
               |                               +----------------------------------+
               |                               |RZ G3S CAN-FD IP                  |
               |                               |                                  |
      can_send |           +-------------+     | +-------------+                  |
     ----------+-----+-----> CAN Frame   +------->             |    +-----------+ |
               |     |     |             |     | |  TMB_0      +---->  TX       | |
               |     |     +-------------+     | +-------------+    |           | |
               |     |     |tx_callback  |     | |  ...        |    |           +----->
               |     +----->             |     | |             +---->           | |
               |           +----------+--+     | +-------------+    |           | |
               |                      |        | |             |    |           | |
               |                      |        | |  TMB_X      +---->           | |
               |                      |        | +-------------+    |           | |
               |                      |        |                    |           | |
               |                      |        |                    +----+------+ |
               |                      v        |                         |        |
  tx_callback()|                   +---+       | TX ("ch_trx") IRQ       |        |
     <---------+-------------------+ + |<--------------------------------+        |
               |                   +---+       |                                  |
                                               +----------------------------------+

Transmission modes:

* Regular (CAN_MODE_ONE_SHOT is not set) - When arbitration is lost or an error occurs,
  message transmission will be attempted further if no transmission abort request is set for this TMB.
* One-shot (CAN_MODE_ONE_SHOT is not set) - CAM-FD attempts to transmit a message only once and
  additional message transmission will not be attempted in this case.

In case of the listen-only (CAN_MODE_LISTENONLY) mode - the TX function is prohibited
and all TX requests will be rejected by driver.

CAN-FD RX path
``````````````
The RZ G3S CAN-FD driver uses one RX FIFO per channel (channel 0 uses RX FIFO0, channel 1 uses RX FIFO1)
for receiving CAN Frames. Maximum number of stored CAN frames defined by CONFIG_CAN_RZGS_FD_MAX_RX Kconfig option.
The CAN Frames are dropped if RX FIFO is full.
The CAN-FD generates Global interrupt for successful reception into any of the RX FIFO buffers (RX "g_recc" IRQ).

CAN frames are stored in RX FIFO if they passes Acceptance Filtering check according to configured AFL Entries.
The RZ G3S CAN-FD stores the "Global Acceptance Filter List Pointer", corresponding to AFL Entry number for
which Acceptance Filtering check has passed, with received CAN frame. This "Global Acceptance Filter List Pointer"
then retrieved from RX FIFO Access Message Buffer Component as "RX FIFO Buffer Pointer Field" CFDRFFDSTSn.CFDRFPTR,
and it is used to identify AFL Entry and call .rx_callback() assigned to it.

.. code-block:: text

              Zephyr | RZ G3S CAN-FD driver              +-----------------------------------+
                     |                                   |RZ G3S CAN-FD IP                   |
                     |                                   | +--------------+                  |
                     |                                   | |RX FIFO       |                  |
                     |                  RX "g_recc" IRQ  | | +----------+ |     +----------+ |
                     |  +------------------------------------|          | |     |RX        | |
                     |  |   CFDRFPTR == GAFLPTR          | | +----------+ |     |          | |
                     |  |                                | | |          | |     |          | |
                     |  |                                | | +----------+ |     |          | |
                     |  |    +----------------------+    | | |          | |     |          <-----
                     |  |    | AFL                  |    | | +-----^----+ |     |          | |
                     |  |    |  +------------+      |    | |       |      |     |          | |
                     |  +------>|AFL entry 0 |      |    | +-------|------+     |          | |
                     |       |  +--+---------+--+   |    |         |            |          | |
                     |       |     |GAFLPTR     |   |    |         +------------+          | |
        rx_callback()|       |     +------------+   |    |                      +----^-----+ |
               <-------------------+rx_callback |   |    |                           |       |
                     |       |  +--+---------+--+   |    |       +---------------+   |       |
                     |       |  |AFL entry X |      |    |       | AFL           |   |       |
                     |       |  +--+---------+--+   +------------>               +---+       |
  can_add_rx_filter()|       |     |GAFLPTR     |   |    |       |               |           |
               -------------->     +------------+   |    |       +---------------+           |
                     |       |     |rx_callback |   |    |                                   |
                     |       |     +------------+   |    |                                   |
                     |       |                      |    |                                   |
                     |       +----------------------+    +-----------------------------------+

CAN-FD statistic
````````````````
The RZ G3S CAN-FD driver supports set of statistic groups, which names are listed below:

* global - "can_rzg3s_global"
* per CAN channel - "can_channel@X", "can_rzg3s_chX" (X is channel id 0,1).

The statistic is available when enabled by Kconfig option `CONFIG_CAN_STATS`.

The "can_channel@X" group provides per-channel statistic information as described in
Zephyr :ref:`can_api` System interface.

The "can_rzg3s_chX" group is RZ G3S CAN-FD driver specific and provides per-channel statistic
information not covered by "can_channel@X" group.

The "can_rzg3s_global" group provides Global statistic information collected by RZ G3S CAN-FD driver.

The RZ G3S CAN-FD driver statistic output provided below:

.. code-block:: console

    Stats Group can_rzg3s_global (hdr addr: 0x20052784)
        ecc_error_ch0 (offset: 24, addr: 0x2005279c): 0
        ecc_error_ch1 (offset: 28, addr: 0x200527a0): 0
        canfd_msg_payload_overflow (offset: 32, addr: 0x200527a4): 0
        rx_msg_lost_rfifo0 (offset: 36, addr: 0x200527a8): 0
        rx_msg_lost_rfifo1 (offset: 40, addr: 0x200527ac): 0
    Stats Group can_channel@0 (hdr addr: 0x200510e8)
        bit0_error (offset: 24, addr: 0x20051100): 0
        bit1_error (offset: 28, addr: 0x20051104): 0
        stuff_error (offset: 32, addr: 0x20051108): 0
        crc_error (offset: 36, addr: 0x2005110c): 0
        form_error (offset: 40, addr: 0x20051110): 0
        ack_error (offset: 44, addr: 0x20051114): 0
        rx_overrun (offset: 48, addr: 0x20051118): 0
    Stats Group can_rzg3s_ch0 (hdr addr: 0x20052720)
        arbitration_lost (offset: 24, addr: 0x20052738): 0
        bus_lock (offset: 28, addr: 0x2005273c): 0
        error_active_state (offset: 32, addr: 0x20052740): 1
        error_warning_state (offset: 36, addr: 0x20052744): 0
        error_passive_state (offset: 40, addr: 0x20052748): 0
        bus_off_state (offset: 44, addr: 0x2005274c): 0
        bus_off_recovery (offset: 48, addr: 0x20052750): 0
        bus_off_recovery_manual (offset: 52, addr: 0x20052754): 0
        tdc_violation (offset: 56, addr: 0x20052758): 0
    Stats Group can_channel@1 (hdr addr: 0x200510b0)
        bit0_error (offset: 24, addr: 0x200510c8): 0
        bit1_error (offset: 28, addr: 0x200510cc): 0
        stuff_error (offset: 32, addr: 0x200510d0): 0
        crc_error (offset: 36, addr: 0x200510d4): 0
        form_error (offset: 40, addr: 0x200510d8): 0
        ack_error (offset: 44, addr: 0x200510dc): 0
        rx_overrun (offset: 48, addr: 0x200510e0): 0
    Stats Group can_rzg3s_ch1 (hdr addr: 0x20052510)
        arbitration_lost (offset: 24, addr: 0x20052528): 0
        bus_lock (offset: 28, addr: 0x2005252c): 0
        error_active_state (offset: 32, addr: 0x20052530): 0
        error_warning_state (offset: 36, addr: 0x20052534): 0
        error_passive_state (offset: 40, addr: 0x20052538): 0
        bus_off_state (offset: 44, addr: 0x2005253c): 0
        bus_off_recovery (offset: 48, addr: 0x20052540): 0
        bus_off_recovery_manual (offset: 52, addr: 0x20052544): 0
        tdc_violation (offset: 56, addr: 0x20052548): 0

CAN-FD testing
--------------

For proper test/samples running two Zephyr snippets **rz-g3s-canfd0-test** and
**rz-g3s-canfd10-test** have been added to allow test/samples
execution for CAN-FD channels 0 and 1.

The CAN-FD shippets code can be found at::

    snippets/rz-g3s-canfd0-test
    snippets/rz-g3s-canfd1-test

To run test/samples for CAN-FD channels 0 use build command as below:

.. code-block:: bash

	west build -b rz_g3s -p always -S rz-g3s-canfd0-test <test|sample>

To run test/samples for CAN-FD channels 1 use build command as below:

.. code-block:: bash

	west build -b rz_g3s -p always -S rz-g3s-canfd1-test <test|sample>

.. toctree::
   :maxdepth: 4

   canfd_api_test.rst
   canfd_api_timing.rst

.. raw:: latex

    \newpage
