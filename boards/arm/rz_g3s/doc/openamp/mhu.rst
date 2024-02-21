Message Handling Unit (MHU)
===========================

Overview
--------

MHU is a function for message communication between Cortex-A55(CA55) cores and Cortex-
M33(CM33/CM33_FPU). Message communication is done by shared RAM (On-chip RAM) for passing message and
response between CPUs and the function (MHU) for notifying when messages and responses are stored in the memory.

It povides the following functions:

* Interrupt by Message and response
* Event routing function
* Software interrupt control
* Provides 5 secure and 5 non-secure channels

Refer to "Message Handling Unit (MHU)" section in Renesas RZ/G3S Group User’s Manual: Hardware

Limitations
```````````

* Software interrupt control not supported

MHU driver overview
--------------------

The RZ G3S MHU driver instance implements MHU single channel to communicate between two cores.
It provides Zephyr :ref:`ipm_api` System interface implementation.
The device-tree node to configure driver is the following:

.. code-block:: dts

    mhu1_shm: memory@62F01000 {
        compatible = "mmio-sram";
        reg = <0x62F01000 0x8>;
    };

    mhu1: mhu@40400020 {
        compatible = "renesas,mhu";
        reg = <0x40400020 0x20>;
        interrupts = <58 0>;
        interrupt-parent = <&nvic>;
        clocks = <&cpg CPG_MOD R9A08G045_MHU_PCLK>;
        resets = <&cpg_rctl R9A08G045_MHU_RESETN>;
        mhu_type = <1>;
        memory-region = <&mhu1_shm>;
        status = "disabled";
    };


Where:

* mhu_type can be 0 to configure MHU to work as transmitter and 1 to work as responder
* memory-region accepts phandle for memory within shared memory region to pass data between CPU

MHU driver testing
-------------------

MHU driver for CM33 and CM33_FPU cores can be tested from **openamp_rsc_table_rzg3** sample.
Please refer to **Openamp** section for additional information.

|
