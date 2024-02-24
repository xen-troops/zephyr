Security attribution unit (SAU)
===============================

SAU overview
------------

The Renesas RZ G3S SoC has one Cortex-M33_FPU (r0p4) System Core which has SAU implemented.

The SAU is a programmable unit that determines the security of an address.
The SAU is only implemented if the ARMv8-M Security Extension is included in the processor.
The number of regions that are included in the SAU is 8.

For instructions and data, the SAU returns the security attribute that is associated with the address.

For instructions, the attribute determines the allowable Security state of the processor when the instruction is executed.
It can also identify whether code at a Secure address can be called from Non-secure state.

For data, the attribute determines whether a memory address can be accessed from Non-secure state,
and also whether the external memory request is marked as Secure or Non-secure.

Refer to Arm® Cortex®-M33 Processor Revision: r0p4 Technical Reference Manual
for more information.

SAU default configuration
-------------------------

| The Renesas RZ G3S Zephyr implementation declares ARM SAU support at SoC level by enabling
| `CONFIG_CPU_HAS_ARM_SAU` Kconfig option. This also enables below Kconfig options by default.

.. code-block:: text

    CONFIG_CPU_HAS_TEE=y
    CONFIG_ARMV8_M_SE=y


SAU testing
-----------

.. note::

    the ARMv8-M Security Extension is not tested.

|
