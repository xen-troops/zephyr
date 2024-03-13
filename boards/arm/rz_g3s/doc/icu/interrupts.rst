Interrupt Controller
====================

Overview
--------

The interrupt controller decides the priority of interrupt sources and controls interrupt requests to the CPU.
The interrupt controller registers set the order of priority of each interrupt, allowing the user to process interrupt
requests according to the user-set priority.

Cortex-M33/Cortex-M33_FPU has Nested Vectored Interrupt Controller (NVIC) built in:

* 256 levels of priority
* 480 interrupts

All IRQ signals are routed to ARM NVIC through IM33/IM33_FPU modules.

The IM33/IM33_FPU performs various interrupt controls including synchronization for the external interrupts of NMI, IRQ
and GPIOINT and internal peripheral interrupts output by each IP.
And it notifies the interrupt to the built-in interrupt controller (NVIC) for Cortex-M33/Cortex-M33_FPU.

* Select 32 TINT from 82 GPIOINT.
* Indicate interrupt status to NVIC.
* Set interrupt detection method. (NMI, IRQ and TINT)
* Integration of bus error interrupts from system bus.
* Integration of ECC error interrupts from On-chip RAM.
* All interrupts can be masked by setting of SYSC Register.

External Interrupts pin IRQs supported are:

* NMI
* IRQ0-7
* TINT0-31 from 82 GPIOINT

Refer to Arm® Cortex®-M33 Processor Revision: r0p4 Technical Reference Manual.

Refer to "Interrupt Controller" section in Renesas RZ/G3S Group User’s Manual: Hardware

NMI pin IRQ
```````````

NMI interrupt is the highest priority interrupt by default even though it can be masked.
NMI is not treated as NMI exception.
The NMI pin input is detected by edge.
It can be selected “Rising-edge detection” or “Falling-edge detection”.

IRQ pin Interrupts
``````````````````

IRQ interrupt is the interrupt from IRQ0-7 input pins.
Using IRQ pins as interrupt must be mapped GPIO pins onto IRQ pins by GPIO register setting.
The IRQ pin inputs are detected by a level or an edge. It can be selected “Low-level detection”, “Falling-edge
detection”, “Rising-edge detection” or “Falling/Rising-edge detection”.

TINT GPIO IRQs
``````````````

The TINT processing is done by RZ G3S GPIO driver :ref:`rzg3s_gpio_label`

Limitations
```````````

* bus error interrupts not supported
* ECC error interrupts not supported
* Low power modes not supported

INTC driver overview
--------------------

The RZ G3S INTC driver is responsible for proper configuration and handling
of NMI and IRQ pin Interrupts and it is implemented as 2nd level interrupt controller.
The mapping between INTC logical IRQ lines and RZ G3S SoC NMI and IRQ pin Interrupts defined as below:

+---------+---------------+
| INTC IRQ|  SoC NVIC IRQ |
+=========+===============+
| 0       | NMI  (0)      |
+---------+---------------+
| 1       | IRQ0 (1)      |
+---------+---------------+
| 2       | IRQ1 (2)      |
+---------+---------------+
| 3       | IRQ2 (3)      |
+---------+---------------+
| 4       | IRQ3 (4)      |
+---------+---------------+
| 5       | IRQ4 (5)      |
+---------+---------------+
| 6       | IRQ5 (6)      |
+---------+---------------+
| 7       | IRQ6 (7)      |
+---------+---------------+
| 8       | IRQ7 (8)      |
+---------+---------------+

The RZ G3S INTC driver is enabled by default if corresponding DT node is enabled.
In addition below Kconfig options enabled in rz_g3s_defconfig.

.. code-block:: text

    CONFIG_MULTI_LEVEL_INTERRUPTS=y
    CONFIG_DYNAMIC_INTERRUPTS=y
    /* automatically enabled */
    CONFIG_INTC_R9A08G045=y

The RZ G3S INTC driver code can be found at:

.. code-block:: text

    drivers/interrupt_controller/intc_r9a08g045.c

The helper macro are defined in:

.. code-block:: text

    dt-bindings/interrupt-controller/r7s9210-intc.h


INTC driver testing
-------------------

samples/drivers/irq_keys
````````````````````````

The **irq_keys** sample allows to test both NMI and IRQ pin Interrupts on RZ/G3S SMARC Evaluation Board Kit.


* GPIO P0_3 pin is configured as IRQ1 and routed to the USER_SW3 button (GPIO7). IRQ is tested by pressing button.

.. warning:: !!Caution!!

* NMI pin connected to PCIE_WAKE# signal which is routed to the B11 pin of the PCIE_SLOT connector.
  NMI pin IRQ can be tested by grounding B11 pin of the PCIE_SLOT connector.

The **irq_keys** DT overlay (samples/drivers/irq_keys/boards/rz_g3s.overlay) for RZ/G3S SMARC Evaluation Board Kit,
which shows example DT configuration is below:

.. code-block:: dts

    / {
        keyboard {
            compatible = "irq-keys";

            pinctrl-names = "default";
            pinctrl-0 = <&keyboard_pins>;

            sw3 {
                   interrupt-parent = <&intc>;
                   interrupts = <R9A08G045_IRQ1 IRQ_TYPE_EDGE_RISING>;
            };

            sw_nmi {
                   interrupt-parent = <&intc>;
                   interrupts = <R9A08G045_IRQ_NMI IRQ_TYPE_EDGE_RISING>;
            };
         };
    };

    &pinctrl {
        keyboard_pins: keyboard_pins {
            sw3-pinmux {
                pinmux = <RZG3S_PINMUX(PORT0, 3, 7)>; /* IRQ1 */
                input-debounce = <(PINCTRL_RZG3S_FILTER_SET(3, 3))>;
            };

            sw3-pins {
                pins = "NMI";
                input-debounce = <(PINCTRL_RZG3S_FILTER_SET(1, 2))>;
            };
        };
    };

To build **irq_keys** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always samples/drivers/irq_keys/

The **irq_keys** test will produce below console output when executed:

.. code-block:: console

    [00:00:00.000,000] <inf> main: Starting IRQ keys sample...
                                                              Number of IRQ Keys detected 2
    [00:00:06.171,000] <inf> main: Button (irq line 1536) pressed 1 times
    [00:00:06.475,000] <inf> main: Button (irq line 1536) pressed 2 times
    [00:00:09.589,000] <inf> main: Button (irq line 1536) pressed 3 times
    [00:00:10.009,000] <inf> main: Button (irq line 1536) pressed 4 times
    [00:00:11.158,000] <inf> main: Button (irq line 1536) pressed 5 times
    [00:00:11.158,000] <inf> main: Changing of IRQ line (1536) detection mode to FALLING EDGE
    [00:00:11.159,000] <inf> main: Button (irq line 1536) pressed 7 times
    [00:00:11.259,000] <inf> main: Button (irq line 1536) pressed 8 times
    [00:00:11.259,000] <inf> main: Changing of IRQ line (1536) detection mode to RISING EDGE
    [00:00:11.362,000] <inf> main: Button (irq line 1536) pressed 9 times
    [00:00:11.363,000] <inf> main: Button (irq line 1536) pressed 10 times
    [00:00:11.363,000] <inf> main: Button (irq line 1536) pressed 11 times
    [00:00:11.363,000] <inf> main: Changing of IRQ line (1536) detection mode to FALLING EDGE
    [00:00:18.425,000] <inf> main: Button (irq line 512) pressed 1 times
    [00:00:18.634,000] <inf> main: Button (irq line 512) pressed 2 times
    [00:00:18.634,000] <inf> main: Button (irq line 512) pressed 3 times
    [00:00:18.635,000] <inf> main: Changing of IRQ line (512) detection mode to FALLING EDGE
    [00:00:18.635,000] <inf> main: Button (irq line 512) pressed 5 times
    [00:00:19.678,000] <inf> main: Button (irq line 512) pressed 6 times
    [00:00:19.679,000] <inf> main: Changing of IRQ line (512) detection mode to RISING EDGE
    [00:00:19.681,000] <inf> main: Button (irq line 512) pressed 7 times
    [00:00:19.682,000] <inf> main: Button (irq line 512) pressed 8 times
    [00:00:20.860,000] <inf> main: Button (irq line 512) pressed 9 times
    [00:00:20.861,000] <inf> main: Changing of IRQ line (512) detection mode to FALLING EDGE

ARM NVIC testing
----------------

tests/arch/arm/arm_interrupt
````````````````````````````

This test to verify code fault handling in ISR execution context and the behavior of irq_lock() and irq_unlock()
when invoked from User Mode. An additional test case verifies that null pointer dereferencing
attempts are detected and interpreted as CPU faults.

To build **tests/arch/arm/arm_interrupt** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/arch/arm/arm_interrupt

Console output:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-340-g17158a79e662 ***
    Running TESTSUITE arm_interrupt
    ===================================================================
    START - test_arm_esf_collection
    Testing ESF Reporting
    E: ***** USAGE FAULT *****
    E:   Attempt to execute undefined instruction
    E: r0/a1:  0x00000000  r1/a2:  0x00000001  r2/a3:  0x00000002
    E: r3/a4:  0x00000003 r12/ip:  0x0000000c r14/lr:  0x0000000f
    E:  xpsr:  0x01000000
    E: Faulting instruction address (r15/pc): 0x0002ec68
    E: >>> ZEPHYR FATAL ERROR 0: CPU exception on CPU 0
    E: Current thread: 0x20050080 (unknown)
    Caught system error -- reason 0
     PASS - test_arm_esf_collection in 0.038 seconds
    ===================================================================
    START - test_arm_interrupt
    Available IRQ line: 479
    E: >>> ZEPHYR FATAL ERROR 1: Unhandled interrupt on CPU 0
    E: Current thread: 0x20050140 (test_arm_interrupt)
    Caught system error -- reason 1
    E: r0/a1:  0x00000003  r1/a2:  0x20054944  r2/a3:  0x00000003
    E: r3/a4:  0x200539a0 r12/ip:  0x00000008 r14/lr:  0x000255cf
    E:  xpsr:  0x610001ef
    E: Faulting instruction address (r15/pc): 0x00023de2
    E: >>> ZEPHYR FATAL ERROR 3: Kernel oops on CPU 0
    E: Fault during interrupt handling

    E: Current thread: 0x20050140 (test_arm_interrupt)
    Caught system error -- reason 3
    E: r0/a1:  0x00000004  r1/a2:  0x20054944  r2/a3:  0x00000004
    E: r3/a4:  0x200539a0 r12/ip:  0x00000000 r14/lr:  0x000255cf
    E:  xpsr:  0x610001ef
    E: Faulting instruction address (r15/pc): 0x00023e00
    E: >>> ZEPHYR FATAL ERROR 4: Kernel panic on CPU 0
    E: Fault during interrupt handling

    E: Current thread: 0x20050140 (test_arm_interrupt)
    Caught system error -- reason 4
    ASSERTION FAIL [0] @ WEST_TOPDIR/zephyr/tests/arch/arm/arm_interrupt/src/arm_interrupt.c:225
    Intentional assert

    E: r0/a1:  0x00000004  r1/a2:  0x000000e1  r2/a3:  0x20050140
    E: r3/a4:  0x000001ef r12/ip:  0x00000000 r14/lr:  0x000255cf
    E:  xpsr:  0x610001ef
    E: Faulting instruction address (r15/pc): 0x0002ef54
    E: >>> ZEPHYR FATAL ERROR 4: Kernel panic on CPU 0
    E: Fault during interrupt handling

    E: Current thread: 0x20050140 (test_arm_interrupt)
    Caught system error -- reason 4
    E: ***** USAGE FAULT *****
    E:   Stack overflow (context area not valid)
    E: r0/a1:  0x00000000  r1/a2:  0x00000000  r2/a3:  0x000240b6
    E: r3/a4:  0x01000000 r12/ip:  0x00000000 r14/lr:  0x00000000
    E:  xpsr:  0x00000000
    E: Faulting instruction address (r15/pc): 0x00000000
    E: >>> ZEPHYR FATAL ERROR 2: Stack overflow on CPU 0
    E: Current thread: 0x20050140 (test_arm_interrupt)
    Caught system error -- reason 2
     PASS - test_arm_interrupt in 0.161 seconds
    ===================================================================
    START - test_arm_null_pointer_exception
    E: ***** MPU FAULT *****
    E:   Data Access Violation
    E:   MMFAR Address: 0x4
    E: r0/a1:  0x00000000  r1/a2:  0x00030054  r2/a3:  0x00000000
    E: r3/a4:  0x00000000 r12/ip:  0x00030040 r14/lr:  0x0002ec1f
    E:  xpsr:  0x61000000
    E: Faulting instruction address (r15/pc): 0x00023cae
    E: >>> ZEPHYR FATAL ERROR 0: CPU exception on CPU 0
    E: Current thread: 0x20050140 (test_arm_null_pointer_exception)
    Caught system error -- reason 0
     PASS - test_arm_null_pointer_exception in 0.038 seconds
    ===================================================================
    START - test_arm_user_interrupt
     PASS - test_arm_user_interrupt in 0.001 seconds
    ===================================================================
    TESTSUITE arm_interrupt succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [arm_interrupt]: pass = 4, fail = 0, skip = 0, total = 4 duration = 0.238 seconds
     - PASS - [arm_interrupt.test_arm_esf_collection] duration = 0.038 seconds
     - PASS - [arm_interrupt.test_arm_interrupt] duration = 0.161 seconds
     - PASS - [arm_interrupt.test_arm_null_pointer_exception] duration = 0.038 seconds
     - PASS - [arm_interrupt.test_arm_user_interrupt] duration = 0.001 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

.. raw:: latex

    \newpage
