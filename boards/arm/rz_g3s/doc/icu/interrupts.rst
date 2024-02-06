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

IRQ pin Interrupt
`````````````````

IRQ interrupt is the interrupt from IRQ0-7 input pins.
Using IRQ pins as interrupt must be mapped GPIO pins onto IRQ pins by GPIO register setting.
The IRQ pin inputs are detected by a level or an edge. It can be selected “Low-level detection”, “Falling-edge
detection”, “Rising-edge detection” or “Falling/Rising-edge detection”.

TINT GPIO IRQs
``````````````

The TINT processing is done by RZ G3S GPIO driver :ref:`rzg3s-gpio-label`

Limitations
```````````

* Digital Noise Filter function not supported
* bus error interrupts not supported
* ECC error interrupts not supported
* Low power modes not supported

INTC driver overview
--------------------

The RZ G3S INTC driver (`drivers/interrupt_controller/intc_r9a08g045.c`) is responsible for proper
configuration and handling of NMI and IRQ pin Interrupts and implemented as 2nd level interrupt controller.
The mapping between INTC logical IRQ lines and RZ G3S SoC NMI and IRQ pin Interrupts defined as below:

+---------+---------+
| INTC IRQ|  SoC IRQ|
+=========+=========+
| 0       | NMI  (0)|
+---------+---------+
| 1       | IRQ0 (1)|
+---------+---------+
| 2       | IRQ1 (2)|
+---------+---------+
| 3       | IRQ2 (3)|
+---------+---------+
| 4       | IRQ3 (4)|
+---------+---------+
| 5       | IRQ4 (5)|
+---------+---------+
| 6       | IRQ5 (6)|
+---------+---------+
| 7       | IRQ6 (7)|
+---------+---------+
| 8       | IRQ7 (8)|
+---------+---------+

The helper macro are defined in::

    include/zephyr/dt-bindings/interrupt-controller/r7s9210-intc.h


INTC driver testing
-------------------

samples/drivers/irq_keys
````````````````````````

The **irq_keys** sample allows to test both NMI and IRQ pin Interrupts on RZ/G3S SMARC Evaluation Board Kit.


* GPIO P0_3 pin is configured as IRQ1 and routed to the USER_SW3 button (GPIO7). IRQ is tested by pressing button.

.. warning:: !!Caution!!

* NMI pin connected to PCIE_WAKE# signal which is routed to the B11 pin of the PCIE_SLOT connector.
  NMI pin IRQ can be tested by grounding B11 pin of the PCIE_SLOT connector.

The **irq_keys** DT overlay (samples/drivers/irq_keys/boards/rz_g3s.overlay) for RZ/G3S SMARC Evaluation Board Kit is below
which shows example DT configuration is below:

::

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
                pinmux = <RZG3S_PINMUX(PORT0, 3, 7)>;
            };
        };
    };

To build **irq_keys** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always samples/drivers/irq_keys/

The **irq_keys** test will produce below console output when executed:

.. code:: console

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

|
