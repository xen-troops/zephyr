Interrupt Controller
====================

NVIC overview
-------------

.. note::

    TODO

INTC overview
-------------

.. note::

    TODO

INTC testing
-------------

samples/drivers/irq_keys
`````````````````````````````````

To build **irq_keys** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always samples/drivers/irq_keys/

The **irq_keys** test will produce below console output when executed:

::

    *** Booting Zephyr OS build v3.5.0-rc2-233-gc7b11c7fbca1 ***
    [00:00:00.001,000] <inf> main: Starting IRQ keys sample...
                                                 Number of IRQ Keys detected 1
    [00:00:10.063,000] <inf> main: Button (irq line 1025) pressed 1 times
    [00:00:13.320,000] <inf> main: Button (irq line 1025) pressed 2 times
    [00:00:15.178,000] <inf> main: Button (irq line 1025) pressed 3 times
    [00:00:16.946,000] <inf> main: Button (irq line 1025) pressed 4 times
    [00:00:18.525,000] <inf> main: Button (irq line 1025) pressed 5 times
    [00:00:18.966,000] <inf> main: Button (irq line 1025) pressed 6 times
    [00:00:18.967,000] <inf> main: Changing of IRQ line (1025) detection mode to FALLING EDGE
    [00:00:19.878,000] <inf> main: Button (irq line 1025) pressed 7 times
    [00:00:20.928,000] <inf> main: Button (irq line 1025) pressed 8 times
    [00:00:21.463,000] <inf> main: Button (irq line 1025) pressed 9 times
    [00:00:21.673,000] <inf> main: Button (irq line 1025) pressed 10 times
    [00:00:21.673,000] <inf> main: Changing of IRQ line (1025) detection mode to RISING EDGE
    [00:00:21.713,000] <inf> main: Button (irq line 1025) pressed 11 times
    [00:00:22.387,000] <inf> main: Button (irq line 1025) pressed 12 times
    [00:00:23.071,000] <inf> main: Button (irq line 1025) pressed 13 times
    [00:00:23.071,000] <inf> main: Changing of IRQ line (1025) detection mode to FALLING EDGE

|
