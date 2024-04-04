.. _snippet-rz-g3s-fpu-scif1:

RZ/G3S FPU via SCIF1
####################

.. code-block:: bash

   west build -p always -b rz_g3s_fpu -S rz-g3s-fpu-scif1 <zephyr app>

Overview
********

This snippet configures Zephyr to use SCIF1 (PMOD1_3A) for Cortex-M33_FPU core
instead of the SCIF3 (SER1_UART).
This can be helpful for a testing purposes because the default
SCIF3 (SER1_UART) connection requires specific USBUART connector that
support 1.8v. To avoid using specific hardware - CM33_FPU core can be
switched to use SCIF1 (PMOD1_3A) console which works on more common 3.3v.

Please note that CM33 core should be flashed with :ref:`hello_world`
to avoid side effects.
