Appendix B. Configure FPU on SCIF1 (experimental)
-------------------------------------------------

.. note::

    The provided snippet configures Zephyr to use SCIF1 (PMOD1_3A) for Cortex-M33_FPU core
    instead of the SCIF3 (SER1_UART).

To configure Cortex-M33_FPU core to use SCIF1 please do the following command:

.. code-block:: bash

   west build -p always -b rz_g3s_fpu -S rz-g3s-fpu-scif1 <zephyr app>

.. note::

	When starting Zephyr application Cortex-A55 core should be stopped on
	u-boot console to prevent hardware sharing issues.

Overview
********

This snippet configures Zephyr to use SCIF1 (PMOD1_3A) for Cortex-M33_FPU core
instead of the SCIF3 (SER1_UART).
This can be helpful for a testing purposes because the default
SCIF3 (SER1_UART) connection requires specific USBUART connector that
support 1.8v. To avoid using specific hardware - CM33_FPU core can be
switched to use SCIF1 (PMOD1_3A) console which works on more common 3.3v.

Please note that Cortex-M33 core should be flashed with :ref:`hello_world` so it
will minimize interfering with SCIF1 from Cortex-M33.

.. note::
    Please be aware that this is experimental feature. The following tests
    will not work in this configuration due to hw sharing conflicts:

    * **uart_async_api**
    * **wdt_basic_api** can be run only from the JLink using command **west debug**
      because TF-A do not support Cortex-M33_FPU core.
