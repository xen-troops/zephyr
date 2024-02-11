Clock Pulse Generator (CPG)
===========================

CPG overview
-------------

Features:

* PLL control
* Clock generation and control
* Reset generation and control
* PLL clock/Clock/Reset Monitor

Refer to "Clock Pulse Generator (CPG)" section in "Renesas RZ/G3S Group User’s Manual: Hardware".

Refer to "Low Power Consumption" section in "Renesas RZ/G3S Group User’s Manual: Hardware".

Refer to "Renesas RZ/G3S clock_list_r1.00" document.

CPG driver overview
-------------------

The RZ G3S CPG driver provides Zephyr System :ref:`clock_control_api` and
:ref:`reset_api` API which are implemented as separate Zephyr drivers:

* clock controller driver
* reset controller driver

The CPG Clock controller driver provides Zephyr System Clock Control API to enable
HW module clock (CPG_CLKON_xx) and manage Module Standby Mode (CPG_BUS_xx_MSTOP).
In the current implementation the Module Standby Mode (MSTOP) is
cleared together with enabling HW module clock.

Hence, the Zephyr is running on Cortex-M33 core and it's started by Cortex-A55 Core - it's
expected that PLL configuration is fully done by SW running by Cortex-A55 Core before
starting Zephyr on Cortex-M33 core.
Therefore the CPG Clock controller driver **does not** perform any kind of PLL control and
uses static default frequencies for PLL/Core clocks as specified in "Renesas RZ/G3S clock_list_r1.00" document.
See :ref:`cpg_freq_tbl`.

The CPG Reset driver provides Zephyr System Rest API to assert/de-assert or toggle HW Module reset line(s).

The RZ G3S CPG DT definition (dts/arm/renesas/rz/r9a08g045.dtsi):

.. code-block:: dts

	cpg: clock-controller@41010000 {
		compatible = "renesas,r9a08g045-cpg-mssr";
		reg = <0x41010000 0x10000>;
		clocks = <&extal_clk>;
		clock-names = "extal";
		#clock-cells = <2>;

		cpg_rctl: reset-controller {
			compatible = "renesas,r9a08g045-cpg-reset";
			#reset-cells = <1>;
			status = "okay";
		};
	};

The Zephyr Clock control and Reset subsystems are enabled by default in rz_g3s_defconfig,
which automatically enables RZ G3S CPG clock and reset drivers
if corresponding DT node are enabled.

.. code-block:: text

    CONFIG_CLOCK_CONTROL=y
    CONFIG_RESET=y
    /* automatically enabled */
    CONFIG_CLOCK_CONTROL_RENESAS_RZG3S=y
    CONFIG_RESET_RENESAS_RZG3S_CPG

The RZ G3S CPG clock control driver code can be found at::

    drivers/clock_control/clock_control_r9a08g045_cpg_mssr.c

The RZ G3S CPG reset driver code can be found at::

    drivers/reset/reset_r9a08g045_cpg.c

The RZ G3S CPG  DT helper macro are defined in::

    dt-bindings/clock/r9a08g045_cpg_mssr.h

The RZ G3S SoC design required enabling module clock and releasing module reset line for enabling HW module.
The HW module enabling code example:

.. code-block:: C

	ret = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->clk_mod);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "Failed to configure clk");
		return ret;
	}

	(void)reset_line_toggle_dt(&cfg->rspin_rst);

Limitations
```````````

* PLL control is not supported

.. _cpg_freq_tbl:

Changing CPG PLL/Core clocks frequencies
----------------------------------------

The default frequencies for PLL/Core clocks can be changed manually by editing `rzg3s_core_clks` table in the
the CPG Clock controller driver (drivers/clock_control/clock_control_r9a08g045_cpg_mssr.c) code:

.. code-block:: C

    static const struct rzg3s_cpg_core_clk rzg3s_core_clks[MOD_CLK_BASE] = {
        /* External Clock Inputs */
        DEF_INPUT(CLK_EXTAL, 24000000),

        DEF_FIXED(R9A08G045_OSCCLK, CLK_EXTAL, 24000000),
        DEF_FIXED(R9A08G045_OSCCLK2, CLK_EXTAL, 8000000),
        ...
    };

|
