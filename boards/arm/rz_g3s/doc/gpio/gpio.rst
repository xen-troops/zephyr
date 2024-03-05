.. _rzg3s_gpio_label:

General Purpose Input Output Port (GPIO)
========================================

GPIO overview
-------------

The RZ G3S SoC can use:

* up to 82 general-purpose I/O pins, which are split on 19 GPIO ports.
* up to 32 GPIO pin IRQs mapped through TINT module.

Each port of the general-purpose I/O port is multiplexed with the terminal of the peripheral function,
and either the general-purpose I/O port function or the peripheral function can be selected by setting
the register.

This section is limited to general-purpose I/O pins functionality only.

Features:

* each GPIO pin can be configured as input or output, or both.
* each GPIO pin can generate interrupts when in input mode.
* each GPIO pin supports buffer drive ability control (IOLH_m)
* each GPIO pin supports pull-up/Pull-down Switching (PUPD_m)
* each GPIO pin supports digital noise filter control (FILONOFF_m)

GPIO Interrupt (TINT)
`````````````````````
GPIO interrupt is the interrupt using GPIO pins as external interrupt input pins.
When using GPIO pins as interrupts, assign GPIO pins as external interrupt input pins (GPIOINT0-81) by GPIO
register setting.

The 32 interrupt sources can be assigned to TINT from GPIOINT0-81 by TINT Source Selection Register (TSSR0-7) in
IM33/IM33_FPU. Only 1:1 mapping is allowed.
The TINT inputs can be detected by a level or an edge. It can be selected “Rising-edge detection”, “Falling-edge
detection”, “High-level detection” or “Low-level detection”.

.. code-block:: text

              +----------------+
              |TINT            |
              |                |
    GPIO0     |                | 0
        ------+                +----
    GPIO1     |                | 1
        ------+                +----
              |                |
        ...   |                | ...
              |                |
              |                |
              |                |
    GPIO81    |                | 31
         -----+                +----
              +----------------+

Refer to "General Purpose Input Output Port (GPIO)" section in "Renesas RZ/G3S Group User’s Manual: Hardware"
Refer to :ref:`rzg3s_pinctrl_label`

Limitations
```````````

* Digital Noise Filter function is not supported

GPIO driver overview
--------------------

The RZ G3S GPIO driver provides Zephyr GPIO System interface :ref:`gpio_api` implementation.

The RZ G3S GPIO driver is logically implemented as gpio-controller and set of GPIO ports (dts/arm/renesas/rz/r9a08g045.dtsi).
GPIO ports provides Zephyr GPIO System interface implementation.
The gpio-controller provides GPIO Interrupt (TINT) management to the GPIO ports.
When GPIO consumers requests GPIO pin IRQ functionality the gpio-controller selects free TINT IRQ and maps it to requested GPIO pin.

.. code-block:: dts

	gpio: gpio-controller@41060020 {
		compatible = "renesas,r9a08g045-gpio";
		reg = <0x41060020 0x30>;
		reg-names = "tint";
		interrupts =
			<429 0>, <430 0>, <431 0>, <432 0>,
			<433 0>, <434 0>, <435 0>, <436 0>,
			<437 0>, <438 0>, <439 0>, <440 0>,
			<441 0>, <442 0>, <443 0>, <444 0>,
			<445 0>, <446 0>, <447 0>, <448 0>,
			<449 0>, <450 0>, <451 0>, <452 0>,
			<453 0>, <454 0>, <455 0>, <456 0>,
			<457 0>, <458 0>, <459 0>, <460 0>;
		#address-cells = <1>;
		#size-cells = <0>;

		port0: gpio@0 {
			reg = <0x0>;
			gpio-controller;
			#gpio-cells=<2>;
			ngpios = <4>;
			status = "disabled";
		};

	...

		port18: gpio@12 {
			reg = <0x12>;
			gpio-controller;
			#gpio-cells=<2>;
			ngpios = <6>;
			status = "disabled";
		};
    };

The GPIO consumer should use reference at GPIO port DT node and GPIO pin number within GPIO port.
The GPIO port should be enabled in DT (example :ref:`GPIO DT request example <rzg3s_gpio_dts_consumer>`).

The GPIO subsystem is enabled by default in ``rz_g3s_defconfig``,
which automatically enables RZ G3S GPIO driver if corresponding DT node is enabled.

.. code-block:: text

    CONFIG_GPIO=y
    /* automatically enabled */
    CONFIG_GPIO_RZG3S=y


The RZ G3S GPIO driver code can be found at:

.. code-block:: text

    drivers/gpio/gpio_rzg3s.c

The DT helper macro are defined in:

.. code-block:: text

    dt-bindings/gpio/gpio.h
    dt-bindings/gpio/rzg3s-gpio.h
    dt-bindings/pinctrl/renesas/pinctrl-r9a08g045.h

Supported GPIO DT flags:

+-----------------------------------+-------+--------------------------+
| DT GPIO flags                     |       |                          |
+===================================+=======+==========================+
|GPIO_ACTIVE_LOW                    |generic|                          |
+-----------------------------------+-------+--------------------------+
|GPIO_ACTIVE_HIGH                   |generic|                          |
+-----------------------------------+-------+--------------------------+
|GPIO_PULL_UP                       |generic|mapped at PUPD_m registers|
+-----------------------------------+-------+--------------------------+
|GPIO_PULL_DOWN                     |generic|mapped at PUPD_m registers|
+-----------------------------------+-------+--------------------------+
|RZG3S_GPIO_DRIVE_IOLH_SET(iolh_val)|custom |mapped at IOLH_m registers|
+-----------------------------------+-------+--------------------------+

The example of DT GPIO flags usage:

.. code-block:: dts

	gpio-consumer {
		out-gpios = <&port8 2 (GPIO_PULL_UP|RZA2_GPIO_DRIVE_IOLH_SET(PINCTRL_RZG3S_PIN_IOLH_A_3_3V_1900)>;
	};

GPIO testing
-------------

tests/drivers/gpio/gpio_basic_api
`````````````````````````````````
To run **gpio_basic_api** test it's required to connect **PMOD1 Type-3A** pins
as described on picture below to form loopback connection between
RZ G3S GPIOI pins P8_2 and P8_3:

.. code-block:: text

    +--------------------------------------------------+
    | RZ G3S SMARC Ev Board                            |
    |                                                  |
    |                                        PMOD1     |
    |                                        Type-3A   |
    |    +-------------+                          +----+
    |    |RZ G3S SoC   |                          |    |
    |    |             |                          |    |
    |    |             |                          |    |
    |    |             |                          |    |
    |    |             |      GPIO12:PMOD1_GPIO12 |    |
    |    |        P8_2 X-------------------------->  9 X--------+
    |    |             |                          |    |        |
    |    |             |                          |    |        |
    |    |             |                          |    |        |
    |    |             |                          |    |        |
    |    |             |      GPIO13:PMOD1_GPIO13 |    |        |
    |    |        P8_3 X<-------------------------+ 10 X<-------+
    |    |             |                          |    |
    |    +-------------+                          +----+
    |                                                  |
    |                                                  |
    +--------------------------------------------------+

The **gpio_basic_api** DT overlay (`tests/drivers/gpio/gpio_basic_api/boards/rz_g3s.overlay`)
for RZ/G3S SMARC Evaluation Board Kit is below:

.. _rzg3s_gpio_dts_consumer:

.. code-block:: dts

	/ {
		resources {
			compatible = "test-gpio-basic-api";
			out-gpios = <&port8 2 0>;
			in-gpios = <&port8 3 0>;
		};
	};

	&port8 {
		status = "okay";
	};

To build **gpio_basic_api** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/gpio/gpio_basic_api

The **gpio_basic_api** test will produce below console output when executed:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-227-g5edb05de40b5 ***
    Running TESTSUITE gpio_port
    ===================================================================
    START - test_gpio_port
    Validate device gpio@8
    Check gpio@8 output 2 connected to input 3
    OUT 2 to IN 3 linkage works
    - bits_physical
    - pin_physical
    - check_raw_output_levels
    - check_logic_output_levels
    - check_input_levels
    - bits_logical
     PASS - test_gpio_port in 0.021 seconds
    ===================================================================
    TESTSUITE gpio_port succeeded
    Running TESTSUITE gpio_port_cb_mgmt
    ===================================================================
    START - test_gpio_callback_add_remove
    callback_2 triggered: 1
    callback_1 triggered: 1
    callback_2 triggered: 1
     PASS - test_gpio_callback_add_remove in 3.610 seconds
    ===================================================================
    START - test_gpio_callback_enable_disable
    callback_2 triggered: 1
    callback_1 triggered: 1
    callback_2 triggered: 1
    callback_1 triggered: 1
     PASS - test_gpio_callback_enable_disable in 3.612 seconds
    ===================================================================
    START - test_gpio_callback_self_remove
    callback_remove_self triggered: 1
    callback_1 triggered: 1
    callback_1 triggered: 1
     PASS - test_gpio_callback_self_remove in 2.510 seconds
    ===================================================================
    TESTSUITE gpio_port_cb_mgmt succeeded
    Running TESTSUITE gpio_port_cb_vari
    ===================================================================
    START - test_gpio_callback_variants
    callback triggered: 1
    OUT init a0001, IN cfg 3400000, cnt 1
    callback triggered: 1
    OUT init 60000, IN cfg 5400000, cnt 1
    callback triggered: 1
    OUT init 60000, IN cfg 5c00000, cnt 1
    callback triggered: 1
    OUT init a0001, IN cfg 3c00000, cnt 1
    callback triggered: 1
    callback triggered: 2
    callback triggered: 3
    OUT init 60000, IN cfg 4400000, cnt 3
    callback triggered: 1
    callback triggered: 2
    callback triggered: 3
    OUT init a0001, IN cfg 2400000, cnt 3
    callback triggered: 1
    callback triggered: 2
    callback triggered: 3
    OUT init 60000, IN cfg 4c00000, cnt 3
    callback triggered: 1
    callback triggered: 2
    callback triggered: 3
    OUT init a0001, IN cfg 2c00000, cnt 3
    Mode 7400000 not supported
     PASS - test_gpio_callback_variants in 8.860 seconds
    ===================================================================
    TESTSUITE gpio_port_cb_vari succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [gpio_port]: pass = 1, fail = 0, skip = 0, total = 1 duration = 0.021 seconds
     - PASS - [gpio_port.test_gpio_port] duration = 0.021 seconds

    SUITE PASS - 100.00% [gpio_port_cb_mgmt]: pass = 3, fail = 0, skip = 0, total = 3 duration = 9.732
    ses
     - PASS - [gpio_port_cb_mgmt.test_gpio_callback_add_remove] duration = 3.610 seconds
     - PASS - [gpio_port_cb_mgmt.test_gpio_callback_enable_disable] duration = 3.612 seconds
     - PASS - [gpio_port_cb_mgmt.test_gpio_callback_self_remove] duration = 2.510 seconds

    SUITE PASS - 100.00% [gpio_port_cb_vari]: pass = 1, fail = 0, skip = 0, total = 1 duration = 8.860
    ses
     - PASS - [gpio_port_cb_vari.test_gpio_callback_variants] duration = 8.860 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

samples/basic/button
````````````````````

To build **basic/button** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always samples/basic/button/

The **USER_SW2** button is enabled by default for test purposes.
The **basic/button** test will produce below console output when executed and
the **USER_SW2** button pressed:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-227-g9c638e9d4fa9 ***
    Set up button at gpio@12 pin 0
    Press the button
    Button pressed at 1859207199
    Button pressed at 1949018630
    Button pressed at 1998292400
    Button pressed at 2057400586

.. raw:: latex

    \newpage
