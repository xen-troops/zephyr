GPIO
====

GPIO overview
-------------

.. note::

    TODO

GPIO testing
-------------

tests/drivers/gpio/gpio_basic_api
`````````````````````````````````

To build gpio_basic_api test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/gpio/gpio_basic_api

The gpio_basic_api test will produce below console output when executed:

|

::

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

|

samples/basic/button
````````````````````

To build gpio_basic_api test run command:

.. code-block:: bash

    west build -b rz_g3s -p always samples/basic/button/

The **USER_SW2** button is enabled by default for test purposes.
The gpio_basic_api test will produce below console output when executed and
the **USER_SW2** button pressed:

|

::

    *** Booting Zephyr OS build v3.5.0-rc2-227-g9c638e9d4fa9 ***
    Set up button at gpio@12 pin 0
    Press the button
    Button pressed at 1859207199
    Button pressed at 1949018630
    Button pressed at 1998292400
    Button pressed at 2057400586

|
