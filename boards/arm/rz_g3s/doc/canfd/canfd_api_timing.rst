Channel0 tests/drivers/can/timing
`````````````````````````````````

The **tests/drivers/can/timing** test is intended to perform full testing of Zephyr CAN API
interaction with RZ G3S CAN-FD driver in loopback CAN test mode.

To build **tests/drivers/can/timing** test for CAN-FD channel 0 run command:

.. code-block:: bash

    west build -b rz_g3s -p always -S rz-g3s-canfd0-test tests/drivers/can/timing

The **tests/drivers/can/api** test will produce below console output when executed:

.. code-block:: console

	 I: can@400c0000:"CANFD IP init done ver:31240143 pclk:80000000Hz ext_clk:0Hz"
	 I: channel@0:"init done"
	 I: channel@1:"init done"
	*** Booting Zephyr OS build v3.5.0-rc2-299-g93cceb9be768 ***
	Running TESTSUITE can_timing
	===================================================================
	testing on device channel@0 @ 40000000 Hz
	START - test_set_timing_data_max
	 PASS - test_set_timing_data_max in 0.001 seconds
	===================================================================
	START - test_set_timing_data_min
	 PASS - test_set_timing_data_min in 0.001 seconds
	===================================================================
	START - test_set_timing_max
	 PASS - test_set_timing_max in 0.001 seconds
	===================================================================
	START - test_set_timing_min
	 PASS - test_set_timing_min in 0.001 seconds
	===================================================================
	START - test_timing
	testing bitrate 20000, sample point 87.5% (valid): sjw = 12, prop_seg = 0, phase_seg1 = 174, phase_se%
	testing bitrate 50000, sample point 87.5% (valid): sjw = 12, prop_seg = 0, phase_seg1 = 174, phase_se%
	testing bitrate 125000, sample point 87.5% (valid): sjw = 10, prop_seg = 0, phase_seg1 = 139, phase_s%
	testing bitrate 250000, sample point 87.5% (valid): sjw = 10, prop_seg = 0, phase_seg1 = 139, phase_s%
	testing bitrate 500000, sample point 87.5% (valid): sjw = 5, prop_seg = 0, phase_seg1 = 69, phase_seg%
	testing bitrate 800000, sample point 80.0% (valid): sjw = 5, prop_seg = 0, phase_seg1 = 39, phase_seg%
	testing bitrate 1000000, sample point 75.0% (valid): sjw = 5, prop_seg = 0, phase_seg1 = 29, phase_se%
	testing bitrate 125000, sample point 90.0% (valid): sjw = 8, prop_seg = 0, phase_seg1 = 143, phase_se%
	testing bitrate 125000, sample point 80.0% (valid): sjw = 32, prop_seg = 0, phase_seg1 = 255, phase_s%
	testing bitrate 125000, sample point 100.0% (invalid): OK
	testing bitrate 1000001, sample point 75.0% (invalid): OK
	  PASS - test_timing in 0.131 seconds
	===================================================================
	START - test_timing_data
	testing bitrate 500000, sample point 87.5% (valid): sjw = 1, prop_seg = 0, phase_seg1 = 13, phase_seg%
	testing bitrate 1000000, sample point 75.0% (valid): sjw = 5, prop_seg = 0, phase_seg1 = 29, phase_se%
	testing bitrate 500000, sample point 90.0% (valid): sjw = 1, prop_seg = 0, phase_seg1 = 17, phase_seg%
	testing bitrate 500000, sample point 80.0% (valid): sjw = 4, prop_seg = 0, phase_seg1 = 31, phase_seg%
	testing bitrate 500000, sample point 100.0% (invalid): OK
	testing bitrate 8000001, sample point 75.0% (invalid): OK
	  PASS - test_timing_data in 0.064 seconds
	===================================================================
	TESTSUITE can_timing succeeded

	------ TESTSUITE SUMMARY START ------

	SUITE PASS - 100.00% [can_timing]: pass = 6, fail = 0, skip = 0, total = 6 duration = 0.199 seconds
	 - PASS - [can_timing.test_set_timing_data_max] duration = 0.001 seconds
	 - PASS - [can_timing.test_set_timing_data_min] duration = 0.001 seconds
	 - PASS - [can_timing.test_set_timing_max] duration = 0.001 seconds
	 - PASS - [can_timing.test_set_timing_min] duration = 0.001 seconds
	 - PASS - [can_timing.test_timing] duration = 0.131 seconds
	 - PASS - [can_timing.test_timing_data] duration = 0.064 seconds

	------ TESTSUITE SUMMARY END ------

	===================================================================
	PROJECT EXECUTION SUCCESSFUL
