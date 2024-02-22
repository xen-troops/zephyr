Channel0 tests/drivers/can/api
``````````````````````````````

The **tests/drivers/can/api** test is intended to perform full testing of Zephyr CAN API
interaction with RZ G3S CAN-FD driver in loopback CAN test mode.

To build **tests/drivers/can/api** test for CAN-FD channel 0 run command:

.. code-block:: bash

    west build -b rz_g3s -p always -S rz-g3s-canfd0-test tests/drivers/can/api

The **tests/drivers/can/api** test will produce below console output when executed:

.. code-block:: console

	I: can@400c0000:"CANFD IP init done ver:31240143 pclk:80000000Hz ext_clk:0Hz"
	I: channel@0:"init done"
	I: channel@1:"init done"
	*** Booting Zephyr OS build v3.5.0-rc2-299-g93cceb9be768 ***
	Running TESTSUITE can_classic
	===================================================================
	START - test_add_filter
	 PASS - test_add_filter in 0.002 seconds
	===================================================================
	START - test_filters_added_while_stopped
	 PASS - test_filters_added_while_stopped in 0.103 seconds
	===================================================================
	START - test_filters_preserved_through_bitrate_change
	 PASS - test_filters_preserved_through_bitrate_change in 0.105 seconds
	===================================================================
	START - test_filters_preserved_through_mode_change
	 PASS - test_filters_preserved_through_mode_change in 0.105 seconds
	===================================================================
	START - test_get_capabilities
	 PASS - test_get_capabilities in 0.002 seconds
	===================================================================
	START - test_get_core_clock
	 PASS - test_get_core_clock in 0.002 seconds
	===================================================================
	START - test_get_state
	 PASS - test_get_state in 0.002 seconds
	===================================================================
	START - test_max_ext_filters
	E: channel@0:"AFLADD: no free entries"
	 PASS - test_max_ext_filters in 0.008 seconds
	===================================================================
	START - test_max_std_filters
	E: channel@0:"AFLADD: no free entries"
	 PASS - test_max_std_filters in 0.008 seconds
	===================================================================
	START - test_receive_timeout
	 PASS - test_receive_timeout in 0.102 seconds
	===================================================================
	START - test_recover
	 PASS - test_recover in 0.002 seconds
	===================================================================
	START - test_recover_while_stopped
	 PASS - test_recover_while_stopped in 0.102 seconds
	===================================================================
	START - test_send_and_forget
	 PASS - test_send_and_forget in 0.003 seconds
	===================================================================
	START - test_send_callback
	 PASS - test_send_callback in 0.002 seconds
	===================================================================
	START - test_send_fd_format
	E: channel@0:"TX: unsupported frame flags 0x04"
	 PASS - test_send_fd_format in 0.005 seconds
	===================================================================
	START - test_send_invalid_dlc
	E: channel@0:"TX: frame dlc 9 exceeds maximum (8)"
	 PASS - test_send_invalid_dlc in 0.005 seconds
	===================================================================
	START - test_send_receive_ext_id
	 PASS - test_send_receive_ext_id in 0.005 seconds
	===================================================================
	START - test_send_receive_ext_id_masked
	 PASS - test_send_receive_ext_id_masked in 0.005 seconds
	===================================================================
	START - test_send_receive_ext_id_rtr
	 PASS - test_send_receive_ext_id_rtr in 0.207 seconds
	===================================================================
	START - test_send_receive_msgq
	 PASS - test_send_receive_msgq in 0.015 seconds
	===================================================================
	START - test_send_receive_std_id
	 PASS - test_send_receive_std_id in 0.005 seconds
	===================================================================
	START - test_send_receive_std_id_masked
	 PASS - test_send_receive_std_id_masked in 0.005 seconds
	===================================================================
	START - test_send_receive_std_id_rtr
	 PASS - test_send_receive_std_id_rtr in 0.206 seconds
	===================================================================
	START - test_send_receive_wrong_id
	 PASS - test_send_receive_wrong_id in 0.102 seconds
	===================================================================
	START - test_send_while_stopped
	 PASS - test_send_while_stopped in 0.102 seconds
	===================================================================
	START - test_set_bitrate
	 PASS - test_set_bitrate in 0.102 seconds
	===================================================================
	START - test_set_bitrate_too_high
	 PASS - test_set_bitrate_too_high in 0.102 seconds
	===================================================================
	START - test_set_bitrate_while_started
	 PASS - test_set_bitrate_while_started in 0.002 seconds
	===================================================================
	START - test_set_mode_while_started
	 PASS - test_set_mode_while_started in 0.002 seconds
	===================================================================
	START - test_set_state_change_callback
	 PASS - test_set_state_change_callback in 0.001 seconds
	===================================================================
	START - test_set_timing_while_started
	 PASS - test_set_timing_while_started in 0.002 seconds
	===================================================================
	START - test_start_while_started
	I: channel@0:"start: already started"
	 PASS - test_start_while_started in 0.004 seconds
	===================================================================
	START - test_stop_while_stopped
	 PASS - test_stop_while_stopped in 0.102 seconds
	===================================================================
	TESTSUITE can_classic succeeded
	Running TESTSUITE can_utilities
	===================================================================
	START - test_can_bytes_to_dlc
	 PASS - test_can_bytes_to_dlc in 0.001 seconds
	===================================================================
	START - test_can_dlc_to_bytes
	 PASS - test_can_dlc_to_bytes in 0.001 seconds
	===================================================================
	START - test_can_frame_matches_filter
	 PASS - test_can_frame_matches_filter in 0.001 seconds
	===================================================================
	TESTSUITE can_utilities succeeded
	Running TESTSUITE canfd
	===================================================================
	START - test_filters_preserved_through_classic_to_fd_mode_change
	 PASS - test_filters_preserved_through_classic_to_fd_mode_change in 0.307 seconds
	===================================================================
	START - test_filters_preserved_through_fd_to_classic_mode_change
	 PASS - test_filters_preserved_through_fd_to_classic_mode_change in 0.307 seconds
	===================================================================
	START - test_get_capabilities
	 PASS - test_get_capabilities in 0.001 seconds
	===================================================================
	START - test_send_receive_classic
	 PASS - test_send_receive_classic in 0.005 seconds
	===================================================================
	START - test_send_receive_fd
	 PASS - test_send_receive_fd in 0.004 seconds
	===================================================================
	START - test_send_receive_mixed
	 PASS - test_send_receive_mixed in 0.004 seconds
	===================================================================
	START - test_set_bitrate_data_while_started
	 PASS - test_set_bitrate_data_while_started in 0.002 seconds
	===================================================================
	START - test_set_timing_data_while_started
	 PASS - test_set_timing_data_while_started in 0.002 seconds
	===================================================================
	TESTSUITE canfd succeeded

	------ TESTSUITE SUMMARY START ------

	SUITE PASS - 100.00% [can_classic]: pass = 33, fail = 0, skip = 0, total = 33 duration = 1.527
	seconds
	 - PASS - [can_classic.test_add_filter] duration = 0.002 seconds
	 - PASS - [can_classic.test_filters_added_while_stopped] duration = 0.103 seconds
	 - PASS - [can_classic.test_filters_preserved_through_bitrate_change] duration = 0.105 seconds
	 - PASS - [can_classic.test_filters_preserved_through_mode_change] duration = 0.105 seconds
	 - PASS - [can_classic.test_get_capabilities] duration = 0.002 seconds
	 - PASS - [can_classic.test_get_core_clock] duration = 0.002 seconds
	 - PASS - [can_classic.test_get_state] duration = 0.002 seconds
	 - PASS - [can_classic.test_max_ext_filters] duration = 0.008 seconds
	 - PASS - [can_classic.test_max_std_filters] duration = 0.008 seconds
	 - PASS - [can_classic.test_receive_timeout] duration = 0.102 seconds
	 - PASS - [can_classic.test_recover] duration = 0.002 seconds
	 - PASS - [can_classic.test_recover_while_stopped] duration = 0.102 seconds
	 - PASS - [can_classic.test_send_and_forget] duration = 0.003 seconds
	 - PASS - [can_classic.test_send_callback] duration = 0.002 seconds
	 - PASS - [can_classic.test_send_fd_format] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_invalid_dlc] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_ext_id] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_ext_id_masked] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_ext_id_rtr] duration = 0.207 seconds
	 - PASS - [can_classic.test_send_receive_msgq] duration = 0.015 seconds
	 - PASS - [can_classic.test_send_receive_std_id] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_std_id_masked] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_std_id_rtr] duration = 0.206 seconds
	 - PASS - [can_classic.test_send_receive_wrong_id] duration = 0.102 seconds
	 - PASS - [can_classic.test_send_while_stopped] duration = 0.102 seconds
	 - PASS - [can_classic.test_set_bitrate] duration = 0.102 seconds
	 - PASS - [can_classic.test_set_bitrate_too_high] duration = 0.102 seconds
	 - PASS - [can_classic.test_set_bitrate_while_started] duration = 0.002 seconds
	 - PASS - [can_classic.test_set_mode_while_started] duration = 0.002 seconds
	 - PASS - [can_classic.test_set_state_change_callback] duration = 0.001 seconds
	 - PASS - [can_classic.test_set_timing_while_started] duration = 0.002 seconds
	 - PASS - [can_classic.test_start_while_started] duration = 0.004 seconds
	 - PASS - [can_classic.test_stop_while_stopped] duration = 0.102 seconds

	SUITE PASS - 100.00% [can_utilities]: pass = 3, fail = 0, skip = 0, total = 3 duration = 0.003
	seconds
	 - PASS - [can_utilities.test_can_bytes_to_dlc] duration = 0.001 seconds
	 - PASS - [can_utilities.test_can_dlc_to_bytes] duration = 0.001 seconds
	 - PASS - [can_utilities.test_can_frame_matches_filter] duration = 0.001 seconds

	SUITE PASS - 100.00% [canfd]: pass = 8, fail = 0, skip = 0, total = 8 duration = 0.632 seconds
	 - PASS - [canfd.test_filters_preserved_through_classic_to_fd_mode_change] duration = 0.307 seconds
	 - PASS - [canfd.test_filters_preserved_through_fd_to_classic_mode_change] duration = 0.307 seconds
	 - PASS - [canfd.test_get_capabilities] duration = 0.001 seconds
	 - PASS - [canfd.test_send_receive_classic] duration = 0.005 seconds
	 - PASS - [canfd.test_send_receive_fd] duration = 0.004 seconds
	 - PASS - [canfd.test_send_receive_mixed] duration = 0.004 seconds
	 - PASS - [canfd.test_set_bitrate_data_while_started] duration = 0.002 seconds
	 - PASS - [canfd.test_set_timing_data_while_started] duration = 0.002 seconds

	------ TESTSUITE SUMMARY END ------

	===================================================================
	PROJECT EXECUTION SUCCESSFUL

Channel1 tests/drivers/can/api
``````````````````````````````

The **tests/drivers/can/api** test is intended to perform full testing of Zephyr CAN API
interaction with RZ G3S CAN-FD driver in loopback CAN test mode.

To build **tests/drivers/can/api** test for CAN-FD channel 1 run command:

.. code-block:: bash

    west build -b rz_g3s -p always -S rz-g3s-canfd1-test tests/drivers/can/api

The **tests/drivers/can/api** test will produce below console output when executed:

.. code-block:: console

	I: can@400c0000:"CANFD IP init done ver:31240143 pclk:80000000Hz ext_clk:0Hz"
	I: channel@0:"init done"
	I: channel@1:"init done"
	*** Booting Zephyr OS build v3.5.0-rc2-299-g93cceb9be768 ***
	Running TESTSUITE can_classic
	===================================================================
	START - test_add_filter
	 PASS - test_add_filter in 0.002 seconds
	===================================================================
	START - test_filters_added_while_stopped
	 PASS - test_filters_added_while_stopped in 0.103 seconds
	===================================================================
	START - test_filters_preserved_through_bitrate_change
	 PASS - test_filters_preserved_through_bitrate_change in 0.105 seconds
	===================================================================
	START - test_filters_preserved_through_mode_change
	 PASS - test_filters_preserved_through_mode_change in 0.105 seconds
	===================================================================
	START - test_get_capabilities
	 PASS - test_get_capabilities in 0.002 seconds
	===================================================================
	START - test_get_core_clock
	 PASS - test_get_core_clock in 0.002 seconds
	===================================================================
	START - test_get_state
	 PASS - test_get_state in 0.002 seconds
	===================================================================
	START - test_max_ext_filters
	E: channel@1:"AFLADD: no free entries"
	 PASS - test_max_ext_filters in 0.008 seconds
	===================================================================
	START - test_max_std_filters
	E: channel@1:"AFLADD: no free entries"
	 PASS - test_max_std_filters in 0.008 seconds
	===================================================================
	START - test_receive_timeout
	 PASS - test_receive_timeout in 0.102 seconds
	===================================================================
	START - test_recover
	 PASS - test_recover in 0.002 seconds
	===================================================================
	START - test_recover_while_stopped
	 PASS - test_recover_while_stopped in 0.102 seconds
	===================================================================
	START - test_send_and_forget
	 PASS - test_send_and_forget in 0.003 seconds
	===================================================================
	START - test_send_callback
	 PASS - test_send_callback in 0.002 seconds
	===================================================================
	START - test_send_fd_format
	E: channel@1:"TX: unsupported frame flags 0x04"
	 PASS - test_send_fd_format in 0.005 seconds
	===================================================================
	START - test_send_invalid_dlc
	E: channel@1:"TX: frame dlc 9 exceeds maximum (8)"
	 PASS - test_send_invalid_dlc in 0.005 seconds
	===================================================================
	START - test_send_receive_ext_id
	 PASS - test_send_receive_ext_id in 0.005 seconds
	===================================================================
	START - test_send_receive_ext_id_masked
	 PASS - test_send_receive_ext_id_masked in 0.005 seconds
	===================================================================
	START - test_send_receive_ext_id_rtr
	 PASS - test_send_receive_ext_id_rtr in 0.207 seconds
	===================================================================
	START - test_send_receive_msgq
	 PASS - test_send_receive_msgq in 0.015 seconds
	===================================================================
	START - test_send_receive_std_id
	 PASS - test_send_receive_std_id in 0.005 seconds
	===================================================================
	START - test_send_receive_std_id_masked
	 PASS - test_send_receive_std_id_masked in 0.005 seconds
	===================================================================
	START - test_send_receive_std_id_rtr
	 PASS - test_send_receive_std_id_rtr in 0.206 seconds
	===================================================================
	START - test_send_receive_wrong_id
	 PASS - test_send_receive_wrong_id in 0.102 seconds
	===================================================================
	START - test_send_while_stopped
	 PASS - test_send_while_stopped in 0.102 seconds
	===================================================================
	START - test_set_bitrate
	 PASS - test_set_bitrate in 0.102 seconds
	===================================================================
	START - test_set_bitrate_too_high
	 PASS - test_set_bitrate_too_high in 0.102 seconds
	===================================================================
	START - test_set_bitrate_while_started
	 PASS - test_set_bitrate_while_started in 0.002 seconds
	===================================================================
	START - test_set_mode_while_started
	 PASS - test_set_mode_while_started in 0.002 seconds
	===================================================================
	START - test_set_state_change_callback
	 PASS - test_set_state_change_callback in 0.001 seconds
	===================================================================
	START - test_set_timing_while_started
	 PASS - test_set_timing_while_started in 0.002 seconds
	===================================================================
	START - test_start_while_started
	I: channel@1:"start: already started"
	 PASS - test_start_while_started in 0.004 seconds
	===================================================================
	START - test_stop_while_stopped
	 PASS - test_stop_while_stopped in 0.102 seconds
	===================================================================
	TESTSUITE can_classic succeeded
	Running TESTSUITE can_utilities
	===================================================================
	START - test_can_bytes_to_dlc
	 PASS - test_can_bytes_to_dlc in 0.001 seconds
	===================================================================
	START - test_can_dlc_to_bytes
	 PASS - test_can_dlc_to_bytes in 0.001 seconds
	===================================================================
	START - test_can_frame_matches_filter
	 PASS - test_can_frame_matches_filter in 0.001 seconds
	===================================================================
	TESTSUITE can_utilities succeeded
	Running TESTSUITE canfd
	===================================================================
	START - test_filters_preserved_through_classic_to_fd_mode_change
	 PASS - test_filters_preserved_through_classic_to_fd_mode_change in 0.307 seconds
	===================================================================
	START - test_filters_preserved_through_fd_to_classic_mode_change
	 PASS - test_filters_preserved_through_fd_to_classic_mode_change in 0.307 seconds
	===================================================================
	START - test_get_capabilities
	 PASS - test_get_capabilities in 0.001 seconds
	===================================================================
	START - test_send_receive_classic
	 PASS - test_send_receive_classic in 0.005 seconds
	===================================================================
	START - test_send_receive_fd
	 PASS - test_send_receive_fd in 0.004 seconds
	===================================================================
	START - test_send_receive_mixed
	 PASS - test_send_receive_mixed in 0.004 seconds
	===================================================================
	START - test_set_bitrate_data_while_started
	 PASS - test_set_bitrate_data_while_started in 0.002 seconds
	===================================================================
	START - test_set_timing_data_while_started
	 PASS - test_set_timing_data_while_started in 0.002 seconds
	===================================================================
	TESTSUITE canfd succeeded

	------ TESTSUITE SUMMARY START ------

	SUITE PASS - 100.00% [can_classic]: pass = 33, fail = 0, skip = 0, total = 33 duration = 1.527
	seconds
	 - PASS - [can_classic.test_add_filter] duration = 0.002 seconds
	 - PASS - [can_classic.test_filters_added_while_stopped] duration = 0.103 seconds
	 - PASS - [can_classic.test_filters_preserved_through_bitrate_change] duration = 0.105 seconds
	 - PASS - [can_classic.test_filters_preserved_through_mode_change] duration = 0.105 seconds
	 - PASS - [can_classic.test_get_capabilities] duration = 0.002 seconds
	 - PASS - [can_classic.test_get_core_clock] duration = 0.002 seconds
	 - PASS - [can_classic.test_get_state] duration = 0.002 seconds
	 - PASS - [can_classic.test_max_ext_filters] duration = 0.008 seconds
	 - PASS - [can_classic.test_max_std_filters] duration = 0.008 seconds
	 - PASS - [can_classic.test_receive_timeout] duration = 0.102 seconds
	 - PASS - [can_classic.test_recover] duration = 0.002 seconds
	 - PASS - [can_classic.test_recover_while_stopped] duration = 0.102 seconds
	 - PASS - [can_classic.test_send_and_forget] duration = 0.003 seconds
	 - PASS - [can_classic.test_send_callback] duration = 0.002 seconds
	 - PASS - [can_classic.test_send_fd_format] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_invalid_dlc] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_ext_id] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_ext_id_masked] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_ext_id_rtr] duration = 0.207 seconds
	 - PASS - [can_classic.test_send_receive_msgq] duration = 0.015 seconds
	 - PASS - [can_classic.test_send_receive_std_id] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_std_id_masked] duration = 0.005 seconds
	 - PASS - [can_classic.test_send_receive_std_id_rtr] duration = 0.206 seconds
	 - PASS - [can_classic.test_send_receive_wrong_id] duration = 0.102 seconds
	 - PASS - [can_classic.test_send_while_stopped] duration = 0.102 seconds
	 - PASS - [can_classic.test_set_bitrate] duration = 0.102 seconds
	 - PASS - [can_classic.test_set_bitrate_too_high] duration = 0.102 seconds
	 - PASS - [can_classic.test_set_bitrate_while_started] duration = 0.002 seconds
	 - PASS - [can_classic.test_set_mode_while_started] duration = 0.002 seconds
	 - PASS - [can_classic.test_set_state_change_callback] duration = 0.001 seconds
	 - PASS - [can_classic.test_set_timing_while_started] duration = 0.002 seconds
	 - PASS - [can_classic.test_start_while_started] duration = 0.004 seconds
	 - PASS - [can_classic.test_stop_while_stopped] duration = 0.102 seconds

	SUITE PASS - 100.00% [can_utilities]: pass = 3, fail = 0, skip = 0, total = 3 duration = 0.003
	seconds
	 - PASS - [can_utilities.test_can_bytes_to_dlc] duration = 0.001 seconds
	 - PASS - [can_utilities.test_can_dlc_to_bytes] duration = 0.001 seconds
	 - PASS - [can_utilities.test_can_frame_matches_filter] duration = 0.001 seconds

	SUITE PASS - 100.00% [canfd]: pass = 8, fail = 0, skip = 0, total = 8 duration = 0.632 seconds
	 - PASS - [canfd.test_filters_preserved_through_classic_to_fd_mode_change] duration = 0.307 seconds
	 - PASS - [canfd.test_filters_preserved_through_fd_to_classic_mode_change] duration = 0.307 seconds
	 - PASS - [canfd.test_get_capabilities] duration = 0.001 seconds
	 - PASS - [canfd.test_send_receive_classic] duration = 0.005 seconds
	 - PASS - [canfd.test_send_receive_fd] duration = 0.004 seconds
	 - PASS - [canfd.test_send_receive_mixed] duration = 0.004 seconds
	 - PASS - [canfd.test_set_bitrate_data_while_started] duration = 0.002 seconds
	 - PASS - [canfd.test_set_timing_data_while_started] duration = 0.002 seconds

	------ TESTSUITE SUMMARY END ------

	===================================================================
	PROJECT EXECUTION SUCCESSFUL
