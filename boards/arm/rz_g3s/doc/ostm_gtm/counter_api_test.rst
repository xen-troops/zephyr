tests/drivers/counter/counter_basic_api
```````````````````````````````````````

The **tests/drivers/counter/counter_basic_api** test is intended to perform full testing of
Zephyr Counter API interaction with RZ G3S GTM/OSTM Counter driver.

To build **tests/drivers/counter/counter_basic_api** test run command:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/counter/counter_basic_api

The **tests/drivers/counter/counter_basic_api** test will produce below console output when executed:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-348-g43d0f3af38ae ***
    Running TESTSUITE counter_basic
    ===================================================================
    START - test_all_channels
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_all_channels in 1.054 seconds
    ===================================================================
    START - test_cancelled_alarm_does_not_expire
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_cancelled_alarm_does_not_expire in 8.146 seconds
    ===================================================================
    START - test_late_alarm
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_late_alarm in 0.815 seconds
    ===================================================================
    START - test_late_alarm_error
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_late_alarm_error in 0.814 seconds
    ===================================================================
    START - test_multiple_alarms
    Skipped for timer@42801000
    Skipped for timer@42801400
    Skipped for timer@42801800
    Skipped for timer@42801c00
    Skipped for timer@42802000
    Skipped for timer@42802400
    Skipped for timer@42802800
    Skipped for timer@42802c00
     PASS - test_multiple_alarms in 0.816 seconds
    ===================================================================
    START - test_set_top_before_start_with_alarm
    Testing timer@42801000
    E: timer@42801000:"alarm: not supported in interval mode"
    Testing timer@42801400
    E: timer@42801400:"alarm: not supported in interval mode"
    Testing timer@42801800
    E: timer@42801800:"alarm: not supported in interval mode"
    Testing timer@42801c00
    E: timer@42801c00:"alarm: not supported in interval mode"
    Testing timer@42802000
    E: timer@42802000:"alarm: not supported in interval mode"
    Testing timer@42802400
    E: timer@42802400:"alarm: not supported in interval mode"
    Testing timer@42802800
    E: timer@42802800:"alarm: not supported in interval mode"
    Testing timer@42802c00
    E: timer@42802c00:"alarm: not supported in interval mode"
     PASS - test_set_top_before_start_with_alarm in 2.008 seconds
    ===================================================================
    START - test_set_top_before_start_with_callback
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_set_top_before_start_with_callback in 1.646 seconds
    ===================================================================
    START - test_set_top_value_with_alarm
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_set_top_value_with_alarm in 1.686 seconds
    =================================================e=================
    START - test_set_top_with_callback_two_times
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_set_top_with_callback_two_times in 2.639 seconds
    ===================================================================
    START - test_short_relative_alarm
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_short_relative_alarm in 0.880 seconds
    ===================================================================
    START - test_single_shot_alarm_notop
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_single_shot_alarm_notop in 1.374 seconds
    ===================================================================
    START - test_single_shot_alarm_top
    Skipped for timer@42801000
    Skipped for timer@42801400
    Skipped for timer@42801800
    Skipped for timer@42801c00
    Skipped for timer@42802000
    Skipped for timer@42802400
    Skipped for timer@42802800
    Skipped for timer@42802c00
     PASS - test_single_shot_alarm_top in 0.816 seconds
    ===================================================================
    TESTSUITE counter_basic succeeded
    Running TESTSUITE counter_no_callback
    ===================================================================
    START - test_set_top_value_without_alarm
    Testing timer@42801000
    Testing timer@42801400
    Testing timer@42801800
    Testing timer@42801c00
    Testing timer@42802000
    Testing timer@42802400
    Testing timer@42802800
    Testing timer@42802c00
     PASS - test_set_top_value_without_alarm in 0.857 seconds
    ===================================================================
    TESTSUITE counter_no_callback succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [counter_basic]: pass = 12, fail = 0, skip = 0, total = 12 duration = 22.694 seconds
     - PASS - [counter_basic.test_all_channels] duration = 1.054 seconds
     - PASS - [counter_basic.test_cancelled_alarm_does_not_expire] duration = 8.146 seconds
     - PASS - [counter_basic.test_late_alarm] duration = 0.815 seconds
     - PASS - [counter_basic.test_late_alarm_error] duration = 0.814 seconds
     - PASS - [counter_basic.test_multiple_alarms] duration = 0.816 seconds
     - PASS - [counter_basic.test_set_top_before_start_with_alarm] duration = 2.008 seconds
     - PASS - [counter_basic.test_set_top_before_start_with_callback] duration = 1.646 seconds
     - PASS - [counter_basic.test_set_top_value_with_alarm] duration = 1.686 seconds
     - PASS - [counter_basic.test_set_top_with_callback_two_times] duration = 2.639 seconds
     - PASS - [counter_basic.test_short_relative_alarm] duration = 0.880 seconds
     - PASS - [counter_basic.test_single_shot_alarm_notop] duration = 1.374 seconds
     - PASS - [counter_basic.test_single_shot_alarm_top] duration = 0.816 seconds

    SUITE PASS - 100.00% [counter_no_callback]: pass = 1, fail = 0, skip = 0, total = 1 duration = 0.857 seconds
     - PASS - [counter_no_callback.test_set_top_value_without_alarm] duration = 0.857 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL
