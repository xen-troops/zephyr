tests/kernel/tickless/tickless_concept
``````````````````````````````````````

Zephyr RZ/G3S GTM as Zephyr OS timer driver can be tested by using **tests/kernel/tickless/tickless_concept** test.
To build **tests/kernel/tickless/tickless_concept** test run command:

.. code-block:: bash

   west build -p always -b rz_g3s -S rz-g3s-gtm-timer-test tests/kernel/tickless/tickless_concept

Console output:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-331-gb7a06954094c ***
    Running TESTSUITE tickless_concept
    ===================================================================
    START - test_tickless_slice
    elapsed slice 110, expected: <100, 110>
    elapsed slice 100, expected: <100, 110>
    elapsed slice 100, expected: <100, 110>
    elapsed slice 100, expected: <100, 110>
     PASS - test_tickless_slice in 0.611 seconds
    ===================================================================
    START - test_tickless_sysclock
    time 2920, 3130
    time 3140, 3340
     PASS - test_tickless_sysclock in 0.426 seconds
    ===================================================================
    TESTSUITE tickless_concept succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [tickless_concept]: pass = 2, fail = 0, skip = 0, total = 2 duration = 1.037
    ses
     - PASS - [tickless_concept.test_tickless_slice] duration = 0.611 seconds
     - PASS - [tickless_concept.test_tickless_sysclock] duration = 0.426 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

.. raw:: latex

    \newpage
