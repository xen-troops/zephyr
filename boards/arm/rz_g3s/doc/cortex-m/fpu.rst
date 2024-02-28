Cortex-M33 Floating-Point Unit (FPU)
====================================

FPU overview
------------

The Renesas RZ G3S SoC has one Cortex-M33_FPU (r0p4) System Core which has FPU implemented.


The Cortex-M33 Floating-Point Unit (FPU) implements the FPv5 floating-point extensions.
The FPU fully supports single-precision add, subtract, multiply, divide, multiply and
accumulate, and square root operations.
It also provides conversions between fixed-point and floating-point data formats,
and floating-point constant instructions.

The FPU provides floating-point computation functionality that is compliant with
the ANSI/IEEE Std 754-2008, IEEE Standard for Binary Floating-Point Arithmetic,
referred to as the IEEE 754 standard.

The FPU contains 32 single-precision extension registers, which you can also access
as 16 doubleword registers for load, store, and move operations.

Refer to Arm® Cortex®-M33 Processor Revision: r0p4 Technical Reference Manual
for more information.

FPU default configuration
-------------------------

To build Zephyr for the Cortex-M33_FPU core the following board should be used: **rz_g3s_fpu**.
So target application can be built using command:

.. code-block:: bash

    west build -b rz_g3s_fpu -p always <APP_PATH>

.. note::

    Refer to :ref:`rz_g3s_prog_debug` for more information about Cortex-M33_FPU System Core build instructions.

The following configuration parameters will be automatically enabled:

.. code-block:: text

    CONFIG_FPU=y
    #if MULTITHREADING=y
    CONFIG_FPU_SHARING=y

FPU testing
-----------

tests/kernel/fpu_sharing/generic
````````````````````````````````

To build **fpu_sharing/generic** test run command:

.. code-block:: bash

    west build -b rz_g3s_fpu -p always tests/kernel/fpu_sharing/generic

Once **fpu_sharing/generic** is loaded it will provide console output showing
the test execution process:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-230-gc935cfadb73a ***
    Running TESTSUITE fpu_sharing_generic
    ===================================================================
    START - test_load_store
    Load and store OK after 0 (high) + 2 (low) tests
    Load and store OK after 100 (high) + 239 (low) tests
    Load and store OK after 200 (high) + 476 (low) tests
    Load and store OK after 300 (high) + 714 (low) tests
    Load and store OK after 400 (high) + 952 (low) tests
    Load and store OK after 500 (high) + 1188 (low) tests
     PASS - test_load_store in 0.723 seconds
    ===================================================================
    START - test_pi
    Pi calculation OK after 50 (high) + 0 (low) tests (computed 3.141598)
    Pi calculation OK after 150 (high) + 2 (low) tests (computed 3.141598)
    Pi calculation OK after 250 (high) + 3 (low) tests (computed 3.141598)
    Pi calculation OK after 350 (high) + 5 (low) tests (computed 3.141598)
    Pi calculation OK after 450 (high) + 6 (low) tests (computed 3.141598)
     PASS - test_pi in 16.591 seconds
    ===================================================================
    TESTSUITE fpu_sharing_generic succeeded

    ------ TESTSUITE SUMMARY START ------

    SUITE PASS - 100.00% [fpu_sharing_generic]: pass = 2, fail = 0, skip = 0, total = 2 duration = 17.314s
     - PASS - [fpu_sharing_generic.test_load_store] duration = 0.723 seconds
     - PASS - [fpu_sharing_generic.test_pi] duration = 16.591 seconds

    ------ TESTSUITE SUMMARY END ------

    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

.. raw:: latex

    \newpage
