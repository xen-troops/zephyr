ADC support
===========
Zephyr provides API for the analog-to-digital converter interface. This can be checked using the following test:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/adc/adc_api/

It runs several tests:
    - test_adc_asynchronous_call - ADC perform one channel reading in asynchronous mode
    - test_adc_invalid_request - the wrong configuration for the ADC sequence is set and the driver is expected to return an error
    - test_adc_repeated_samplings - periodic reading of 2 ADC channels is performed, repeat reading is requested by a user callback
    - test_adc_sample_one_channel - one channel is tested in synchronous mode
    - test_adc_sample_two_channels - all available channels specified in devicetree are tested
    - test_adc_sample_with_interval - test with 100 ms interval between samples

The result of the test will be the following:

::

    *** Booting Zephyr OS build v3.5.0-rc2-280-ge76b6c593adc ***
    Running TESTSUITE adc_basic
    ===================================================================
    START - test_adc_asynchronous_call
    Samples read: 0x0b96 0x0caf 0x0dc2 0x0ecb 0x0fd6 0xffff8000
     PASS - test_adc_asynchronous_call in 0.007 seconds
    ===================================================================
    START - test_adc_invalid_request
    E: adc@50058000: supported only 12-bit resolution
    E: adc@50058000: supported only 12-bit resolution
    Samples read: 0x0fe2 0xffff8000 0xffff8000 0xffff8000 0xffff8000 0xffff8000
     PASS - test_adc_invalid_request in 0.016 seconds
    ===================================================================
    START - test_adc_repeated_samplings
    repeated_samplings_callback: done 1
    Samples read: 0x0fff 0x0b38 0xffff8000 0xffff8000 0xffff8000 0xffff8000
    repeated_samplings_callback: done 2
    Samples read: 0x0fff 0x0b38 0x0f65 0x0c3b 0xffff8000 0xffff8000
    repeated_samplings_callback: done 3
    Samples read: 0x0fff 0x0b38 0x0f61 0x0cfb 0xffff8000 0xffff8000
    repeated_samplings_callback: done 4
    Samples read: 0x0fff 0x0b38 0x0f86 0x0da5 0xffff8000 0xffff8000
    repeated_samplings_callback: done 5
    Samples read: 0x0fff 0x0b38 0x0fb6 0x0e0f 0xffff8000 0xffff8000
    repeated_samplings_callback: done 6
    Samples read: 0x0fff 0x0b38 0x0fed 0x0e8c 0xffff8000 0xffff8000
    repeated_samplings_callback: done 7
    Samples read: 0x0fff 0x0b38 0x0fff 0x0ee4 0xffff8000 0xffff8000
    repeated_samplings_callback: done 8
    Samples read: 0x0fff 0x0b38 0x0fff 0x0f0e 0xffff8000 0xffff8000
    repeated_samplings_callback: done 9
    Samples read: 0x0fff 0x0b38 0x0fff 0x0f3d 0xffff8000 0xffff8000
    repeated_samplings_callback: done 10
    Samples read: 0x0fff 0x0b38 0x0fff 0x0f48 0xffff8000 0xffff8000
     PASS - test_adc_repeated_samplings in 0.091 seconds
    ===================================================================
    START - test_adc_sample_one_channel
    Samples read: 0x0ff0 0xffff8000 0xffff8000 0xffff8000 0xffff8000 0xffff8000
     PASS - test_adc_sample_one_channel in 0.008 seconds
    ===================================================================
    START - test_adc_sample_two_channels
    Samples read: 0x0fff 0x0ef7 0x0725 0xffff8000 0xffff8000 0xffff8000
     PASS - test_adc_sample_two_channels in 0.007 seconds
    ===================================================================
    START - test_adc_sample_with_interval
    sample_with_interval_callback: sampling 0
    sample_with_interval_callback: sampling 1
    sample_with_interval_callback: sampling 2
    sample_with_interval_callback: sampling 3
    sample_with_interval_callback: sampling 4
    Samples read: 0x0e2f 0x0ea6 0x0ec6 0x0ee8 0x0ee9 0xffff8000
     PASS - test_adc_sample_with_interval in 0.410 seconds
    ===================================================================
    TESTSUITE adc_basic succeeded
    ------ TESTSUITE SUMMARY START ------
    SUITE PASS - 100.00% [adc_basic]: pass = 6, fail = 0, skip = 0, total = 6 durats
     - PASS - [adc_basic.test_adc_asynchronous_call] duration = 0.007 seconds
     - PASS - [adc_basic.test_adc_invalid_request] duration = 0.016 seconds
     - PASS - [adc_basic.test_adc_repeated_samplings] duration = 0.091 seconds
     - PASS - [adc_basic.test_adc_sample_one_channel] duration = 0.008 seconds
     - PASS - [adc_basic.test_adc_sample_two_channels] duration = 0.007 seconds
     - PASS - [adc_basic.test_adc_sample_with_interval] duration = 0.410 seconds
    ------ TESTSUITE SUMMARY END ------
    ===================================================================
    PROJECT EXECUTION SUCCESSFUL

Sample data is output in the following format: if a sample is read, its value is displayed
in ADC levels, if not read, the number 0xffff8000 is displayed. Additionally, the ADC can be checked by applying voltage from 0 to 1.8 V to pins 1-8
of the ADC connector located on the RZ/G3S SMARC Module board. In this case,
the ADC values of corresponding channel will be from 0 to 0xFFF, respectively.
