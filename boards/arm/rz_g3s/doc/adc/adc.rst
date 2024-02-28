A/D Converter (ADC)
===================

ADC overview
------------

The Renesas RZ G3S Soc has a successive approximation A/D converter with a 12-bit accuracy.
Up to eight analog input channels and one dedicated channel to TSU can be selected.

* Resolution 12 bits
* Input voltage range 0 V to 1.8 V
* Minimum conversion time 1.0 µs per channel (when A/D conversion clock ADC_ADCLK is 100 MHz)
* The A/D conversion result is stored in a 32-bit data register corresponding to each channel.
* Sample-and-hold function
* Trigger mode

    * Software trigger mode and hardware trigger mode are available.
    * Software trigger mode to start A/D conversion by software
    * Hardware trigger mode to start A/D conversion by an asynchronous trigger or synchronous trigger

* Operating mode

    * Select mode and scan mode are available.
    * Select mode to convert the specified single analog input channel
    * Scan mode to convert the analog inputs of arbitrarily selected channels in ascending order of channel number

* Conversion mode

    * Single mode and repeat mode are available.
    * Single mode to proceed A/D conversion only once
    * Repeat mode to repeatedly proceed A/D conversion

Refer to "A/D Converter" section in "Renesas RZ/G3S Group User’s Manual: Hardware"

ADC driver overview
-------------------

Zephyr RZ/G3S Renesas ADC driver provides Zephyr :ref:`adc_api` System interface implementation.

The ADC subsystem is **not** enabled by default in ``rz_g3s_defconfig``. To enable Zephyr
ADC functionality and RZ G3S ADC driver below Kconfig options have to be enabled:

.. code-block:: text

    CONFIG_ADC=y
    /* optional */
    CONFIG_ADC_ASYNC=y
    /* automatically enabled */
    CONFIG_ADC_RZG3S=y

The RZ G3S ADC driver code can be found at:

.. code-block:: text

    drivers/adc/adc_rz_g3s.c

ADC testing
-----------

tests/drivers/adc/adc_api
`````````````````````````

Zephyr RZ/G3S ADC driver can be tested by using **tests/drivers/adc/adc_api** test application.
Use below command to build ADC **tests/drivers/adc/adc_api** test application:

.. code-block:: bash

    west build -b rz_g3s -p always tests/drivers/adc/adc_api

It runs several tests:

    - test_adc_asynchronous_call - ADC perform one channel reading in asynchronous mode
    - test_adc_invalid_request - the wrong configuration for the ADC sequence is set and the driver is expected to return an error
    - test_adc_repeated_samplings - periodic reading of 2 ADC channels is performed, repeat reading is requested by a user callback
    - test_adc_sample_one_channel - one channel is tested in synchronous mode
    - test_adc_sample_two_channels - all available channels specified in devicetree are tested
    - test_adc_sample_with_interval - test with 100 ms interval between samples

The **tests/drivers/adc/adc_api** sample application will produce below console output when executed:

.. code-block:: console

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
in ADC levels, if not read, the number 0xffff8000 is displayed.
Additionally, the ADC can be checked by applying voltage from 0 to 1.8 V to pins 1-8
of the ADC connector located on the RZ/G3S SMARC Module board. In this case,
the ADC values of corresponding channel will be from 0 to 0xFFF, respectively.

.. raw:: latex

    \newpage
