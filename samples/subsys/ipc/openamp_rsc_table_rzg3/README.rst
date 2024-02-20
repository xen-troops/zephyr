.. zephyr:code-sample:: openamp-rsc-table-rzg3
   :name: OpenAMP using resource table for RZ/G boards
   :relevant-api: ipm_interface

   Send messages between two cores using OpenAMP and a resource table.

Overview
********

This is sample implementation of the client OpenAMP application.
This application demonstrates how to use OpenAMP with Zephyr based on a resource_table
with RZ/G Multi-OS Package v2.0.0.
It is designed to respond for the requests from RPMSG client sample
application provided by Renesas RZ/G BSP.

This sample implementation is compatible with platforms that embed
a Linux kernel OS on the main processor and a Zephyr application on
the co-processor.

Building the application
*************************

Zephyr
-------

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/ipc/openamp_rsc_table_rzg3
   :goals: test

Linux
------

Start linux system on Cortex-A55 core using BSP images provided by
Renesas BSP pack.

Running the sample
*******************

Zephyr console
---------------

Open a serial terminal (minicom, putty, etc.) and connect the board with the
following settings:

- Speed: 115200
- Data: 8 bits
- Parity: None
- Stop bits: 1

Reset the board.

Linux console
---------------

Open a Linux shell (minicom, ssh, etc.) and run the following command:

.. code-block:: console

   root@linuxshell: rpmsg_client_sample 0 0

Result on Zephyr console on boot
--------------------------------

The following message will appear on the corresponding Zephyr console:

.. code-block:: console

   ***** Booting Zephyr OS v#.##.#-####-g########## *****
   [00:00:00.000,000] <inf> openamp_rsc_table: Starting application threads!

   [00:00:00.000,000] <inf> openamp_rsc_table:
   OpenAMP[remote]  linux responder demo started

   [00:00:02.520,000] <inf> openamp_rsc_table: new_service_cb: message received0

   [00:00:02.520,000] <inf> openamp_rsc_table:
   OpenAMP[remote] Linux sample client responder started

   [00:00:03.341,000] <inf> openamp_rsc_table: OpenAMP demo ended
