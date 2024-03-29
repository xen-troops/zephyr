.. _rz_g3s_openamp:

OpenAMP
=======

Overview
--------

OpenAMP (Open Asymmetric Multi-Processing) is a framework providing the software components needed to enable
the development of software applications for AMP systems. It allows operating systems to interact within a
broad range of complex homogeneous and heterogeneous architectures and allows asymmetric multiprocessing
applications to leverage parallelism offered by the multi-core configuration.

The framework provides the following key capabilities.

- Provides Life Cycle Management, and Inter Processor Communication capabilities for management of remote
  compute resources and their associated software contexts.
- Provides a standalone library usable with RTOS and Baremetal software environments.
- Compatibility with upstream Linux remoteproc, rpmsg, and VirtIO component

OpenAMP driver testing
----------------------

TheOpenAMP sample code can be found at:

.. code-block:: text

    samples/subsys/ipc/openamp_rsc_table_rzg3

This application demonstrates how to use OpenAMP with Zephyr based on a resource_table
with RZ/G Multi-OS Package v2.0.0.
It is designed to respond for the requests from RPMSG client sample
application provided by Renesas RZ/G BSP.

This sample implementation is compatible with platforms that embed
a Linux kernel OS on the main processor and a Zephyr application on
the co-processor.

Design overview
```````````````
The provided sample introduces Renesas approach implementation.
The following shared memory for OpenAMP communication was reserved:

+-------------------------+---------+
| Cortex-M33 Address table| Name    |
+-------------------------+---------+
| ...                     |         |
+-------------------------+---------+
| 0x62F00000 - 0x63500000 | OpenAMP |
+-------------------------+---------+
| ...                     |         |
+-------------------------+---------+

Where OpenAMP region divided for the following sub-regions:

+-------------------------+----------------+
| OpenAMP Address table   | Name           |
+-------------------------+----------------+
| 0x62F00000 - 0x62F01000 | resource_table |
+-------------------------+----------------+
| ...                     |                |
+-------------------------+----------------+
| 0x62F01000 - 0x62F01008 | MHU shared mem |
+-------------------------+----------------+
| ...                     |                |
+-------------------------+----------------+
| 0x63000000 - 0x63005000 | VRING0 control |
+-------------------------+----------------+
| 0x63005000 - 0x63010000 | VRING1 control |
+-------------------------+----------------+
| ...                     |                |
+-------------------------+----------------+
| 0x63200000 - 0x63500000 | VRING shmem    |
+-------------------------+----------------+

This regions are used by OpenAMP library internals to
perform interprocess communication between Cortex-A55 and Cortex-M33/Cortex-M33_FPU cores.
On Renesas RZ/G boards the shared memory between CPU are mapped in different address spaces
maps the following way:

.. code-block:: text

   0x40000000 - 0x4fffffff on A55 to
   0x60000000 - 0x6fffffff on M33/M33_FPU cores.

Current implementation uses the same shared region for both Cortex-M33 and Cortex-M33_FPU cores,
as was mentioned in the document **r01an5869ej0200-rzg-cm33-multi-os-pkg.pdf** in
Multi-OS Package v2.0.0. And uses MHU ch1 to communicate with Cortex-M33 and MHU ch0 to communicate
with Cortex-M33_FPU core.

The provided sample using resource_table to pass shared memory information from Virtio device to
Virtio driver (Linux side). By default Zephyr expects resource_table to be retrieved from the
Driver side, that's why another resource_table implementation was used.

.. note::

    Vring addresses, passed from Zephyr side to Linux side should be converted to Cortex-A55 format
    that's why `CM33_TO_A55_ADDR_S` and `CM33_TO_A55_ADDR_NS` macros were introduced for Secure and
    Non-Secure address conversion. Same is applicable for shared memory addresses.

On initialization stage OpenAMP requests for the resource table:

.. code-block:: C

   rsc_table_get(&rsc_tab_addr, &rsc_size);

which is placed in the corresponding linker section and then copy to the resource table region:

.. code-block:: C

   memcpy((void *)RSC_TABLE_ADDR, rsc_tab_addr, rsc_size);

Where RSC_TABLE_ADDR is start address of the resource_memory shared region.

Vring addresses should be registered in metal_virtio using Cortex-M33 addresses of vring0 and vring1 regions.

From Linux side the following command should be used to initiate the communication

.. code-block:: bash

   rpmsg_client_sample 0 0

to access Cortex-M33 core and

.. code-block:: bash

   rpmsg_client_sample 0 1

to access Cortex-M33_FPU core.

Building sample application
```````````````````````````

This application can be built for **rz_g3s** board to start on Cortex-M33 and for **rz_g3s_fpu** board to
start on Cortex-M33_FPU core.

Please use the following command to build for Cortex-M33 core:

.. code-block:: bash

    west build -p always -b rz_g3s samples/subsys/ipc/openamp_rsc_table_rzg3

and for Cortex-M33_FPU core:

.. code-block:: bash

    west build -p always -b rz_g3s_fpu samples/subsys/ipc/openamp_rsc_table_rzg3

After building `openamp_rsc_table_rzg3` sample it can be started on JLink using :ref:`Debugging` instructions or
flashed to either eMMC or qSPI using instructions from :ref:`Flashing on eMMC` and :ref:`Flashing on qSPI`.

Please use the following **srec** file generated on build:

.. code-block:: bash

   <build_dir>/zephyr/openamp_rsc_table_rzg3.srec

Linux BSP build preparation
```````````````````````````

Provided sample was designed to work with the Linux build with Zephyr support for Renesas RZ/G3S
board. BSP uses Yocto as build system.
Please refer to :ref:`Building yocto image` for download and build instructions.

Test scenario
`````````````

After the preparation stage the following system configuration is expected:

* Cortex-A55 is booted to Linux and the following lines are shown on Cortex-A55 console:

.. code-block:: console

   BSP: RZG3S/RZG3S-SMARC-EVK/1.0.0
   LSI: RZG3S
   Version: 1.0.0
   smarc-rzg3s login: root
   [   23.035550] audit: type=1006 audit(1600598656.996:2): pid=172 uid=0 old-auid=4294967295 auid=0 tty=(none) old-ses=4294967295 ses=1 res=1
   root@smarc-rzg3s:~#

Use `root` user to login.

* `openamp_rsc_table_rzg3` loaded to either CM33 or CM33_FPU core or loaded for qSPI/eMMC see :ref:`Flash loaders`

.. note::

    When starting `openamp_rsc_table_rzg3` on Cortex-M33_FPU please assure that qSPI and eMMC doesn't flashed with `openamp_rsc_table_rzg3`.
    If it is please reflash it with :ref:`hello_world` sample.

The following lines should appear on Cortex-M33/M33_FPU console:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-423-g6f6e7f30b321 ***
    I: Starting application..!
    I: Starting application threads!
    I: OpenAMP[remote]  linux responder demo started

* From Linux console please run the following commands:

.. code-block:: bash

   rpmsg_client_sample 0 0

to start test with Cortex-M33 core and

.. code-block:: bash

   rpmsg_client_sample 0 1

to start test with Cortex-M33_FPU core.

* On Linux console the following output should appear:

.. code-block:: console

    root@smarc-rzg3s:~# rpmsg_sample_client 0 0
    Successfully probed IPI device
    metal: info:      metal_uio_dev_open: No IRQ for device 42f00000.rsctbl.
    Successfully open uio device: 42f00000.rsctbl.
    Successfully added memory device 42f00000.rsctbl.
    metal: info:      metal_uio_dev_open: No IRQ for device 43000000.vring-ctl0.
    Successfully open uio device: 43000000.vring-ctl0.
    Successfully added memory device 43000000.vring-ctl0.
    metal: info:      metal_uio_dev_open: No IRQ for device 43200000.vring-shm0.
    Successfully open uio device: 43200000.vring-shm0.
    Successfully added memory device 43200000.vring-shm0.
    metal: info:      metal_uio_dev_open: No IRQ for device 42f01000.mhu-shm.
    Successfully open uio device: 42f01000.mhu-shm.
    Successfully added memory device 42f01000.mhu-shm.
    Initialize remoteproc successfully.
    creating remoteproc virtio
    initializing rpmsg shared buffer pool
    initializing rpmsg vdev
     1 - Send data to remote core, retrieve the echo and validate its integrity ..
    Remote proc init.
    RPMSG endpoint has created.
    RPMSG service has created.
    sending payload number 0 of size 17
    echo test: sent : 17
     received payload number 0 of size 17
    sending payload number 1 of size 18
    echo test: sent : 18
     received payload number 1 of size 18
    sending payload number 2 of size 19
    echo test: sent : 19

    [snip]

    sending payload number 471 of size 488
    echo test: sent : 488
     received payload number 471 of size 488
    ************************************
     Test Results: Error count = 0
    ************************************
    Quitting application .. Echo test end
    Stopping application...
    root@smarc-rzg3s:~#

* On Cortex-M33/M33_FPU console the following output should appear:

.. code-block:: console

    *** Booting Zephyr OS build v3.5.0-rc2-423-g6f6e7f30b321 ***
    I: Starting application..!
    I: Starting application threads!
    I: OpenAMP[remote]  linux responder demo started
    I: new_service_cb: message received from service rpmsg-service-0
    I: OpenAMP[remote] Linux sample client responder started
    I: OpenAMP demo ended
    I: OpenAMP Linux sample client responder ended
    I: Starting application threads!
    I: OpenAMP[remote]  linux responder demo started
    I: new_service_cb: message received from service rpmsg-service-0
    I: OpenAMP[remote] Linux sample client responder started
    I: OpenAMP demo ended
    I: OpenAMP Linux sample client responder ended
    I: Starting application threads!
    I: OpenAMP[remote]  linux responder demo started

.. raw:: latex

    \newpage
