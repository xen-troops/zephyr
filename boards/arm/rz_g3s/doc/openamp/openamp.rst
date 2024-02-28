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

**NOTE** Vring addresses, passed from Zephyr side to Linux side should be converted to Cortex-A55 format
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
Please refer to the README.rst file in **openamp_rsc_table_rzg3** for the details about building
and testing.

Linux BSP build preparation
```````````````````````````

Provided sample was designed to work with the Linux build with Zephyr support for Renesas RZ/G3S
board. BSP uses yocto as build system.
Please refer to
`meta-rz-zephyr
<https://gitbud.epam.com/rec-rzzp/meta-zephyr-rz/-/blob/rzg3s_dev/README.md>`_
for download and build instructions.

.. raw:: latex

    \newpage
