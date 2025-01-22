.. _arm_scmi_shell:

ARM SCMI shell sample
#####################

Overview
********

This sample demonstrates the usage of ARM SCMI. It provides access to the ARM SCMI shell interface
for demo and testing purposes of different ARM SCMI protocols:

* Base protocol
* Reset domain management protocol

Caveats
*******

Zephyr ARM SCMI relies on the firmware running in EL3 layer to be compatible
with Arm System Control and Management Interface Platform Design Document
and ARM SCMI driver used by the sample.

Building and Running
********************
For building the application:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/firmware/arm_scmi
   :board: rpi_5
   :goals: build

Running on RPI5
^^^^^^^^^^^^^^^

* The Trusted Firmware-A (TF-A) binary should be placed at RPI5 rootfs
  as `armstub8-2712.bin`.
* The Zephyr application binary should be placed at RPI5 rootfs
  as `zephyr.bin`.
* Modify config.txt parameter `kernel=zephyr.bin`

Sample Output
*************

.. code-block:: console

	I: scmi base protocol v0002.0000
	I: scmi base revision info vendor 'EPAM:' fw version 0x0 protocols:2 agents:0
	I: scmi calling method:smc
	I: scmi reset rotocol version 0x0001.0000 num_domains:4
	*** Booting Zephyr OS build v3.6.0-99-g75fb2f9016d3 ***
	ARM SCMI shell sample


	uart:~$ arm_scmi base revision \
	I: scmi base protocol v0002.0000
	I: scmi base revision info vendor 'EPAM:' fw version 0x0 protocols:2 agents:0
	ARM SCMI base protocol v0002.0000
	  vendor        :EPAM
	  subvendor     :
	  fw version    :0x0
	  protocols     :2
	  num_agents    :0
	uart:~$ arm_scmi reset list
	domain_id,name,latency,attributes
	0,swinit_pcie1,0x7fffffff,0x00000000
	1,bridge_pcie1,0x7fffffff,0x00000000
	2,swinit_pcie2,0x7fffffff,0x00000000
	3,bridge_pcie2,0x7fffffff,0x00000000
	uart:~$ arm_scmi reset info 1
	ARM SCMI reset domain: 1
	 name           : bridge_pcie1
	 latency        : unk
	 attr           : 0x00000000
	uart:~$ arm_scmi reset a
	  assert     autoreset
	uart:~$ arm_scmi reset assert 1
	uart:~$ arm_scmi reset deassert 1
