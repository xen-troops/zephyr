.. _Building yocto image:

Building yocto image
====================

.. _Linux Start-up Guide for RZ/G3S Board Support Package: https://www.renesas.com/us/en/document/mas/linux-start-guide-rzg3s-board-support-package-v100

Overview
--------

RZ/G3S board is designed to start different systems on cores. It uses Yocto as the build system
to build Linux system and BootLoaders to run Linux on CA55 core.

The provided Yocto layer is an enhancement on top of Reneass FSP build see `Linux Start-up Guide for RZ/G3S Board Support Package`_.
And it will build the following artifacts in the directory `build/tmp/deploy/images/smarc-rzg3s`:

* RZ/G3S Linux kernel: Image-smarc-rzg3s.bin
* Device tree file: Image-r9a08g045s33-smarc.dtb
* root filesystem: <image name>-smarc-rzg3s.tar.bz2
* L2 Boot loader for SPI: bl2_bp_spi-smarc-rzg3s.srec
* L2 Boot loader for eMMC: bl2_bp_emmc-smarc-rzg3s.srec
* L3 Boot loader: fip-smarc-rzg3s.srec
* Flash Writer: FlashWriter-smarc-rzg3s.mot
* SD image: core-image-<image name> -smarc-rzg3s.wic.gz
* BMAP image: core-image-<image name> -smarc-rzg3s.wic.bmap

``NOTE`` <image-name> can be either `core-image-minimal` or `core-image-bsp`.

The top-level meta-layer is `meta-rz-zephyr`, please see README.md file for details: `meta-rz-zephyr`_.

.. _meta-rz-zephyr: https://gitbud.epam.com/rec-rzzp/meta-zephyr-rz/-/blob/rzg3s_dev/README.md

Compatibility
-------------

This Yocto build is based on top of RZ/G3S Board Support Package Version 1.0.0 (`Linux Start-up Guide for RZ/G3S Board Support Package`_)
with ``meta-rz-features`` layer provided with Multi-OS Package v2.0.0. Please note that current release is based
on Multi-OS Package v2.0.0 and do not support Multi-OS-Package v2.0.1.

Current Yocto repo provides the following changes to the Renesas yocto recipes:

* trusted-firmware-a: Add `PLAT_M33_BOOT_SUPPORT=1` to start CM33 core on boot;
* rpmsg-sample_0.1: Updates to rpmsg-sample code to work with newer versions of OpenAMP;
* linux-renesas_5.10: Update kernel CPG driver to enable SCIF and OSTM clocks for Zephyr;
* u-boot_2021.10: Disable sdhi2 device as it shares scif1 pin used by Cortex-M33 core.

Getting started with Yocto
--------------------------

Initialize yocto repository:

.. code-block:: bash

    repo init -u git@gitbud.epam.com:rec-rzzp/rzg3s-manifest.git -b manifest_init
    repo sync

``NOTE`` Yocto initialization using repo for manifests. Please follow the link for repo installation details: `repo Install`_.

.. _repo Install: https://gerrit.googlesource.com/git-repo#install

Before building please install all needed packages, provided in *Section 2.1 Building images*
in `Linux Start-up Guide for RZ/G3S Board Support Package`_.

Initializate bitbake environment using command:

.. code-block:: bash

    TEMPLATECONF=$PWD/meta-zephyr-rz/docs/template/conf  source poky/oe-init-build-env

Start the build:

.. code-block:: bash

    MACHINE=smarc-rzg3s bitbake core-image-minimal

The above commands will clone all necessary repositories, generate necessary configuration files
and run build.

All built files will be located in directory `/build/tmp/deploy/images/smarc-rzg3s`.

**NOTE** BL2 TF-A image will be built with `PLAT_M33_BOOT_SUPPORT` feature to load image to CM33 core on boot.

.. _Flash loaders:

Flash loaders to the target board
---------------------------------

To Flash bootloaders to the target board please follows this steps:

* Switch the board to SCIF Download mode. Refer to :ref:`SCIF boot`.
See section 4.1.1 of `Linux Start-up Guide for RZ/G3S Board Support Package`_ for details.

* Connect to the board minicom consoles using minicom (:ref:`Linux minicom terminal`);
* When the following message appear:

.. code-block:: bash

    SCI Download mode (Notmal SCI boot)
    -- Load Program to SRAM -----------------

* Use minicom `ASCII` mode to upload FlashWriter (FlashWriter-smarc-rzg3s.mot).
Refer to :ref:`Upload with minicom`;

* After upload the following message appear:

.. code-block:: bash

    Flash writer for RZ/G3S Series
    Product Code : RZ/G3S

* Then upload images to either qSPI or eMMC (see below).

Flash to qSPI
`````````````

* use XLS2 command to upload bl2 image:

.. code-block:: bash

    >XLS2
    ===== Qspi writing of RZ/G3 Board Command =============
    Load Program to Spiflash
    Writes to any of SPI address.
    Program size & Qspi Save Address
    ===== Please Input Program Top Address ============
    Please Input : H'a1e00
    ===== Please Input
    Qspi Save Address ===
    Please Input : H'0
    please send ! ('.' & CR stop load)

* upload bl2_bp_spi-smarc-rzg3s.srec from minicom. Refer to :ref:`Upload with minicom`;
* after successful download the following output will appear:

.. code-block:: bash

    Erase SPI Flash memory...
    Erase Completed
    Write to SPI Flash memory.
    ======= Qspi Save Information =================
    SpiFlashMemory
    Stat Address : H'00000000
    SpiFlashMemory
    End Address : H'0001BCCF
    ================================================

* use XLS2 command to upload fip image:

.. code-block:: console

    >XLS2
    ===== Qspi writing of RZ/G3 Board Command =============
    Load Program to Spiflash
    Writes to any of SPI address.
    Program size & Qspi Save Address
    ===== Please Input Program Top Address ============
    Please Input : H’0
    ===== Please Input Qspi Save Address ===
    Please Input : H'64000
    please send ! ('.' & CR stop load)

* upload fip-smarc-rzg3s.srec from minicom. Refer to :ref:`Upload with minicom`;
* after successful download the following output will appear:

.. code-block:: bash

    Erase SPI Flash memory...
    Erase Completed
    Write to SPI Flash memory.
    ======= Qspi Save Information =================
    SpiFlashMemory Stat Address : H'00064000
    SpiFlashMemory End Address : H'0014782E
    ===========================================================

* Switch the board to qSPI boot mode. See section 4.1.1 of `Linux Start-up Guide for RZ/G3S Board Support Package`_;

Flash to eMMC
`````````````

* use EM_W command to upload bl2 image:

.. code-block:: console

    >EM_W
    EM_W Start --------------
    ---------------------------------------------------------
    Please select,eMMC Partition Area.
    0:User Partition Area : 62160896 KBytes
    eMMC Sector Cnt : H'0 - H'0768FFFF
    1:Boot Partition 1 : 32256 KBytes
    eMMC Sector Cnt : H'0 - H'0000FBFF
    2:Boot Partition 2 : 32256 KBytes
    eMMC Sector Cnt : H'0 - H'0000FBFF
    ---------------------------------------------------------
    Select area(0-2)>1
    -- Boot Partition 1 Program -----------------------------
    Please Input Start Address in sector :1
    Please Input Program Start Address : a1e00
    Work RAM (H'00020000-H'000FFFFF) Clear....
    please send ! ('.' & CR stop load)

* upload bl2_bp_emmc-smarc-rzg3s.srec from minicom. Refer to :ref:`Upload with minicom`;
* after successful download the following output will appear:

.. code-block:: console

    SAVE -FLASH.......
    EM_W Complete

* use EM_W command to upload fip image:

.. code-block:: console

    > EM_W
    EM_W Start --------------
    ---------------------------------------------------------
    Please select,eMMC Partition Area.
    0:User Partition Area : 62160896 KBytes
    eMMC Sector Cnt : H'0 - H'0768FFFF
    1:Boot Partition 1 : 32256 KBytes
    eMMC Sector Cnt : H'0 - H'0000FBFF
    2:Boot Partition 2 : 32256 KBytes
    eMMC Sector Cnt : H'0 - H'0000FBFF
    ---------------------------------------------------------
    Select area(0-2)>1
    -- Boot Partition 1 Program -----------------------------
    Please Input Start Address in sector :320
    Please Input Program Start Address : 0
    Work RAM(H'00020000-H'000FFFFF) Clear....
    please send ! ('.' & CR stop load)

* upload fip-smarc-rzg3s.srec from minicom. Refer to :ref:`Upload with minicom`;
* after successful download the following output will appear:

.. code-block:: console

    SAVE -FLASH.......
    EM_W Complete!

* set EXT_CSD register:

.. code-block:: console

    >em_secsd
    Please Input EXT_CSD Index(H'00 - H'1FF) : b1
    EXT_CSD[B1] = 0x02
    Please Input Value(H'00 - H'FF) : 2
    EXT_CSD[B1] = 0x02
    >em_secsd
    Please Input EXT_CSD Index(H'00 - H'1FF) : b3
    EXT_CSD[B3] = 0x09
    Please Input Value(H'00 - H'FF) : 8
    EXT_CSD[B3] = 0x08
    >

* Switch the board to eMMC boot mode. See section 4.1.1 of `Linux Start-up Guide for RZ/G3S Board Support Package`_;

.. _Upload with minicom:

Upload with minicom
```````````````````

For upload file with mimicom press `Ctrl+a` then `s`. In menu choose `ascii` then file to upload.

.. _Start Linux:

Starting Linux on the target board
----------------------------------

This step is optional for all Zephyr tests referenced in this document, except for :ref:`rz_g3s_openamp`.

For :ref:`rz_g3s_openamp` this step is mandatory.

To load Linux rootfs microSD card should be used.

Please plug in micro SD card to your PC and then execute:

.. code-block:: bash

    sudo bmaptool copy core-image-minimal-smarc-rzg3s.wic.gz /dev/sda

From `/build/tmp/deploy/images/smarc-rzg3s` folder where `/dev/sda` is the device associated with microSD card.
See section 3.1 of `Linux Start-up Guide for RZ/G3S Board Support Package`_ for details.

Then plug this microSD card to slot on carry board. Schema is provided in Section 5 of `Linux Start-up Guide for RZ/G3S Board Support Package`_.

Hit reset.
