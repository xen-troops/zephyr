.. _rzg3s_pinctrl_label:

PINCTRL/PINMUX
==============

PINCTRL overview
----------------

Renesas RZ G3S (r9a08g045) SoC series has combined Pin Function and GPIO Controller module.

The  Pin Function provides possibility to configure:

* 82 GPIO pins function and parameters. GPIO pins can work as GPIO IN/OUT pin or as peripheral function pins;

    * function: GPIO or Peripheral;
    * Pull-Up/Pull-Down;
    * Driving Ability;
    * Digital Noise Filter;
    * input GPIO pin Interrupt Enable

* Special Purpose pins parameters - functions of such pins is fixed (NMI, I3C, AUDIO_CLK1 pins as an example)

    * Driving Ability;
    * Digital Noise Filter;
    * Input enable;
    * Output enable;

* Group of pins, such ETHx, SD_CHx, XSPI, I3C. For Group of pins the applied configuration affect all pins in group

    * IO voltage mode
    * standby mode (I3C)
    * XSPI Hi-Z pin state

Refer to "General Purpose Input Output Port (GPIO)" section in "Renesas RZ/G3S Group User’s Manual: Hardware"

Refer to Renesas "RZG3S_pinfunction_List_r1.1.xls"

PINCTRL driver overview
-----------------------

The RZ G3S PINCTRL driver provides Zephyr :ref:`pinctrl_api` System interface implementation.
The RZ G3S PINCTRL driver is defined in Device tree as pin-controller (dts/arm/renesas/rz/r9a08g045.dtsi).

.. code-block:: dts

    pinctrl: pin-controller@41030000 {
        compatible = "renesas,r9a08g045-pinctrl";
        reg = <0x41030000 DT_SIZE_K(64)>;
        reg-names = "pinctrl";
        #address-cells = <1>;
        #size-cells = <1>;
    };

Refer to DT bindings documentation:

.. code-block:: text

    dts/bindings/pinctrl/renesas,rzg3s-pinctrl.yaml
    dts/bindings/pinctrl/pincfg-node.yaml
    dts/bindings/pinctrl/pinctrl-device.yaml

The PINCTRL subsystem is enabled by default in ``rz_g3s_defconfig``,
which automatically enables RZ G3S PINCTRL driver if corresponding DT node is enabled.

.. code-block:: text

    CONFIG_PINCTRL=y
    /* automatically enabled */
    CONFIG_PINCTRL_RZG3S=y

The RZ G3S PINCTRL driver code can be found at:

.. code-block:: text

    drivers/pinctrl/pinctrl_rzg3s.c
    drivers/pinctrl/Kconfig.rzg3s
    soc/arm/renesas_rz/rz_g/pinctrl_soc.h

The RZ G3S PINCTRL driver DT helper macro are defined in:

.. code-block:: text

    dt-bindings/pinctrl/renesas/pinctrl-r9a08g045.h

GPIO pins configuration
```````````````````````

The RZ G3S PINCTRL driver allows to configure GPIO pins by using **pinmux** DT nodes (the nodes with **pinmux** property).
The GPIO pins are grouped in Ports with up to 8 pins per ports, each of them configurable
as GPIO or Peripheral Function pin.
Up to 8 different alternate function modes exist for each single GPIO pin.

When GPIO is used in as GPIO the pin the configurations as actually performed by request from
RZ G3S GPIO driver (:ref:`rzg3s_gpio_label`) which is passed internally to RZ G3S PINCTRL driver.

When GPIO pin is used as Peripheral Function pin the pin configuration shell be provided in Device tree.
For RZ G3S SoC the pin control DT configuration shell be placed in *boards/arm/rz_g3s/rz_g3s_pinctrl.dtsi* file.
For example, SCIF0 pin configuration:

.. code-block:: dts

    &pinctrl {
        scif0_pins: scif0 {
            scif0-pinmux {
                pinmux = <RZG3S_PINMUX(PORT6, 3, 1)>, /* TXD */
                        <RZG3S_PINMUX(PORT6, 4, 1)>; /* RXD */
                /* Additional cfg properties */
                /* bias-pull-down; */
                /* bias-pull-up; */
                /* bias-pull-pin-default; */
                /* drive-strength-microamp; */
                /* input-debounce; */
            };
        };
    };

The additional optional configuration parameters will be applied to every pin specified in **pinmux** property.
The GPIO **pinmux** nodes supports following additional configuration parameters:

* Pull-Up/Pull-Down configuration which will be reflected in **PUPD_m** registers

    * **"bias-pull-down"** - Pull-down is selected;
    * **"bias-pull-up"** - Pull-up is selected;
    * **"bias-pull-pin-default"** - Pull-Up/Pull-Down configuration will be left unchanged;
    * if **none** of the above is specified then Neither pull-up nor pull-down is set (disabled).

* Driving Ability configuration which will be reflected in **IOLH_m** registers

    * **"drive-strength-microamp"** - the drive Ability of pin.
      This property should use values defined by PINCTRL_RZG3S_PIN_IOLH_xx helper macro in pinctrl-r9a08g045.h
      depending on pin group and selected power supply.

* Digital Noise Filter configuration which will be reflected in **FILONOFF_m, FILNUM_m and FILCLKSEL_m** registers

    * **"input-debounce"** - the Digital Noise Filter configuration of the pins.
      This property should be defined using INCTRL_RZG3S_FILTER_SET() helper macro in pinctrl-r9a08g045.h to specify
      values for FILNUM_m and FILCLKSEL_m registers and enable Digital Noise Filter.

Special Purpose pins configuration
``````````````````````````````````

The Special Purpose pins has statically assigned function, but supports additional parameters configuration.
The RZ G3S PINCTRL driver allows to configure Special Purpose pins by using **pins** DT nodes (the nodes with **pins** property).
The Special Purpose pins are identified by name.

Refer to dts/bindings/pinctrl/renesas,rzg3s-pinctrl.yaml and
"General Purpose Input Output Port (GPIO)" section in "Renesas RZ/G3S Group User’s Manual: Hardware" for list of supported pins.
Example of Special Purpose pins configuration:

.. code-block:: dts

    &pinctrl {
        example_pins: example0 {
            a-pins {
                pins = "AUDIO_CLK1", "TMS_SWDIO";
                input-enable;
                /* Additional cfg properties */
            };
            b-pins {
                pins = "TMS_SWDIO";
                drive-strength-microamp = <PINCTRL_RZG3S_PIN_IOLH_A_3_3V_9000>;
                /* Additional cfg properties */
            };
        };
    };

The additional optional configuration parameters will be applied to every pin specified in **pins** property.
The Special Purpose **pins** nodes supports following additional configuration parameters:

* Driving Ability configuration which will be reflected in **IOLH_m** registers

    * **"drive-strength-microamp"** - the Drive Ability of pin.
      This property should use values defined by PINCTRL_RZG3S_PIN_IOLH_xx helper macro in pinctrl-r9a08g045.h
      depending on pin group and selected power supply.

* Input Enable configuration which will be reflected in **IEN_m** registers

    * **"input-enable"** - Input enabled;
    * **"input-disable"** - Input disabled;
    * if **none** of the above is specified then Input Enable configuration will be left unchanged.

* Output enable configuration which will be reflected in **ETH_MODE** registers

    * **"output-enable"** - The Direction of the IO buffer is Output;
    * **"input-enable"** - The Direction of the IO buffer is Input;
    * if **none** of the above is specified then Output enable will be left unchanged.

* Digital Noise Filter configuration which will be reflected in **FILONOFF_m, FILNUM_m and FILCLKSEL_m** registers.
  It's supported only for the **"NMI"** Special pin

    * **"input-debounce"** - the Digital Noise Filter configuration of the pins.
      This property should be defined using INCTRL_RZG3S_FILTER_SET() helper macro in pinctrl-r9a08g045.h to specify
      values for FILNUM_m and FILCLKSEL_m registers and enable Digital Noise Filter.

Group of pins configuration
```````````````````````````

The Group of pins supports additional parameters configuration, which will affect all pins in group.
The RZ G3S supports below Group of pins:

* ETH0 settings reflects in Ether Ch0 Voltage Mode Control Register (ETH0_POC);
* ETH1 settings reflects in Ether Ch0 Voltage Mode Control Register (ETH0_POC);
* SD_CH0 settings reflects in SD Ch0 IO Voltage Mode Control Register (SD_CH0_POC);
* SD_CH1 settings reflects in SD Ch1 IO Voltage Mode Control Register (SD_CH1_POC);
* XSPI settings reflects in XSPI IO Voltage Mode Control Register (XSPI_POC) and
  XSPI/OCTA Output Enable Control Register (XSPI/OCTA Hi-Z);
* I3C settings reflects I3C Control Register (I3C_SET)

The RZ G3S PINCTRL driver allows to configure the Group of pins by using **groups** DT nodes (the nodes with **groups** property).
The Group of pins are identified by name.

Refer to dts/bindings/pinctrl/renesas,rzg3s-pinctrl.yaml and
"General Purpose Input Output Port (GPIO)" section in "Renesas RZ/G3S Group User’s Manual: Hardware" for list of supported groups.
Example of Group of pins configuration:

.. code-block:: dts

    &pinctrl {
        example_groups: example0 {
            a-group {
                pins = "XSPI";
                power-source = <1800>;
            };
            b-group {
                pins = "I3C";
                power-source = <1200>;
                low-power-enable;
            };
        };
    };

The Group of pins **groups** nodes supports following configuration parameters depending on group:

* ETH0, ETH1, SD_CH0, SD_CH1, XSPI, I3C

    * **"power-source"** - sets the IO voltage mode for the corresponding group of pins in x_POC registers.

* XSPI

    * **"bias-high-impedance"** - Control to Hi-Z (output disable) in XSPI/OCTA Hi-Z register;
    * **"bias-disable"** - Unlock Hi-Z in XSPI/OCTA Hi-Z register.

* I3C

    * **"low-power-enable"** - Standby mode in I3C_SET register;
    * **"low-power-disable"** - Normal mode in I3C_SET register.


* Output enable configuration which will be reflected in **ETH_MODE** registers

    * **"output-enable"** - The Direction of the IO buffer is Output;
    * **"input-enable"** - The Direction of the IO buffer is Input;
    * if **none** of the above is specified then Output enable will be left unchanged.

PINCTRL consumer interface
``````````````````````````

The consumer device can use standard **pinctrl-X** (and **pinctrl-names**) DT properties to specify
required Pins configuration in DT, and then use Zephyr :ref:`pinctrl_api` to apply it.

.. code-block:: dts

    &scif1 {
        current-speed = <115200>;
        pinctrl-0 = <&scif1_pins>;
        pinctrl-names = "default";
        status = "okay";
    };

    &canfd {
        pinctrl-0 = <&can0_pins &can1_pins>;
        pinctrl-names = "default";
    };

Refer to Zephyr :ref:`pinctrl-guide` for more information.

.. raw:: latex

    \newpage
