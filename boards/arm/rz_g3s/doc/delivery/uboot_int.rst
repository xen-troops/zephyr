Integration with u-boot
-----------------------

The u-boot and Zephyr application are executed on different System Cores in parallel, according to the current Renesas RZ G3S MPU Multi-OS SW design:

* u-boot on Cortex-A55
* Zephyr on Cortex-M33

As result, u-boot and Zephyr application should not share SoC HW resources otherwise it will cause HW corruption and unpredictable behavior.
For example, if SCIF1 (SER0_UART) is used by Zephyr application then:

* SCIF1 has to be disabled in u-boot device tree
* any SoC IP which share pins with SCIF1 (SER0_UART) should be disabled also.

For example, default Renesas RZ G3S u-boot has sdhi2 (SD2) device enabled which shares SD2_CD (P14_1) pin with SCIF1_RXD pin on RZ/G3S-EVKIT,
as result it breaks SCIF1 RX functionality due to pinmux corruption. The below patch shows example of how sdhi2 (SD2) can be disabled in u-boot:

.. code-block:: diff

     arch/arm/dts/smarc-rzg3s.dts | 2 +-
     1 file changed, 1 insertion(+), 1 deletion(-)

    diff --git a/arch/arm/dts/smarc-rzg3s.dts b/arch/arm/dts/smarc-rzg3s.dts
    index 01f7f0c890..118e311277 100644
    --- a/arch/arm/dts/smarc-rzg3s.dts
    +++ b/arch/arm/dts/smarc-rzg3s.dts
    @@ -84,7 +84,7 @@
        pinctrl-0 = <&sdhi2_pins>;

        bus-width = <4>;
    -   status = "okay";
    +   status = "disabled";
     };

     &eth0 {
    --
    2.25.1
