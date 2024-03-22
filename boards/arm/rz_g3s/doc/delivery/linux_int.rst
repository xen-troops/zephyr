Integration with Linux
-----------------------

The Linux and Zephyr application are executed on different System Cores in parallel, according to the current Renesas RZ G3S MPU Multi-OS SW design:

* Linux on Cortex-A55
* Zephyr on Cortex-M33

As result, Linux and Zephyr application should not share SoC HW resources otherwise it will cause HW corruption and unpredictable behavior.
More over, Linux consider all SoC HW which it knows about as exclusively HW used only by Linux, which first of all include all enabled
in device tree SoC devices. In addition, Linux knows about **ALL** clocks in the system and tries to disable all of them for which no users
are detected (unused clocks).

For example, SCIF1 (SER0_UART) is used by Zephyr application and not defined in Linux device tree.
In this case, Linux will still try to disable SCIF1 clock as it will consider SCIF1 as unused clock.
As result, when both Linux and Zephyr are running the SCIF1 will stop working after Linux is fully booted.

The below patch shows example of how to prevent Linux from manipulating Clocks which are used by HW assigned to Zephyr application,
in this case SCIF1, SCIF3 and OSTM2:

.. code-block:: diff

    diff --git a/drivers/clk/renesas/r9a08g045-cpg.c b/drivers/clk/renesas/r9a08g045-cpg.c
    index f2790305aab5..44bcd7d5e0f2 100644
    --- a/drivers/clk/renesas/r9a08g045-cpg.c
    +++ b/drivers/clk/renesas/r9a08g045-cpg.c
    @@ -464,8 +464,10 @@ static const unsigned int r9a08g045_crit_mod_clks[] __initconst = {
        MOD_CLK_BASE + R9A08G045_IA55_CLK,
        MOD_CLK_BASE + R9A08G045_DMAC_ACLK,
        MOD_CLK_BASE + R9A08G045_VBAT_BCLK,
    -        MOD_CLK_BASE + R9A08G045_SCIF1_CLK_PCK,
    -        MOD_CLK_BASE + R9A08G045_OSTM2_PCLK,
    +   MOD_CLK_BASE + R9A08G045_SCIF1_CLK_PCK,
    +   MOD_CLK_BASE + R9A08G045_OSTM2_PCLK,
    +   MOD_CLK_BASE + R9A08G045_SCIF3_CLK_PCK,
     };

     const struct rzg2l_cpg_info r9a08g045_cpg_info = {
    --

Below is another example of how disable CANFD IP in Linux:

.. code-block:: diff

    diff --git a/arch/arm64/boot/dts/renesas/rz-smarc2-common.dtsi b/arch/arm64/boot/dts/renesas/rz-smarc2-common.dtsi
    index a1948922919d..5b5afeb02017 100644
    --- a/arch/arm64/boot/dts/renesas/rz-smarc2-common.dtsi
    +++ b/arch/arm64/boot/dts/renesas/rz-smarc2-common.dtsi
    @@ -94,12 +94,12 @@ da7212: codec@1a {
     &canfd {
            pinctrl-0 = <&can0_pins &can1_pins>;
            pinctrl-names = "default";
    -       status = "okay";
    +       status = "disabled";
            channel0 {
    -               status = "okay";
    +               status = "disabled";
            };
            channel1 {
    -               status = "okay";
    +               status = "disabled";
            };
     };

    diff --git a/drivers/clk/renesas/r9a08g045-cpg.c b/drivers/clk/renesas/r9a08g045-cpg.c
    index 0067a0fe8c21..58338afc9a63 100644
    --- a/drivers/clk/renesas/r9a08g045-cpg.c
    +++ b/drivers/clk/renesas/r9a08g045-cpg.c
    @@ -464,6 +464,8 @@ static const unsigned int r9a08g045_crit_mod_clks[] __initconst = {
            MOD_CLK_BASE + R9A08G045_IA55_CLK,
            MOD_CLK_BASE + R9A08G045_DMAC_ACLK,
            MOD_CLK_BASE + R9A08G045_VBAT_BCLK,
    +       MOD_CLK_BASE + R9A08G045_CANFD_PCLK,
    +       MOD_CLK_BASE + R9A08G045_CANFD_CLK_RAM,
     };
