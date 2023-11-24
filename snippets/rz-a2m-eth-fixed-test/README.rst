.. _snippet-rz-a2m-eth-fixed-test:

RZ/A2M ETHERC fixed-link Test
#########################################

.. code-block:: console

   west build -p always -b rz_a2m -S rz-a2m-eth-fixed-test samples/net/<sample>

Overview
********

This snippet enables RZ/A2M Ethernet functionality on RZA2MEVK with fixed-link feature,
so it can be used with Net samples.

Ether1 is configured to use fixed-link only.

Ether2 is configured to:
	- use fixed-link
	- use LINKSTA

	The RZA2MEVK board has PHY RTL8201FL (U28) LED1 pin to SoC P3_0/ET1_LINKSTA pin which allows
to perform Link status check for Ether2 (ETHERC1) port without accessing Eth PHY.

	Note.
	By default, RTL8201FL PHY's leds configured to state LINK100/ACT100 which makes it useless for
	link detection (check Page 7 Reg 19 in PHY's manual).
	It can be W/A by manually configuring PHY using mdio shell commands to apply only
	LINK100 to the LED1 pin before plug-in cable:

	- Read Page Reg 31
	uart:~$ mdio read 7 0x1f
	7[1f]: 0x0

	- Write Reg 31 to 0x7
	uart:~$ mdio write 7 0x1f 7
	uart:~$ mdio read 7 0x1f
	7[1f]: 0x7

	- Read Page 7 Reg 19
	uart:~$ mdio read 7 0x13
	7[13]: 0x434

	- Write Page 7 Reg 19 to 0x404 (clean bits 4:5)
	uart:~$ mdio write 7 0x13 0x404
	uart:~$ mdio read 7 0x13
	7[13]: 0x404
