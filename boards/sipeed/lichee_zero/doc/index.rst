.. _lichee_zero:

Sipeed Lichee Zero
####################

Overview
********

The Sipeed Lichee Zero is a development board based on the Allwinner V3s SoC.
It features a single-core ARM Cortex-A7 processor running at up to 1.2GHz,
64MB of built-in DDR2 memory, and various peripherals including UART, GPIO,
SPI, I2C, USB, and Ethernet.

Hardware
********

- SoC: Allwinner V3s (sun8i_v3s)
- CPU: ARM Cortex-A7 (single-core, up to 1.2GHz)
- Memory: 64MB DDR2 (built-in)
- Storage: TF card slot
- LEDs: 3 user LEDs (Red, Green, Blue)
- UART: 3 UART controllers
- GPIO: Multiple GPIO pins
- Other: SPI, I2C, USB OTG, Ethernet

Supported Features
****************

The Zephyr port for the Sipeed Lichee Zero supports the following features:

- Serial console (UART0)
- GPIO
- Timer
- Interrupts

Programming and Debugging
************************

The board can be programmed and debugged using:

- J-Link debugger
- OpenOCD

More Information
***************

- `Sipeed Lichee Zero Product Page <https://www.sipeed.com/lichee-zero>`_
- `Allwinner V3s Datasheet <https://linux-sunxi.org/>`_
