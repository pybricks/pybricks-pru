# Pybricks EV3 PRU firmware source code

This is the source code for the firmware which runs on the Programmable Real-Time Unit Subsystem (PRUSS) of the EV3.

## Quick Start

1. Obtain the GCC `pru-elf` toolchain. Prebuilt Linux binaries are available [here](https://github.com/dinuxbg/gnupru/releases). Ensure that the tools are in your `PATH` (for example, by running `pru-gcc -v`)
2. Run `make` in this directory
3. The binary file which will be embedded into Pybricks is `pru_ledpwm.bin`
3. If you want to examine the resulting assembly code, run `pru-objdump -dS pru_ledpwm.elf`

## What is the purpose of this code?

The PRU is a small custom microcontroller core embedded inside the larger AM1808 System-on-Chip which powers the Mindstorms EV3. Although it has access to most of the system, the intended use-case for the PRU is to offload tasks which require "real-time" performance. Moving tasks from the main ARM CPU to the PRU frees up the main CPU from having to handle these events.

The PRUs have their own instruction and data memory separate from the ARM CPU, and they are relatively small, so a common developer workflow is to embed precompiled PRU firmware blobs into the (much larger) ARM code. The GitHub Actions workflows in this repository help automate this.

The AM1808 contains two PRU cores, called PRU0 and PRU1. PRU0 is used to provide two additional UART serial ports for sensor ports 3 and 4, and this is done using a TI-provided software UART library. PRU1 runs the code in this repository.

Currently, PRU1 is used to implement brightness control for the front LEDs on the EV3 brick.

## Functionality and resources used

This firmware runs on PRU1.

Data is exchanged between the ARM CPU and the PRU firmware using shared memory located in the second 64 KiB of the on-chip RAM (i.e starting at address `0x8001_0000`).

This firmware requires `CONTABPROPTR1.C30` to be set such that the constant table entry points to the shared memory (i.e. the field should have the value `0x0100`).

### LED PWM functionality

This code allows varying the brightness of the front LEDs on the EV3 brick (the red/green LEDs in the directional button pad). This is done by turning them on and off for a specified percentage of the time (pulse-width modulation). PWM is being implemented entirely in software and not via a dedicated PWM peripheral.

There are four LEDs, and their brightnesses can be controlled independently. Each LED's brightness can be a value from 0 to 255 inclusive.

The timing for the LED PWM is derived from a hardware timer, Timer0.TIM34. Timer0.TIM34 is expected to count continuously with a period of $256\times256$ (i.e. counting from 0 to 0xffff inclusive).

The LEDs are turned on and off through the PRU's special register R30. This requires the GPIO alternate function to be programmed to select the PRU1 function.

The PRU firmware expects these resources to be set up ahead of time by the ARM firmware.
