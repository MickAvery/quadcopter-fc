# Quadcopter Flight Controller
> Name to be determined

## Introduction
As a student, what's the best way to get choosy firmware companies to notice you when you're not a nerd with straight-A's? Why, make a custom quadcopter flight controller, of course!

## Components
### Hardware
The main flight controller is an **STM32F407 with an ARM Cortex M4 chip**.
The other components of the flight controller are subject to change, so I won't put them here for now.
### Software
The embedded control software is developed using ChibiOS open-source RTOS.

## Build
I asume you have to following packages:
1. OpenOCD on-chip debugger
2. ARM cross-compiler and debug tools (I'll make this more specific later on)

Asuming you have the packages above, just run `make` to compile. You'll notice a `build/` folder is made that houses the binary files.
Next, have **two** terminal windows open, and in each one:
1. Run `openocd -f stm32f4.cfg` on one window
2. Run `arm-none-eabi-gdb build/ch.elf`. Here you'll get a prompt much like running regular GDB. Then run the following commands:
    * `tar ext :3333`
    * `mon reset halt`
    * `load`
    * `run`

Instructions incomplete for now, I'll add to this later on. For now, at least you know how to compile the code and load it into the board.
