# GD32F4xx firmware library with gcc and makefile support
This repo enable arm-none-eabi-gcc and Makefile support to GigaDevice GD32F4xx Firmware Library (v3.0.3 without any changes). 

And the default linker script and startup asm file is for GD32F470ZGT6 to support development with JLC LiangShanPi.

GD32F4xx standard firmware library is suitable for GD32F4xx series MCU. The library is compatible with the CMSIS (Cortex-M Microcontroller Software Interface Standard), and includes programs, data structures and macro definitions. It covers the features of all the integrated peripherals, contains all the related drivers and sample programs.

# to support other GD32F4xx parts

You can modify:

- Makefile (note the '-DGD32F470)
- Firmware/gd32f4xx.ld to define memory layout to match your MCU
- Choose the startup asm file in 'Firmware' dir

to support other GD32F4xx parts.

# to build

The default 'User' codes is to blink four LEDs on JLC LiangShanPi board.

to build it, type:
```
make
```

The gd32f470zgt6.elf/bin/hex will be generated in 'build' dir.

# to program

```
openocd -f /usr/share/openocd/scripts/interface/cmsis-dap.cfg -f /usr/share/openocd/scripts/target/stm32f4x.cfg -c "program build/gd32f470zgt6.elf verify reset exit"
```

or simple way:
```
make program
```

