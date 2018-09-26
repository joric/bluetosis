# Bluetosis

Bluetooth firmware for the Mitosis keyboard (right half runs Bluetooth client and Gazell host simultaneously)

## Precompiled Firmware

Bluetosis firmware does NOT come with the softdevice because it would violate Nordic redistribution terms, download
softdevice s130 2.0.1 from the [Nordic site](https://www.nordicsemi.com/eng/nordic/Products/nRF51822/S130-SD-v2/53724)
and flash it the same way as firmware (you need to do it just once).

* [latest .hex file (Bluetosis releases section)](https://github.com/joric/bluetosis/releases)
* [nRF5x softdevice s130 2.0.1 (Nordic site)](https://www.nordicsemi.com/eng/nordic/Products/nRF51822/S130-SD-v2/53724)

All fimware updates and softdevice go for the right half of the Mitosis keyboard, left half should be held intact.

## Default Layout (Mitosis-BT)

* Press <kbd>Adjust</kbd> + <kbd>←</kbd> <kbd>↓</kbd> <kbd>↑</kbd> <kbd>→</kbd> to switch between three Bluetooth devices and a receiver
* Press <kbd>Fn</kbd> + <kbd>Adjust</kbd> + <kbd>←</kbd> <kbd>↓</kbd> <kbd>↑</kbd> <kbd>→</kbd> to reset three corresponding Bluetooth devices or erase all bonds

[![](https://kle-render.herokuapp.com/api/3f5dd1c848bb9a7a723161ad5e0c8e39?6)](http://www.keyboard-layout-editor.com/#/gists/3f5dd1c848bb9a7a723161ad5e0c8e39)

## Uploading

### ST-Link V2

To flash nRF modules, connect ST-LINK/V2 to the module programming pins (SWCLK, SWDIO, GND, 3.3V - top to bottom) and run this batch (windows 10):

```
@echo off
set path=C:\SDK\openocd-0.10.0-dev-00247-g73b676c\bin-x64;%path%
set file=%~dp0custom\iar\_build\nrf51822_xxac.hex
openocd -f interface/stlink-v2.cfg -f target/nrf51.cfg ^
-c init -c "reset halt" -c "flash write_image erase %file:\=/%" -c reset -c exit

```

### BluePill

This is basically an [$1.80][Bluepill] STM32 board (STM32F103C8T6)
that you can use as an ST-Link V2 replacement.
You need to flash the programmer firmware ([Blackmagic]) first.
It creates two virtual ports (GDB and UART for debugging) no OpenOCD needed.

* set jumpers to 0-1 0-0, download [Demonstrator GUI](https://www.st.com/en/development-tools/flasher-stm32.html) from ST-LINK, hook up UART adapter ([RX - A9, TX - A10](https://i.imgur.com/sLyYM27.jpg))
* open `blackmagic.bin`, built with `make clean && make PROBE_HOST=stlink`, select 128K device, offset 0x08002000, hit Flash
* set jumpers to 0-0 0-0, update USB drivers with [zadig](https://zadig.akeo.ie/), hook up production board via SWD ([SWCLK - A5, SWDIO - B14](https://i.imgur.com/Ikt8yZz.jpg))
* upload production firmware:
`arm-none-eabi-gdb --quiet --batch -ex "target extended-remote \\.\COM5" -ex "mon swdp_scan" -ex "att 1" –ex "load nrf51822_xxac.hex" –ex kill`
(considering COM5 is the first virtual port)

Flashing Bluepill board | Uploading firmware | Debugging firmware
-|-|-
![](https://i.imgur.com/sLyYM27.jpg)|![](https://i.imgur.com/Ikt8yZz.jpg)|![](https://i.imgur.com/KtkEpR9.jpg)


## Building

Copy this repository to the `nRF5_SDK_12/bluetosis` folder.

Mind that symlink or junction won't work on Windows 10 for some reason (GCC Makefile error `... is a directory. stop`).

### IAR

Open .eww, hit Make, that's it.
I'm using a single plate (reversed) version for
the debug configuration (modules soldered to the top of the PCB),
to build standard version remove `COMPILE_REVERSED` from defines.

### GCC

Open `nRF5_SDK_12/components/toolchain/gcc/Makefile.posix` make sure that `GNU_INSTALL_ROOT := /usr/`, then:

```
sudo apt install openocd gcc-arm-none-eabi
cd nRF5_SDK_12
git clone https://github.com/joric/bluetosis && cd bluetosis
cd firmware/custom/armgcc && make
```

Working GCC linker settings for softdevice s130 2.0.1 and [YJ-14015] modules (256K ROM, 16K RAM) appear to be:
```
  FLASH (rx) : ORIGIN = 0x1b000, LENGTH = 0x25000
  RAM (rwx) :  ORIGIN = 0x20002000, LENGTH = 0x2000
```

To build with this settings, set stack and heap to 1024 or something in `gcc_startup_nrf51.S` (originally 2048).
You could also use Makefile defines:

```
ASMFLAGS += -D__HEAP_SIZE=1024 -D__STACK_SIZE=1024
```

### Firmware merging

You can also merge Bluetooth softdevice with Bluetooth firmware using mergehex utility from
[nRF5x Command Line Tools](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_installation.html):
```
mergehex.exe -m s130_nrf51_2.0.1_softdevice.hex nrf51822_xxac.hex -o out.hex
```

But it's much faster and more convenient to flash softdevice just once and do frequent firmware updates.

## Debugging

There is a built in `NRF_LOG` in Nordic SDK but it doesn't work with GCC (probably no memory).
I had to write a small drop-in replacement. In IAR you can try using `NRF_LOG_ENABLED`
and `NRF_LOG_USES_UART` in `sdk-config.h`.
Note that built in debugging has its own pin settings in `sdk-config.h`
(not the ones that are in the `custom_board.h`).

Neither nRF51822 nor ST-Link V2 have SWO pin for printf
([there is no tracing hardware in the nRF51 series](https://devzone.nordicsemi.com/f/nordic-q-a/1875/nrf51822---debug-output-via-j-link-swo)),
so I had to use UART. You only need ONE pin to print messages via UART (e.g. using Arduino IDE Serial Monitor).
There are no free broken out pins on the Mitosis so I've used pin 19 (bottom right key) as TX_PIN_NUMBER
and it worked.

Hook up a single UART RX pin at 115200 baud ([currently pin 21, key S15 or S23](https://i.imgur.com/apx8W8W.png)).
You will also need common GND and VCC to make it work. It doesn't really interfere much with the keyboard matrix so you can use any pin you want,
just don't use the same pin for TX and RX to avoid feedback.

You can also use [Blackmagic] probe for debugging. It is actually the best because it has a built in UART ([pin A3][pinout-bmp])
on the second virtual COM port so you won't need another USB.
I personally use [Bluepill] board with Blackmagic firmware as a programmer and a debugger and Putty with enabled local echo
as a serial monitor.

## Status

### Works

* Bluetooth and Gazell timesharing
* Battery level reporting via Bluetooth
* Debugging via UART
* Basic QMK layout support
* Bluetooth pairing shortcut
* Switching between RF and Bluetooth modes
* Switching between Bluetooth devices

### TODO

* Full QMK/TMK suport (maybe)

#### QMK support is still in progress (firmware compiles fine without QMK)

QMK firwmare has massive incompatibility issues with ICCARM (IAR) that can't be fixed with preprocessor.
So it's either a fully-GCC setup (armgcc and maybe uVision Keil) or patching QMK
(changes are small but I don't know if they ever get merged to the upstream).
I've patched and compiled QMK for ICCARM but couldn't get correct keycodes so far.

#### QMK incompatibility issues

* Excessive and unnecessary binary literals (e.g. `0b011` is C++14 only), should use hex or decimals
* GCC-specific switch case ranges (`case A ... Z`, ), should use `if` and `else`
* GCC-specific `__attribute__` keyword, e.g. `__attribute__ ((weak))`, should use `__WEAK` define
* Inplace initializations (`#define MACRO(...) ({static const macro_t __m[]; PROGMEM={__VA_ARGS__}; &__m[0]})`)

#### Patching QMK for IAR

* add `#ifdef __ICCARM__` for IAR-specific code
* add `__attribute__(x)=`  to the preprocessor directives, use `__weak` or whatever instead
* add dummy Atmel-specific variables and functions `PORTF`, `PORTD`, etc.

#### QMK embedding guide

* add `QMK_KEYBOARD_H="your_hardware_name.h"` to the preprocessor directives
* add `#include "keyboard.h"`, add QMK paths (quantum/, tmk_core/common/, etc.)
* implement `timer_read32()` if it's incompatible (e.g. use `app_timer_cnt_get` for nrf5x)
* implement `matrix_row_t matrix_get_row(uint8_t row)` callback (just read from array)
* implement host driver callbacks, add `host_set_driver(&driver)` to the init sequence
* add `keyboard_task()` to the main loop

See this GCC-only TMK core-based project for example (all API calls are precisely the same):

* https://github.com/Lotlab/nrf51822-keyboard

## Software

* [nRF5 SDK] - nRF51/52 toolchain (this version uses [SDK 12.3.0](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v12.x.x/nRF5_SDK_12.3.0_d7731ad.zip))
* [IAR] - IDE that includes a C/C++ compiler (IAR 8.30 for ARM)
* [OpenOCD] - embedded debugger for Windows 10 ([openocd-0.10.0-dev-00247-g73b676c.7z])
* [WinAVR] - firmware tools for AVR MCU ([WinAVR-20100110-install.exe])
* [Zadig] - you will need to install libusb in order to run OpenOCD ([zadig-2.3.exe])

## Hardware

* [ST-LINK/V2][stlink]: $2.54, had in stock (you can also use [$1.80](https://www.aliexpress.com/item//32583160323.html) STM32 board [instead](https://gojimmypi.blogspot.com/2017/07/BluePill-STM32F103-to-BlackMagic-Probe.html)).
* [YJ-14015][yj-ali]: nrf51822 modules, 3 pcs: $10.5 ($3.50 * 3), free shipping, need 2/3, so $7.
* [10 main PCB's][mitosis.zip] from [EasyEDA], $13.32 ($2 + $11.32 for trackable shipping), used 4/10, so $5.32.
* [3 receiver PCB's][receiver.zip] from [OshPark], $5.40, free shipping, used only 1/3, so $1.80.
* [Arduino Pro Micro](https://www.aliexpress.com/item/-/32648920631.html) from Aliexpress (price varies from $2.54 to $4), had in stock.
* [Si2302] mosfets: board survives reverse polarity for a while, you may just [short the pads](https://i.imgur.com/h1Mx8Yw.jpg).
* [ASMB-MTB1-0A3A2] from Aliexpress (or Cree CLVBA-FKA, or 3 single LEDs, very optional)
* [AMS1117]: 5v to 3v regulator, had in stock. You probably can use diodes for 2v drop.
* [1206 4.7k] resistor arrays, 2 pcs: had in stock (taken from an old motherboard).
* Switches and caps: most of you have more than you can handle.

### Total

* Keyboard: $12.32 ($7 + $5.32) and I got enough PCBs to build 3 and they can reuse receiver.
* Receiver: $7.84 ($3.50 + $1.80 + $2.54) firmware upgrade to ble and you wouldn't need it at all.

So, about $20 for a single keyboard.

### PCB Manufacturers

* https://oshpark.com - 3 purple receiver PCBs, $5.40, untracked, 21 days
* http://easyeda.com - 10 green PCBs for $2 + $11.32 shipping = $13.32, trackable, 10 days
* https://www.elecrow.com - 6 black PCBs for $4.90 + $6.42 shipping = $11.32, trackable, 36 days
* http://dirtypcbs.com - 10 black for $16.95 + $9.00 shipping = $25.95, untrackable, lost/refunded
* https://www.seeedstudio.com - 10 black PCBs for $4.90 + $16.50 shipping = $21.40 - untested
* https://jlcpcb.com - 10 black PCBs for $2 + $10.98 shipping = $12.98 - untested

## Schematics

There were speculations that Core 51822 has 32 GPIO pins available, so it's possible to make an Atreus62 without
using a keyboard matrix. It is not true. GPIO 26/27 are shared by the 32kHz crystal
and [there are only 31 GPIOs hence PIN31 is not existing](https://devzone.nordicsemi.com/f/nordic-q-a/5674/pin-31-interrupt). So you could only have 29 keys
on each side (58 total) if you manage to layout them without crossing.

Pins that can be used for side autodetection: 16 (L_S16), 18 (L_S22) (left-only pins); 22 (R_S22), 25 (R_S16) (right-only pins).
There are 5 unused pins on each side (6 with LEDs): 11, 12, 20, 22, 25 (left, LED pin 23); 11, 12, 16, 18, 20 (right, LED pin 17).


* [nRF51822 Core-B Schematics](https://www.waveshare.com/w/upload/5/57/Core51822-Schematic.pdf)
* [nRF51822 Core-B Pinout](https://www.waveshare.com/img/devkit/accBoard/Core51822-B/Core51822-B-pin.jpg)

### Mitosis PCB

* Original Mitosis hardware and PCB repository: https://github.com/reversebias/mitosis-hardware

![Mitosis-PCB](https://i.imgur.com/TDxuXfz.jpg)

## Mitosis Clones

* [Interphase](https://github.com/Durburz/interphase) by [/u/Durburz_](https://www.reddit.com/user/Durburz_) (66 keys, diode matrix, voltage regulator, AAA battery) ([Reddit](https://redd.it/7ggeww))
* [Meiosis](https://redd.it/7uasay)  by [/u/SouthPawEngineer](https://www.reddit.com/u/SouthPawEngineer) (also [Telophase](https://redd.it/7ruihw), [Helicase](https://redd.it/7zfj19) and [Centromere](https://redd.it/8qkib4), no source files available) ([Site](https://southpawdesign.net))
* [Chimera](https://github.com/GlenPickle/Chimera) by [/u/GlenPickle](https://www.reddit.com/user/GlenPickle) (also Chimera Ergo, Chimera Ortho, Chimera Ergo Mini and Chimera Ergo 42)
* [Kissboard](https://github.com/fhtagnn/kissboard) by [/u/fhtagnn](https://www.reddit.com/user/fhtagnn) ([AdNW](http://adnw.de)-inspired layout, PCB's are not tested) ([Reddit](https://redd.it/8bauoz)) ([Album](https://imgur.com/a/A95FF))
* [Dichotomy](https://redd.it/7g54l8) by [/u/Snipeye](https://www.reddit.com/user/Snipeye) (48-key Mitosis clone with digital encoders, no sources available) ([Youtube](https://youtu.be/5jmmYbgtOgI)) ([Kickstarter](https://www.kickstarter.com/projects/1090732691/dichotomy-keyboard-and-mouse)) 
* [Trident](https://github.com/YCF/Trident) by [/u/imFengz](https://www.reddit.com/u/imFengz) (Wireless Let's Split, module and battery placed between the switches) ([Reddit](https://redd.it/6um7eg)) ([Image](https://i.imgur.com/mCTgwu5.png))
* [Orthrus](https://github.com/bezmi/orthrus) by [/u/bezmi](https://www.reddit.com/u/bezmi) (great 52-key Atreus/Mitosis crossover, KiCad project) ([Reddit](https://redd.it/8txry7)) 
* [Comet](https://github.com/satt99/comet46-hardware) by [/u/SaT99](https://www.reddit.com/user/SaT999) (Comet46 - split 40% wireless keyboard) ([Gallery](https://imgur.com/a/vs1W5qB)) ([Firmware](https://github.com/satt99/comet46-firmware)) ([Reddit](https://redd.it/8ykwjj))

## Other wireless solutions

### Bluetooth UART service

There are a few split keyboards that successfully run Bluetooth HID and Bluetooth UART service concurrently.
Most of them use Adafruit nRF52 library and simple sketches written and compiled in Arduino IDE.

#### Arduino nRF5

There is [Arduino-nRF5 by Sandeep Mistry] that supports nRF51.
You could use [arduino-BLEPeripheral] library for sketches.
Works fine with BLE400 board ([Arduino IDE setup](https://i.imgur.com/8dfPZFm.jpg), [wiring](https://i.imgur.com/A9QIN2j.jpg)).
Sadly this library has [multiple issues](https://github.com/sandeepmistry/arduino-BLEPeripheral/issues/160) with Windows 10.

[Arduino-nRF5 by Sandeep Mistry]: https://github.com/sandeepmistry/arduino-nRF5
[arduino-BLEPeripheral]: https://github.com/sandeepmistry/arduino-BLEPeripheral

#### Arduino nRF52

Note that Arduino nRF52 builds (usually based on [Bluefruit nRF52](https://www.adafruit.com/product/3406) boards)
are NOT compatible with the Mitosis keyboard (softdevice s132 and all the software is nRF52-only, Mitosis is nRF51-based).

* [Curves - my bluetooth split](https://redd.it/86asf6) by [/u/JKPro777](http://reddit.com/u/JKPro777)
* [Split Bluetooth Keyboard](https://redd.it/7fdrdz) by [/u/wezfurlong](https://www.reddit.com/u/wezfurlong) ([gist](https://gist.github.com/wez/b30683a4dfa329b86b9e0a2811a8c593))

#### BlueMicro

This is a drop-in Pro Micro replacement that is compatible with Arduino nRF52 boards (it's NOT compatible with nRF51).
BlueMicro is open source, official repositories are [BlueMicro_BLE] (firmware) and [NRF52-Board] (hardware).

* [Iris gets the BLE treatment](https://redd.it/8rtvi7) by [/u/jpconstantineau](http://reddit.com/u/jpconstantineau)
* [Ergotravel 2](https://redd.it/8i2twe) by [/u/jpconstantineau](http://reddit.com/u/jpconstantineau)

[BlueMicro_BLE]: https://github.com/jpconstantineau/BlueMicro_BLE 
[NRF52-Board]: https://github.com/jpconstantineau/NRF52-Board

## References

* [My fork of the Mitosis repository (bonus documentation included)](https://github.com/joric/mitosis/tree/devel)
* [Mitosis bluetooth firmware for SDK 11 (now deprecated)](https://github.com/joric/mitosis/tree/devel/mitosis-bluetooth)
* [Reddit thread](https://redd.it/91s4pu)

[Blackmagic]: https://github.com/blacksphere/blackmagic
[Bluepill]: https://www.aliexpress.com/item//32583160323.html
[mitosis-bt.hex]: https://raw.githubusercontent.com/joric/mitosis/devel/precompiled/mitosis-bt.hex
[ST-LINK/V2]: http://www.ebay.com/itm/331803020521
[OpenOCD]: http://www.freddiechopin.info/en/download/category/10-openocd-dev
[YJ-14015]: https://www.ebay.com/itm/282575577879
[Blackmagic]: https://gojimmypi.blogspot.com/2017/07/BluePill-STM32F103-to-BlackMagic-Probe.html
[nRF5 SDK 11]: https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v11.x.x/nRF5_SDK_11.0.0_89a8197.zip
[pinout]: https://i.imgur.com/apx8W8W.png
[RAM]: https://devzone.nordicsemi.com/b/blog/posts/rom-and-ram-management
[EasyEDA]: https://easyeda.com
[OshPark]: https://oshpark.com
[ASMB-MTB1-0A3A2]: https://www.aliexpress.com/item/-/32809898075.html
[Si2302]: https://www.aliexpress.com/item/-/32883659198.html
[mitosis.zip]: https://github.com/reversebias/mitosis-hardware/blob/master/gerbers/mitosis.zip
[receiver.zip]: https://github.com/reversebias/mitosis-hardware/blob/master/gerbers/receiver.zip
[stlink]: http://www.ebay.com/itm/331803020521
[yj-ali]: https://www.aliexpress.com/item/-/32832872640.html
[yj-ebay]: https://www.ebay.com/itm/282575577879
[IAR]: https://www.iar.com
[NRF5 SDK]: https://developer.nordicsemi.com/nRF5_SDK
[OpenOCD]: http://www.freddiechopin.info/en/download/category/10-openocd-dev
[WinAVR]: https://sourceforge.net/projects/winavr
[Zadig]: https://zadig.akeo.ie
[nRF5_SDK_11.0.0_89a8197.zip]: https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v11.x.x/nRF5_SDK_11.0.0_89a8197.zip
[openocd-0.10.0-dev-00247-g73b676c.7z]: http://www.freddiechopin.info/en/download/category/10-openocd-dev?download=140%3Aopenocd-0.10.0-dev-00247-g73b676c
[WinAVR-20100110-install.exe]: https://sourceforge.net/projects/winavr/files/WinAVR/20100110/WinAVR-20100110-install.exe/download
[zadig-2.3.exe]: https://zadig.akeo.ie/downloads/zadig-2.3.exe
[AMS1117]: https://www.aliexpress.com/item/-/32826077143.html
[1206 4.7k]: https://www.aliexpress.com/item/-/32853745131.html
[pinout-bmp]: https://i.imgur.com/KtkEpR9.jpg
