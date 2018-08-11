@echo off

set build=Debug

if "%1"=="" (
set file=%~dp0custom\iar\_build\nrf51822_xxac.hex
) else (
set file=%1
)

set option=1

if "%option%"=="0" goto stlinkv2
if "%option%"=="1" goto blackmagic

:stlinkv2

rem ST-Link V2 and OpenOCD

set openocd=C:\SDK\openocd-0.10.0-dev-00247-g73b676c\bin-x64
set path=%openocd%;%path%

openocd -f interface/stlink-v2.cfg -f target/nrf51.cfg ^
-c init -c "reset halt" -c "flash write_image erase %file:\=/%" -c reset -c exit

goto end

:blackmagic

set port=COM5

set eabi=C:\Users\User\AppData\Local\Arduino15\packages\adafruit\tools\gcc-arm-none-eabi\5_2-2015q4\bin
set path=%eabi%;%path%

set nordic=C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin
set path=%nordic%;%path%

echo Uploading...

arm-none-eabi-gdb.exe --quiet --batch -ex "target extended-remote \\.\%port%" -ex "mon swdp_scan" ^
-ex "file %file:\=/%" -ex "att 1" -ex load -ex kill

:end

