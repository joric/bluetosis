@echo off

set file=%~dp0custom\armgcc\_build\nrf51822_xxac.hex
set sdev=%~dp0..\..\components\softdevice\s130\hex\s130_nrf51_2.0.1_softdevice.hex

set nordic=C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin
set path=%nordic%;%path%
set tmp=tmp.hex

if "%1"=="reset" (
mergehex -m %sdev% %file% -o %tmp%
set file=%tmp%
set erase_opt=-ex "mon erase"
) else if not "%1"=="" (
set file=%1
)

set option=1

if "%option%"=="0" goto stlinkv2
if "%option%"=="1" goto blackmagic

:stlinkv2

set openocd=C:\SDK\openocd-0.10.0-dev-00247-g73b676c\bin-x64
set path=%openocd%;%path%

openocd -f interface/stlink-v2.cfg -f target/nrf51.cfg ^
-c init -c "reset halt" -c "flash write_image erase %file:\=/%" -c reset -c exit

goto end

:blackmagic

set port=COM8
mode %port% | find "RTS" > nul
if errorlevel 1 echo Port %port% not found && exit

set eabi=C:\Users\User\AppData\Local\Arduino15\packages\adafruit\tools\gcc-arm-none-eabi\5_2-2015q4\bin
set path=%eabi%;%path%

arm-none-eabi-gdb.exe --quiet --batch -ex "target extended-remote \\.\%port%" -ex "mon swdp_scan" ^
-ex "file %file:\=/%" -ex "att 1" %erase_opt% -ex load -ex kill

:end

