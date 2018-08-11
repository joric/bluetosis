@echo off

set iar=C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.1\common\bin
set nordic=C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin
set path=%iar%;%nordic%;%path%

set s130=..\..\components\softdevice\s130\hex\s130_nrf51_2.0.0_softdevice.hex
set dest=..\precompiled_iar\mitosis-bt.hex

set build=nrf51822_xxac
set option=0

if "%option%"=="0" goto iar
if "%option%"=="1" goto gcc

goto end

:iar
set file=custom\iar\_build\nrf51822_xxac.hex
IarBuild.exe custom\iar\mitosis_bluetooth.ewp %build% || exit
goto publish

:gcc
set file=custom\armgcc\_build\nrf51822_xxac.hex
cd custom\armgcc && bash -c make && cd /d %~dp0 || exit

:publish
if not "%build%"=="Release" goto end

echo Merging %s130% and %file% to %dest%...
mergehex.exe --quiet -m %s130% %file% -o %dest%

:end
