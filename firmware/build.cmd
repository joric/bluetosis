@echo off

if exist ".reversed" set USER_DEFINES=-DCOMPILE_REVERSED=1

set option=1

if "%option%"=="0" goto iar
if "%option%"=="1" goto gcc

goto end

:iar
set iar=C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.1\common\bin
set path=%iar%;%path%
set file=custom\iar\_build\nrf51822_xxac.hex
IarBuild.exe custom\iar\mitosis_bluetooth.ewp nrf51822_xxac || exit
goto publish

:gcc
set file=custom\armgcc\_build\nrf51822_xxac.hex
cd custom\armgcc && bash -c "make USER_DEFINES=%USER_DEFINES%" && cd /d %~dp0 || exit
goto publish

:publish

call program.cmd

:end
