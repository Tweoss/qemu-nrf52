@ECHO on
setlocal

:: Script for CLion to run before debugging after build

SET first=C:\msys64\home\vgol\qemu-nrf52\build\qemu-system-arm.exe -M nrf52832DK -device loader,file=
SET program=%1
SET last= -nographic -s -S
SET command=%first%%program%%last%

set PATH=C:\msys64\mingw64\bin;C:\msys64\usr\bin;%PATH%

echo %command%
start %command%

::SLEEP 5
