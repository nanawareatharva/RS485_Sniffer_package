@echo off
REM ==================================================================
REM  Flash the Room Monitor firmware (1ACCORD / COOL CROP) onto a
REM  Waveshare ESP32-S3-Touch-LCD-7B. No Arduino software needed.
REM
REM  Usage:  FLASH.bat COM21        (or just double-click and type the port)
REM
REM  Saved door counts and settings are kept (this does not erase the flash).
REM ==================================================================
setlocal
cd /d "%~dp0"
set PORT=%1
if "%PORT%"=="" set /p PORT=Enter the board's COM port (see Device Manager, e.g. COM21):
if "%PORT%"=="" goto :end

echo.
echo Flashing to %PORT% ...
echo If it stays at "Connecting...": hold BOOT, tap RESET, release BOOT.
echo.
esptool.exe --chip esp32s3 --port %PORT% --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode keep --flash_freq keep --flash_size keep 0x0 RS485_Sniffer.ino.bootloader.bin 0x8000 RS485_Sniffer.ino.partitions.bin 0xe000 boot_app0.bin 0x10000 RS485_Sniffer.ino.bin
if errorlevel 1 (
  echo.
  echo [FAILED] Check: USB cable in the port marked UART, UART switch on UART1, correct COM port,
  echo          Arduino IDE Serial Monitor closed. Then try again.
) else (
  echo.
  echo [DONE] The board restarts by itself and shows the start-up screen.
)

:end
echo.
pause
endlocal
