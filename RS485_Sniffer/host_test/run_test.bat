@echo off
REM Builds and runs the decoder host test with MSVC (Visual Studio 2019 Build Tools).
REM Build output goes to %TEMP%\rs485_host_test so the sketch folder stays clean.
setlocal
set OUT=%TEMP%\rs485_host_test
if not exist "%OUT%" mkdir "%OUT%"
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cl /nologo /EHsc /W4 /std:c++17 /I"%~dp0.." "%~dp0test_decoder.cpp" "%~dp0..\modbus_decoder.cpp" /Fo"%OUT%\\" /Fe"%OUT%\test_decoder.exe" || exit /b 1
"%OUT%\test_decoder.exe"
