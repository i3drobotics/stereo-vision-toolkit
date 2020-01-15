@echo off
echo installing drivers...

CALL :checkMSIReg pylon_USB_Camera_Driver.msi
if %errorlevel% EQU 0 msiexec /i pylon_USB_Camera_Driver.msi
CALL :checkMSIReg pylon_GigE_Performance_Driver.msi
if %errorlevel% EQU 0 msiexec /i pylon_GigE_Performance_Driver.msi
CALL :checkMSIReg pylon_GigE_Filter_Driver.msi
if %errorlevel% EQU 0 msiexec /i pylon_GigE_Filter_Driver.msi
exit /b 0

:checkMSIReg
	for /f "tokens=4" %%a in (
		'reg query HKLM\SOFTWARE\Classes\Installer\Products /s /f %~1'
	) do set "FOUND=%%a"
	exit /B %FOUND%