@echo off

SETLOCAL EnableDelayedExpansion
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%

:: download file
SET downloadfile=vc_redist.x64.exe

:: url for downloading opencv
SET url=https://aka.ms/vs/16/release/%downloadfile%
echo %url%

:: folder where download is placed
SET downloadfolder=%cd%
:: full output filepath
SET downloadpath=%downloadfolder%\%downloadfile%

:: download vcredist from url
curl -o "%downloadpath%" -L %url%

:: reset working directory
cd "%initcwd%"

ENDLOCAL