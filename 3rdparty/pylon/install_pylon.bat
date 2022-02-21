@echo off

SETLOCAL
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd "%scriptpath:~0,-1%"

:: set i3drsgm version
set version=6.3.0.23157

:: url for downloading opencv
SET url="https://github.com/i3drobotics/pylon-build/releases/download/%version%/pylon-%version%-win64.zip"
:: output folder where donwload is placed
SET "outputfolder=%cd%"
:: full output filepath
SET "output=%outputfolder%\pylon-%version%-win64.zip"
:: download opencv from url
curl -o "%output%" -L %url%
:: extract archive
powershell Expand-Archive "%output%" -DestinationPath "%outputfolder%"
:: delete downloaded file
del "%output%"

:: complete message
echo Pylon install complete

:: reset working directory
cd "%initcwd%"

ENDLOCAL
