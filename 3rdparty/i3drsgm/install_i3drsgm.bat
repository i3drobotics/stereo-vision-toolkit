@echo off

SETLOCAL
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%

:: set i3drsgm version
set version=1.0.13

:: url for downloading opencv
SET url="https://github.com/i3drobotics/i3drsgm/releases/download/v%version%/i3drsgm-%version%.exe"
:: output folder where donwload is placed
SET "outputfolder=%cd%"
:: full output filepath
SET "output=%outputfolder%\i3drsgm-%version%.exe"
:: download opencv from url
curl -o "%output%" -L %url%
:: extract self-exracting archive
%output% -o%outputfolder% -y
:: delete downloaded file
del %output%

:: install opencv (removed as internal opencv is used instead)
:: call i3drsgm\opencv-4.5.0\install_opencv.bat

:: install phobosIntegration
call i3drsgm\phobosIntegration-1.0.54\install_phobosIntegration.bat %token%

:: complete message
echo I3DRSGM install complete

:: reset working directory
cd %initcwd%

ENDLOCAL
