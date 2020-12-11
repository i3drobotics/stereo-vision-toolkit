@echo off

SETLOCAL EnableDelayedExpansion
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%

:: install i3drsgm
call ..\3rdparty\i3drsgm\install_i3drsgm.bat

:: install boost (vorboss mirror specified for downloading from sourceforge optimized for EU)
:: see https://sourceforge.net/p/forge/documentation/Mirrors/ for sourceforge mirror list if this is slow for you
call ..\3rdparty\boost-1.66.0\setup_boost.bat --mirror vorboss

:: install opencv
call ..\3rdparty\opencv-contrib-cuda\setup_opencv.bat

:: reset working directory
cd %initcwd%

:: complete message
echo 3rdparty install complete.

ENDLOCAL