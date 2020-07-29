@echo off

SETLOCAL EnableDelayedExpansion
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%

:: set default option values
set option_clean=false
set option_uninstall=false
set option_with_i3drsgm=false

:: find options in arguments
FOR %%a IN (%*) DO (
    set arg=%%a
    if "!arg!"=="--clean" (
        set option_clean=true
    )
    if "!arg!"=="--uninstall" (
        set option_uninstall=true
    )
    if "!arg!"=="--with-i3drsgm" (
        set option_with_i3drsgm=true
    )
)

set script_params=""
if "%option_clean%"=="true" (
    set "script_params=--clean"
)
if "%option_uninstall%"=="true" (
    set "script_params=--uninstall"
)

if "%option_with_i3drsgm%"=="true" (
    :: install i3drsgm (also install phobos-integration dependency but not opencv)
    call ..\3rdparty\i3drsgm-1.0.2\setup_i3drsgm.bat %script_params% --with-phobos-integration
)

:: install boost (vorboss mirror specified for downloading from sourceforge optimized for EU)
:: see https://sourceforge.net/p/forge/documentation/Mirrors/ for sourceforge mirror list if this is slow for you
call ..\3rdparty\boost-1.66.0\setup_boost.bat %script_params% --mirror vorboss

:: install opencv
call ..\3rdparty\opencv-3.4.10\setup_opencv.bat %script_params%

:: reset working directory
cd %initcwd%

:: complete message
echo 3rdparty install complete.

ENDLOCAL