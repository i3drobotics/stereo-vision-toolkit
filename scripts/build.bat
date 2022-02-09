@echo off

SETLOCAL EnableDelayedExpansion
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%
cd ..

set "visual_studio_path=%programfiles(x86)%\Microsoft Visual Studio\2019\Community"

:: setup visual studio variables
call "%visual_studio_path%\VC\Auxiliary\Build\vcvars64.bat"

mkdir build
cd build
:: release build
qmake.exe "CONFIG+=qtquickcompiler WITH_I3DRSGM SHOW_CONSOLE" ..\stereo_vision_toolkit.pro -spec win32-msvc
jom.exe
:: debug build
qmake.exe "CONFIG+=debug qml_debug qtquickcompiler" ..\stereo_vision_toolkit.pro -spec win32-msvc
jom.exe

:: reset working directory
cd %initcwd%

:: update version in docs & appcast
call "scripts\update_version.bat"

:: complete message
echo build complete.

ENDLOCAL