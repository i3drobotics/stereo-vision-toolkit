@echo off

SETLOCAL EnableDelayedExpansion
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%
cd ..

set "visual_studio_path=%programfiles(x86)%\Microsoft Visual Studio\2019\Community"
set "qt_creator_path=C:\Qt\Tools\QtCreator\bin"

:: setup visual studio variables
call "%visual_studio_path%\VC\Auxiliary\Build\vcvars64.bat"

mkdir build
cd build
:: release build
qmake.exe ..\stereo_vision_toolkit.pro -spec win32-msvc "CONFIG+=qtquickcompiler CONFIG+=DEV_BRANCH CONFIG+=WITH_VIMBA CONFIG+=WITH_I3DRSGM"
%qt_creator_path%\jom.exe
:: debug build
qmake.exe ..\stereo_vision_toolkit.pro -spec win32-msvc "CONFIG+=debug CONFIG+=qml_debug CONFIG+=qtquickcompiler CONFIG+=DEV_BRANCH CONFIG+=WITH_I3DRSGM"
%qt_creator_path%\jom.exe

:: reset working directory
cd %initcwd%

:: complete message
echo build complete.

ENDLOCAL