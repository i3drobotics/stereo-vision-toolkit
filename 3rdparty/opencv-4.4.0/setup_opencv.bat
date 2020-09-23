@echo off

SETLOCAL EnableDelayedExpansion
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%
set opencv_version=4.4.0
SET "opencv_version_=%opencv_version:.=_%"

:: set default option values
set option_clean=false
set option_uninstall=false

:: find options in arguments
FOR %%a IN (%*) DO (
    set arg=%%a
    if "!arg!"=="--clean" (
        set option_clean=true
    )
    if "!arg!"=="--uninstall" (
        set option_clean=true
        set option_uninstall=true
    )
)

if "%option_clean%"=="true" (
    echo Cleaning directory...
    rmdir /Q /S opencv
)

if "%option_uninstall%"=="true" (
    exit /b %ERRORLEVEL%
)

:: extract version numbers
for /F "tokens=1,2,3 delims=." %%a in ("%opencv_version%") do (
   set version_major=%%a
   set version_minor=%%b
   set version_patch=%%c
)

set /a version_major_i=%version_major%
set /a version_minor_i=%version_minor%
set /a version_patch_i=%version_patch%

if %version_major_i% LEQ 2 (
    echo Cannot download opencv <= 2
    exit /b 1
)

:: download file
SET downloadfile=opencv-%opencv_version%-vc14_vc15.exe
:: different download file name for <= v3.3 (vc14 only)
if %version_major_i% EQU 3 (
    if %version_minor_i% LEQ 3 (
        SET downloadfile=opencv-%opencv_version%-vc14.exe
    )
)

:: url for downloading opencv
SET url=https://github.com/opencv/opencv/releases/download/%opencv_version%/%downloadfile%
echo %url%

:: folder where download is placed
SET downloadfolder=%cd%
:: full output filepath
SET downloadpath=%downloadfolder%\%downloadfile%

:: download opencv from url
curl -o "%downloadpath%" -L %url%

:: extract self-exracting archive
%downloadfile% -o%downloadfolder% -y
:: delete downloaded file
del %downloadfile%
echo OpenCV install complete.

:: reset working directory
cd %initcwd%

ENDLOCAL