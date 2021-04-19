@echo off

SETLOCAL
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%
cd ..

:: define which appcast will be changed
set InputFile=Appcast.xml
set TmpFile=Appcast_tmp.xml

:: read i3drsgm version from file
set /p version=< version.txt

:: get date in correct format
for /f %%i in ('powershell ^(get-date^).DayOfWeek') do set dow=%%i
set dow=%dow:~0,3%
for /f %%i in ('powershell ^(get-date^).Day') do set day=%%i
::(Get-Culture).DateTimeFormat.GetMonthName((Get-Date).Month)
for /f %%i in ('powershell ^(get-Culture^).DateTimeFormat.GetMonthName^(^(get-date^).Month^)') do set month=%%i
for /f %%i in ('powershell ^(get-date^).Year') do set year=%%i
set mtime=%time:~0,5%

set "pubDate=%dow%, %day% %month% %year% %mtime% +0000"

set appcast_version_line=3
set "appcast_version=	version=^"%version%^""
set appcast_item_title_line=17
set "appcast_item_title=			^<title^>Version %version%^</title^>"
set appcast_item_pubDate_line=18
set "appcast_item_pubDate=			^<pubDate^>%pubDate%^</pubDate^>"
set appcast_fevor_url_line=31
set "appcast_fevor_url=				url=^"https://github.com/i3drobotics/stereo-vision-toolkit/releases/download/v%version%/StereoVisionToolkit-%version%-Win64.exe^""
set appcast_fervor_version_line=32
set "appcast_fervor_version=				fervor:version=^"%version%^""

echo %appcast_version%
echo %appcast_item_title%
echo %appcast_item_pubDate%
echo %appcast_fervor_version%

(for /f "tokens=1* delims=[]" %%a in ('find /n /v "##" ^< "%InputFile%"') do (
    if "%%~a"=="%appcast_version_line%" (
        echo %appcast_version%
    ) else if "%%~a"=="%appcast_item_title_line%" (
        echo %appcast_item_title%
    ) else if "%%~a"=="%appcast_item_pubDate_line%" (
        echo %appcast_item_pubDate%
    ) else if "%%~a"=="%appcast_fevor_url_line%" (
        echo %appcast_fevor_url%
    ) else if "%%~a"=="%appcast_fervor_version_line%" (
        echo %appcast_fervor_version%
    ) ELSE (
        echo.%%b
    )
)) > %TmpFile%

copy %TmpFile% %InputFile%
del %TmpFile%

:: reset working directory
cd %initcwd%

:: complete message
echo update complete.

ENDLOCAL