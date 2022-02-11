@echo off

SETLOCAL
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd "%scriptpath:~0,-1%"
cd ..

:: define appcast file to update
set appcast_file=Appcast.xml
set index_file=index.html
set tmp_appcast_file=Appcast_tmp.xml
set tmp_index_file=tmp_index.html

:: read i3drsgm version from file
set /p version=< version.txt

:: update version in appcast file (and update timestamp)

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
set appcast_guid_link_line=19
set "appcast_guid_link=			^<guid isPermaLink=^"true^"^>https://raw.githubusercontent.com/i3drobotics/stereo-vision-toolkit/v%version%/release.md^</guid^>"
set appcast_link_line=22
set "appcast_link=			^<link^>https://raw.githubusercontent.com/i3drobotics/stereo-vision-toolkit/v%version%/release.md^</link^>"
set appcast_sparkle_link_line=23
set "appcast_sparkle_link=			^<sparkle:releaseNotesLink^>https://raw.githubusercontent.com/i3drobotics/stereo-vision-toolkit/v%version%/release.md^</sparkle:releaseNotesLink^>"
set appcast_fevor_url_line=31
set "appcast_fevor_url=				url=^"https://github.com/i3drobotics/stereo-vision-toolkit/releases/latest/download/StereoVisionToolkit-%version%-Win64.exe^""
set appcast_fervor_version_line=32
set "appcast_fervor_version=				fervor:version=^"%version%^""

echo %appcast_version%
echo %appcast_item_title%
echo %appcast_item_pubDate%
echo %appcast_fervor_version%

(for /f "tokens=1* delims=[]" %%a in ('find /n /v "##" ^< "%appcast_file%"') do (
    if "%%~a"=="%appcast_version_line%" (
        echo %appcast_version%
    ) else if "%%~a"=="%appcast_item_title_line%" (
        echo %appcast_item_title%
    ) else if "%%~a"=="%appcast_item_pubDate_line%" (
        echo %appcast_item_pubDate%
    ) else if "%%~a"=="%appcast_guid_link_line%" (
        echo %appcast_guid_link%
    ) else if "%%~a"=="%appcast_link_line%" (
        echo %appcast_link%
    ) else if "%%~a"=="%appcast_sparkle_link_line%" (
        echo %appcast_sparkle_link%
    ) else if "%%~a"=="%appcast_fevor_url_line%" (
        echo %appcast_fevor_url%
    ) else if "%%~a"=="%appcast_fervor_version_line%" (
        echo %appcast_fervor_version%
    ) ELSE (
        echo.%%b
    )
)) > %tmp_appcast_file%

copy %tmp_appcast_file% %appcast_file%
del %tmp_appcast_file%

:: update version in index file
set index_version_line=10
set "index_version=  ^<h3^>Latest release: %version%^</h3^>"
set index_version_link_line=11
set "index_version_link=  ^<button onclick=^"location.href='https://github.com/i3drobotics/stereo-vision-toolkit/releases/latest/download/StereoVisionToolkit-%version%-Win64.exe'^" type=^"button^"^>"

echo %index_version%
echo %index_version_link%

cd docs

(for /f "tokens=1* delims=[]" %%a in ('find /n /v "##" ^< "%index_file%"') do (
    if "%%~a"=="%index_version_line%" (
        echo %index_version%
    ) else if "%%~a"=="%index_version_link_line%" (
        echo %index_version_link%
    ) ELSE (
        echo.%%b
    )
)) > %tmp_index_file%

copy %tmp_index_file% %index_file%
del %tmp_index_file%

:: reset working directory
cd %initcwd%

:: complete message
echo update complete.

ENDLOCAL