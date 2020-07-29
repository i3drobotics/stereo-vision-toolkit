@echo off

SETLOCAL EnableDelayedExpansion
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%

set boost_version=1.66.0
SET "boost_version_=%boost_version:.=_%"

:: set default option values
set option_clean=false
set option_uninstall=false
set option_mirror=""
set mirror=""

:: find options in arguments
set /a arg_count=0
FOR %%a IN (%*) DO (
    Set /a arg_count+=1
    set arg=%%a
    if "!arg!"=="--clean" (
        set option_clean=true
    )
    if "!arg!"=="--uninstall" (
        set option_clean=true
        set option_uninstall=true
    )
    if "!arg!"=="--mirror" (
        set option_mirror=true
        set /a search_arg_num=!arg_count!+1
        set /a arg_count_b=0
        :: TODO check variable was found at index
        FOR %%b IN (%*) DO (
            Set /a arg_count_b+=1
            if !arg_count_b!==!search_arg_num! (
                set mirror=%%b
            )
        )
    )
)

if "%option_mirror%"=="true" (
    set "mirror_str=?use_mirror=%mirror%"
)

:: download file
SET downloadfile=boost_%boost_version_%.tar.gz
:: url for downloading boost
set url=https://sourceforge.net/projects/boost/files/boost/%boost_version%/%downloadfile%/download%mirror_str%
:: folder where download is placed
SET downloadfolder=%cd%
:: full output filepath
SET downloadpath=%downloadfolder%\%downloadfile%

if "%option_clean%"=="true" (
    echo Cleaning directory...
    rmdir /Q /S boost_1_66_0
    del boost_%boost_version_%
)

if "%option_uninstall%"=="true" (
    exit /b %ERRORLEVEL%
)

echo Downloading boost...
:: download boost from url
curl -o "%downloadpath%" -L %url%

echo Boost download complete.

:: install boost in current directory
echo Extracting boost...
tar -xf %downloadfile%

echo Boost extraction complete.

echo Setting up boost
:: bootstrap boost
cd boost_%boost_version_%
cmd /c bootstrap

echo Building boost
:: build boost
set "boost_libs=--with-chrono --with-date_time --with-filesystem --with-iostreams --with-system --with-thread"
.\b2 %boost_libs% --toolset=msvc-14.1 architecture=x86 address-model=64 link=static

echo Boost build complete
cd ..

:: remove installer
del %downloadfile%

:: reset working directory
cd %initcwd%

EXIT /B %ERRORLEVEL%