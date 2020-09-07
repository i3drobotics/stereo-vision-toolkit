@echo off

SETLOCAL EnableDelayedExpansion
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%

:: set i3drsgm version
set version=1.0.2

:: set default option values
set option_clean=false
set option_uninstall=false
set option_token=false
set option_with_opencv=false
set option_with_phobos_integration=true
set token=""

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
    if "!arg!"=="--with-opencv" (
        set option_with_opencv=true
    )
    if "!arg!"=="--with-phobos-integration" (
        set option_with_phobos_integration=true
    )
    if "!arg!"=="--token" (
        set option_token=true
        set /a search_arg_num=!arg_count!+1
        set /a arg_count_b=0
        :: TODO check variable was found at index
        FOR %%b IN (%*) DO (
            Set /a arg_count_b+=1
            if !arg_count_b!==!search_arg_num! (
                set token=%%b
            )
        )
    )
)

if "%option_clean%"=="true" (
    echo Cleaning directory...
    rmdir /Q /S i3drsgm
)

if "%option_uninstall%"=="true" (
    exit /b %ERRORLEVEL%
)

:: search for token in first argument
set token_found=false
if "%option_token%"=="false" (
    :: token not stated in argument, look for it in token.txt file
    echo Reading PAT from 'token.txt'
    if exist "token.txt" (
        :: read github PAT from text file
        set /p token=< token.txt
        set token_found=true
    ) else (
        :: failed to file token.txt file
        echo Token file not found
    )
) else (
    set token_found=true
)

if "%token_found%"=="false" (
    echo GitHub token not found. Cannot download without token permission.
    exit /b 1
)

:: download phobos integration exe from github release using token
fetch --repo="https://github.com/i3drobotics/i3drsgm" --tag="v%version%" --release-asset="i3drsgm-%version%.exe" --progress --github-oauth-token=%token% ./
:: extract self-exracting archive
i3drsgm-%version%.exe -o"./" -y
:: delete downloaded file
del i3drsgm-%version%.exe

if "%option_with_opencv%"=="true" (
    :: install opencv
    call i3drsgm\opencv-3.4.1\install_opencv.bat
) else (
    rmdir /Q /S i3drsgm\opencv-3.4.1
)

if "%option_with_phobos_integration%"=="true" (
    :: install phobosIntegration
    call i3drsgm\phobosIntegration-1.0.54\install_phobosIntegration.bat %token%
) else (
    rmdir /Q /S i3drsgm\phobosIntegration-1.0.54
)

:: complete message
echo I3DRSGM install complete

:: reset working directory
cd %initcwd%

ENDLOCAL
