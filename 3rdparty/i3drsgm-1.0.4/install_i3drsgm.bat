@echo off

SETLOCAL
:: set working directory to script directory
SET initcwd=%cd%
SET scriptpath=%~dp0
cd %scriptpath:~0,-1%

:: set i3drsgm version
set version=1.0.4

:: search for token in first argument
set token_found=false
if "%~1"=="" (
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
    :: token found in first argument, read token from argument
    set token=%~1
    set token_found=true
)

:: download phobos integration exe from github release using token
fetch --repo="https://github.com/i3drobotics/i3drsgm" --tag="v%version%" --release-asset="i3drsgm-%version%.exe" --progress --github-oauth-token=%token% ./
:: extract self-exracting archive
i3drsgm-%version%.exe -o"./" -y
:: delete downloaded file
del i3drsgm-%version%.exe

:: install phobosIntegration
call i3drsgm\phobosIntegration-1.0.54\install_phobosIntegration.bat %token%

:: complete message
echo I3DRSGM install complete

:: reset working directory
cd %initcwd%

ENDLOCAL
