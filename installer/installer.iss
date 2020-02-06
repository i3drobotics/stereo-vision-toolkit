; -- Stereo Vision Toolkit (Pro) --
; Inno installer script for Stereo Vision Toolkit
; MUST be installed on x64 bit machine

#define AppName "Stereo Vision Toolkit"
#define AppVersion "1.2.3"
#define InstallerName "Stereo Vision Toolkit Installer"
#define ExeName "StereoVisionToolkit.exe"
#define IconName "i3dr_logo.ico"

[Setup]
AppId={{IT5MN0J1-DHW4-I62K-FYU-9E4SMMFRFDYL}}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher=i3D Robotics Ltd.
AppPublisherURL=http://www.i3drobotics.com/
AppSupportURL=https://github.com/i3drobotics/stereo_vision_toolkit
AppUpdatesURL=https://github.com/i3drobotics/stereo_vision_toolkit/releases
DefaultDirName={pf64}/i3DR/{#AppName}
DefaultGroupName=i3D Robotics
LicenseFile=../LICENSE
OutputBaseFilename={#InstallerName}
SetupIconFile="../{#IconName}"
Compression=lzma2
SolidCompression=yes
ArchitecturesAllowed=x64
ArchitecturesInstallIn64BitMode=x64

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "../build/release/*"; Excludes: "\params\*.xml,*.lic,install_drivers.bat"; DestDir: "{app}"; Flags: ignoreversion createallsubdirs recursesubdirs
Source: "../build/release/vc_redist.x64.exe"; DestDir: {tmp}; Flags: deleteafterinstall
Source: "../build/release/pylon_USB_Camera_Driver.msi"; DestDir: {tmp}; Flags: deleteafterinstall
Source: "../build/release/pylon_GigE_Filter_Driver.msi"; DestDir: {tmp}; Flags: deleteafterinstall
Source: "../build/release/pylon_GigE_Performance_Driver.msi"; DestDir: {tmp}; Flags: deleteafterinstall
Source: "../licenses/*"; DestDir: "{app}/licenses"; Flags: ignoreversion createallsubdirs recursesubdirs
Source: "../LICENSE"; DestDir: "{app}/licenses"
Source: "../{#IconName}"; DestDir: "{app}"

[Run]
Filename: {tmp}\vc_redist.x64.exe; Parameters: "/q /passive /Q:a /c:""msiexec /q /i vcredist.msi"" "; StatusMsg: Installing VC++ Redistributable...;
Filename: "msiexec.exe"; Parameters: "/q /i ""{tmp}\pylon_USB_Camera_Driver.msi"" /qb"; WorkingDir: {tmp}; StatusMsg: Installing Pylon USB Camera Driver...;
Filename: "msiexec.exe"; Parameters: "/q /i ""{tmp}\pylon_GigE_Filter_Driver.msi"" /qb"; WorkingDir: {tmp}; StatusMsg: Installing Pylon GigE Filter Driver...;
Filename: "msiexec.exe"; Parameters: "/q /i ""{tmp}\pylon_GigE_Performance_Driver.msi"" /qb"; WorkingDir: {tmp}; StatusMsg: Installing Pylon GigE Performance Driver...;

[Icons]
Name: "{group}\{cm:UninstallProgram,{#AppName}}"; Filename: "{uninstallexe}"
Name: "{group}\{#AppName}"; Filename: "{app}\{#ExeName}"; IconFilename: "{app}\{#IconName}"
Name: "{commondesktop}\{#AppName}"; Filename: "{app}\{#ExeName}"; IconFilename: "{app}\{#IconName}"; Tasks: desktopicon