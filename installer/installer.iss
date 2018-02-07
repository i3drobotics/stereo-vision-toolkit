; -- 64Bit.iss --
; Demonstrates installation of a program built for the x64 (a.k.a. AMD64)
; architecture.
; To successfully run this installation and the program it installs,
; you must have a "x64" edition of Windows.

; SEE THE DOCUMENTATION FOR DETAILS ON CREATING .ISS SCRIPT FILES!

[Setup]
AppId={{4A16961A-FA66-4C80-A04E-D5370D76763B}}
AppName=Stereo Vision Toolkit
AppVersion=1.0.0
AppPublisher=i3D Robotics Ltd.
AppPublisherURL=http://www.i3drobotics.com/
AppSupportURL=https://github.com/i3drobotics/stereo-vision-toolkit
AppUpdatesURL=https://github.com/i3drobotics/stereo-vision-toolkit/releases
DefaultDirName={pf64}/i3DR/Stereo Vision Toolkit
DefaultGroupName=i3D Robotics
LicenseFile=../LICENSE
OutputBaseFilename=svk_v1.0.0
SetupIconFile="../i3dr_logo.ico"
Compression=lzma2
SolidCompression=yes
ArchitecturesAllowed=x64
ArchitecturesInstallIn64BitMode=x64

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Files]
Source: "../build/release/*"; DestDir: "{app}"; Flags: ignoreversion createallsubdirs recursesubdirs
Source: "../build/release/vcredist_x64.exe"; DestDir: {tmp}; Flags: deleteafterinstall
Source: "../licenses/*"; DestDir: "{app}/licenses"; Flags: ignoreversion createallsubdirs recursesubdirs
Source: "../LICENSE"; DestDir: "{app}/licenses"

[Run]
Filename: {tmp}\vcredist_x64.exe; Parameters: "/q /passive /Q:a /c:""msiexec /q /i vcredist.msi"" "; StatusMsg: Installing VC++ Redistributable...;

[Icons]
Name: "{group}\{cm:UninstallProgram,Stereo Vision Toolkit}"; Filename: "{uninstallexe}"
Name: "{group}\i3DR Stereo Vision Toolkit"; Filename: "{app}\stereo-vision-toolkit.exe"; IconFilename: "{app}\i3dr_logo.ico"
Name: "{commondesktop}\i3DR Stereo Vision Toolkit"; Filename: "{app}\stereo-vision-toolkit.exe"; IconFilename: "{app}\i3dr_logo.ico"