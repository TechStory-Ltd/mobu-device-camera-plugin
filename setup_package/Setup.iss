; Installer of the TechStory OSC Camera Tracker device plugin for Autodesk MotionBuilder.
; Build: open in Inno Setup 6 and compile, or run  ISCC.exe Setup.iss
;
; Expected layout next to this folder, one dll per MotionBuilder version (run ..\build_all.bat to produce them):
;   ..\bin\<MotionBuilder version>\device_oscCamera.dll   - the plugin built with -DMOBU_VERSION=<version>

#define MyAppName "TechStory OSC Camera Tracker Device for Autodesk MotionBuilder"
#define MyAppVersion "1.0"
#define MyAppPublisher "TechStory Ltd"
#define MyAppExeName "Setup_TechStoryOSCCamera_MoBu"
#define MyPluginDll "device_oscCamera.dll"

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"; InfoBeforeFile:"infoBefore_en.txt"; InfoAfterFile: "infoAfter_en.txt"

[Setup]
; NOTE: The value of AppId uniquely identifies this application.
; Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{7C3B5E8A-9D41-4F6B-A2E7-3D58B1C6F094}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
;AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
CreateAppDir=no
OutputBaseFilename={#MyAppExeName}
Compression=lzma
SolidCompression=yes
AppCopyright=Copyright (C) 2026 TechStory Ltd
SetupIconFile=techstory.ico
WizardImageFile=techstory_title.bmp
WizardImageStretch=no
WizardSmallImageFile=techstory_title_sm.bmp
DisableWelcomePage=no
; the plugin goes under Program Files of MotionBuilder
PrivilegesRequired=admin
ArchitecturesInstallIn64BitMode=x64

[Types]
Name: "full"; Description: "Full installation"
Name: "2017"; Description: "MotionBuilder 2017"
Name: "2018"; Description: "MotionBuilder 2018"
Name: "2019"; Description: "MotionBuilder 2019"
Name: "2022"; Description: "MotionBuilder 2022"
Name: "2023"; Description: "MotionBuilder 2023"
Name: "2024"; Description: "MotionBuilder 2024"
Name: "2025"; Description: "MotionBuilder 2025"
Name: "2026"; Description: "MotionBuilder 2026"
Name: "2027"; Description: "MotionBuilder 2027"

Name: "custom"; Description: "Custom installation"; Flags: iscustom

[Components]
Name: "MotionBuilder2017"; Description: "MotionBuilder 2017 plugin"; Types: full 2017 custom;
Name: "MotionBuilder2018"; Description: "MotionBuilder 2018 plugin"; Types: full 2018 custom;
Name: "MotionBuilder2019"; Description: "MotionBuilder 2019 plugin"; Types: full 2019 custom;
Name: "MotionBuilder2022"; Description: "MotionBuilder 2022 plugin"; Types: full 2022 custom;
Name: "MotionBuilder2023"; Description: "MotionBuilder 2023 plugin"; Types: full 2023 custom;
Name: "MotionBuilder2024"; Description: "MotionBuilder 2024 plugin"; Types: full 2024 custom;
Name: "MotionBuilder2025"; Description: "MotionBuilder 2025 plugin"; Types: full 2025 custom;
Name: "MotionBuilder2026"; Description: "MotionBuilder 2026 plugin"; Types: full 2026 custom;
Name: "MotionBuilder2027"; Description: "MotionBuilder 2027 plugin"; Types: full 2027 custom;

[Files]

; 2017
Source: "..\bin\2017\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2017}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2017

; 2018
Source: "..\bin\2018\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2018}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2018

; 2019
Source: "..\bin\2019\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2019}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2019

; 2022
Source: "..\bin\2022\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2022}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2022

; 2023
Source: "..\bin\2023\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2023}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2023

; 2024
Source: "..\bin\2024\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2024}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2024

; 2025
Source: "..\bin\2025\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2025}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2025

; 2026
Source: "..\bin\2026\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2026}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2026

; 2027
Source: "..\bin\2027\{#MyPluginDll}"; DestDir: "{code:MoBu_Path64|2027}\bin\x64\plugins\"; Flags: ignoreversion; Components: MotionBuilder2027


; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Code]

function MoBu_Path32(Param: String): String;
var
  key, path: String;
begin
  key := 'Software\Autodesk\MotionBuilder\' + Param;
  Result := '';
  if RegKeyExists(HKEY_LOCAL_MACHINE, key) then
  begin
    If RegQueryStringValue(HKEY_LOCAL_MACHINE, key, 'InstallPath', path) then
    begin
      Result := path;
    end;
  end;
end;

function MoBu_Path64(Param: String): String;
var
  key, path: String;
begin
  key := 'Software\Autodesk\MotionBuilder\' + Param;
  Result := '';
  if IsWin64 and RegKeyExists(HKLM64, key) then
  begin
    If RegQueryStringValue(HKLM64, key, 'InstallPath', path) then
    begin
      Result := path;
    end;
  end;
end;

function IsMoBu(Param: String): Boolean;
begin
  Result := False;
  if RegKeyExists(HKLM, 'Software\Autodesk\MotionBuilder\' + Param) then
  begin
    Result := True;
  end;
end;

function IsMoBu64(Param: String) : Boolean;
begin
  Result := False;
  if IsWin64 and RegKeyExists(HKLM64, 'Software\Autodesk\MotionBuilder\' + Param) then
  begin
    Result := True;
  end;
end;
