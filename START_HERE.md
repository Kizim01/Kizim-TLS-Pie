# START HERE

Use this file to resume work quickly without starting from scratch.

Quick access folder:
- [START_HERE/README.md](START_HERE/README.md)

## 1. Open the correct repository
Open this folder in VS Code:
- C:/Users/sunun/Documents/GitHub/Kizim TLS Pie

## 2. Start a new Copilot chat with this prompt
Copy and paste this as your first message:

```text
Continue from this repository context. Read these first:
1) AI_HANDOFF_CHANGELOG.md
2) PROJECT_CONTEXT.md
3) AI_HANDOFF_CHECKLIST.md
Then summarize current status, pending hardware validation, and the next 3 recommended actions.
```

## 3. Read these files first (in order)
1. [AI_HANDOFF_CHANGELOG.md](AI_HANDOFF_CHANGELOG.md)
2. [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md)
3. [AI_HANDOFF_CHECKLIST.md](AI_HANDOFF_CHECKLIST.md)

## 4. Setup bundles you can use immediately
- [Pi_Setup_Package](Pi_Setup_Package)
- [Pi_Setup_Package.zip](Pi_Setup_Package.zip)
- [MicroView_Setup_Package](MicroView_Setup_Package)
- [MicroView_Setup_Package.zip](MicroView_Setup_Package.zip)

## 5. If working on Raspberry Pi setup
Read:
- [Pi_Setup_Package/TLS_Pie_Pi_Setup_Guide.md](Pi_Setup_Package/TLS_Pie_Pi_Setup_Guide.md)
- [Pi_Setup_Package/TLS_Pie_Pi_Setup_Checklist.md](Pi_Setup_Package/TLS_Pie_Pi_Setup_Checklist.md)

## 6. If working on MicroView firmware and OLED states
Read:
- [MicroView_Setup_Package/MicroView_OLED_Status_Reference.md](MicroView_Setup_Package/MicroView_OLED_Status_Reference.md)
- [MicroView_Setup_Package/MicroView_Quick_Setup_Checklist.md](MicroView_Setup_Package/MicroView_Quick_Setup_Checklist.md)
- [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)

## 7. Wiring note you must keep
Status return line for OLED abort/REC state:
- Pi GPIO22 -> level shifter -> MicroView D4 (PISTATUS)
- Shared ground is required.

## 8. Save chat progress so nothing gets lost
At end of each session:
1. Add a short summary to [AI_HANDOFF_CHANGELOG.md](AI_HANDOFF_CHANGELOG.md)
2. Commit changes
3. Push to your private repo

## 9. Optional one-liner to refresh Pi package zip
From repo root in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\Pi_Setup_Package\build_package.ps1
```

## 10. Optional one-liner to rebuild MicroView package zip
From repo root in PowerShell:

```powershell
if (Test-Path 'MicroView_Setup_Package.zip') { Remove-Item 'MicroView_Setup_Package.zip' -Force }; Compress-Archive -Path 'MicroView_Setup_Package\*' -DestinationPath 'MicroView_Setup_Package.zip' -Force
```
