---
name: reference-matlab-cli
description: How to run MATLAB scripts from terminal on this machine (R2023a, batch mode, no GUI)
metadata:
  type: reference
---

MATLAB R2023a path: `C:\Program Files\MATLAB\R2023a\bin\matlab.exe`

Chay script tu terminal (batch mode, khong GUI):
```powershell
& "C:\Program Files\MATLAB\R2023a\bin\matlab.exe" -batch "cd('c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot\simulations'); sim_sm1_full"
```

- `-batch "script"`: chay script roi thoat, khong mo GUI, in output ra terminal
- Can `cd(...)` truoc vi MATLAB bat dau o thu muc mac dinh
- Timeout nen de ~5 phut (300s) cho simulation dai
- Output (figures, .mat) luu binh thuong nhu chay trong MATLAB GUI
