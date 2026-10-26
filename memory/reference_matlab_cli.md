---
name: reference-matlab-cli
description: Chạy MATLAB R2023a (C:\MATLAB\R2023a) và MiKTeX (C:\Program Files\MiKTeX) từ terminal; bẫy `clear` khi bọc lệnh; chạy song song được
metadata:
  type: reference
---

Đường dẫn trên máy (kiểm tra lại 2026-10-26, đường dẫn cũ `C:\Program Files\MATLAB\...` và
`AppData\Local\Programs\MiKTeX` không còn tồn tại):

- MATLAB R2023a: `C:\MATLAB\R2023a\bin\matlab.exe` (cũng có trong PATH: `matlab`)
- MiKTeX pdflatex: `C:\Program Files\MiKTeX\miktex\bin\x64\pdflatex.exe`
- Python 3.12 có sẵn (`python`).

```powershell
& "C:\MATLAB\R2023a\bin\matlab.exe" -batch "cd('c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot\simulations'); sim_thesis; sim_sm1_full"
```

- `-batch`: chạy rồi thoát, không GUI, in ra stdout (chuyển hướng vào log khi chạy nền).
- Phải `cd(...)` vì script dùng `addpath('../models', ...)` tương đối.
- **Bẫy:** mọi script `sim_*.m` bắt đầu bằng `clear`, nên lệnh bọc kiểu
  `t0=tic; sim_thesis; toc(t0)` báo lỗi *sau khi* script đã lưu xong, và các lệnh phía sau
  (vd. `sim_sm1_full`) bị bỏ qua. Dùng `disp('XONG')` thay cho biến timer.
- Ba tiến trình MATLAB chạy song song được (máy 16 luồng, 15 GB RAM — RAM là giới hạn,
  `sim_thesis_extra` giữ ~1,5 GB kết quả). Thời gian đo 2026-10-26: `sim_thesis` ~5 phút,
  `sim_sm1_full` ~2 phút, `sim_thesis_extra` ~20 phút, `sweep_gains` ~6 phút.
- MATLAB và Octave 8.4 cho kết quả trùng tới 4 chữ số (đã đối chiếu Phần M, 2026-10-26).
- Đừng chạy lại `sim_sm2`: xem [[project-progress]] — hình SM2 đã nộp phải giữ nguyên.

Related: [[project-progress]]
