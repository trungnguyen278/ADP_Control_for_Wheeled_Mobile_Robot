---
name: run-sim
description: Chạy mô phỏng MATLAB của luận văn (sim_thesis, sim_sm1, sim_sm2, sim_sm1_full, test_openloop) và xuất figure PDF. Dùng khi cần chạy lại mô phỏng, cập nhật kết quả sau khi đổi tham số controller, hoặc tái tạo file results/*.mat đã bị xóa. Trigger - "chạy sim", "chạy lại mô phỏng", "run simulation", "xuất figure", "export figures", "cập nhật kết quả".
---

# Chạy mô phỏng MATLAB

## Đường dẫn

```
MATLAB: C:\Program Files\MATLAB\R2023a\bin\matlab.exe
REPO:   c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot
```

## Quy tắc bắt buộc

1. **Phải `cd` vào đúng thư mục** trước khi gọi script — mọi script dùng
   `addpath('../models', ...)` là đường dẫn tương đối cứng.
2. **Dùng `-batch`**, không `-r` (tránh treo khi có lỗi, không mở GUI).
3. **Timeout ≥ 300000 ms** cho `sim_thesis`; ≥ 180000 ms cho các sim khác.
4. Chạy nền (`run_in_background: true`) nếu sim dài, để user còn tương tác được.

## Lệnh

```powershell
& "C:\Program Files\MATLAB\R2023a\bin\matlab.exe" -batch "cd('c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot\simulations'); <TEN_SCRIPT>"
```

Xuất figure — đổi `simulations` thành `plotting`:

```powershell
& "C:\Program Files\MATLAB\R2023a\bin\matlab.exe" -batch "cd('c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot\plotting'); <TEN_SCRIPT>"
```

## Bảng script

| Script | Thư mục | Thời gian | Sinh ra | Cần trước |
|---|---|---|---|---|
| `test_openloop` | simulations | ~10 s | `test_openloop_results.mat` | — |
| `sim_sm1` | simulations | ~1 ph | `sm1_results.mat` | — |
| `sim_sm2` | simulations | ~2 ph | `sm2_results.mat` | — |
| `sim_sm1_full` | simulations | ~3 ph | `sm1_full_results.mat` | — |
| `sim_thesis` | simulations | **~5 ph** | `thesis_results.mat` (300 MB) | — |
| `export_sm1_figures` | plotting | ~20 s | `docs/reports/sm1/figures/` | `sim_sm1` |
| `export_all_figures` | plotting | ~1 ph | `docs/reports/sm2/figures/`, `docs/thesis/figures/` | `sim_sm2`, `sim_thesis`, `sim_sm1_full` |

## Sau khi chạy

- Kiểm tra output terminal có lỗi MATLAB không (`Error using`, `Undefined function`).
- Nếu đổi tham số → chạy sim → **phải chạy tiếp `export_*`** rồi build lại LaTeX,
  nếu không hình trong luận văn sẽ lệch với số liệu. Xem skill `build-doc`.
- Cập nhật `PROGRESS.md` với số liệu mới (Jc, z_rms) và ngày chạy.
- **Không `git add` file `.mat`** — đã bị `.gitignore` chặn, đó là cố ý.

## Khi thiếu file .mat

`results/*.mat` không có trong git (quá lớn). Nếu `export_*` báo không tìm thấy file
`.mat`, phải chạy lại script `sim_*` tương ứng ở cột "Cần trước".
