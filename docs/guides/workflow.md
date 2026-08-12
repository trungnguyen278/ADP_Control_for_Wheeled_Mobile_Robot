# Quy trình làm việc

Từ sửa code đến ra PDF nộp thầy.

---

## Chuỗi phụ thuộc

```
  sm1_params.m          (tham số controller)
  wmr_params.m          (tham số robot)
        │
        ▼
  simulations/sim_*.m   ──chạy──►  results/*.mat      [KHÔNG commit]
        │
        ▼
  plotting/export_*.m   ──chạy──►  docs/**/figures/*.pdf   [CÓ commit]
        │
        ▼
  pdflatex ×2           ──────────►  docs/**/*.pdf     [CÓ commit]
```

Đổi tham số ở tầng trên → **phải chạy lại toàn bộ tầng dưới**, nếu không hình trong
luận văn sẽ không khớp với số liệu đang mô tả.

---

## 1. Chạy mô phỏng

MATLAB R2023a. Script dùng `addpath` tương đối nên **bắt buộc `cd` vào `simulations/`**.

```powershell
$MATLAB = "C:\Program Files\MATLAB\R2023a\bin\matlab.exe"
$REPO   = "c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot"

& $MATLAB -batch "cd('$REPO\simulations'); sim_thesis"
```

| Script | Nội dung | Thời gian | Sinh ra |
|---|---|---|---|
| `test_openloop` | 4 test mô hình vòng hở | ~10 s | `test_openloop_results.mat` |
| `sim_sm1` | SM1 kinematic, AC vs BS | ~1 phút | `sm1_results.mat` |
| `sim_sm2` | SM2 kinematic, 4 phương pháp × 2 quỹ đạo | ~2 phút | `sm2_results.mat` |
| `sim_sm1_full` | 5 phương pháp × 2 điều kiện, full model | ~3 phút | `sm1_full_results.mat` |
| `sim_thesis` | 3 quỹ đạo + robustness khối lượng + nhiễu | **~5 phút** | `thesis_results.mat` (300 MB) |

> `-batch` chạy rồi thoát, không mở GUI, in output ra terminal.
> Đặt timeout ≥ 300000 ms cho `sim_thesis`.

---

## 2. Xuất hình sang PDF

Chạy **sau** khi đã có `.mat` tương ứng. `cd` vào `plotting/`.

```powershell
& $MATLAB -batch "cd('$REPO\plotting'); export_all_figures"
```

| Script | Cần file | Ghi ra |
|---|---|---|
| `export_sm1_figures` | `sm1_results.mat` | `docs/reports/sm1/figures/` (7 hình) |
| `export_all_figures` | `sm2_results.mat`, `thesis_results.mat`, `sm1_full_results.mat` | `docs/reports/sm2/figures/` (6) + `docs/thesis/figures/` (13) |

Hình xuất dạng vector (`'ContentType','vector'`) — chữ trong hình vẫn sắc nét khi phóng to.

---

## 3. Build LaTeX

MiKTeX. Chạy `pdflatex` **2 lần** để cross-reference (`\ref`, `\cite`, mục lục) đúng.

```powershell
$env:PATH = "C:\Users\LEGION\AppData\Local\Programs\MiKTeX\miktex\bin\x64;$env:PATH"
Set-Location "$REPO\docs\thesis"
pdflatex -interaction=nonstopmode thesis.tex
pdflatex -interaction=nonstopmode thesis.tex
```

| Tài liệu | Thư mục |
|---|---|
| Luận văn | `docs/thesis/thesis.tex` |
| Báo cáo SM1 | `docs/reports/sm1/sm1_report.tex` |
| Báo cáo SM2 | `docs/reports/sm2/sm2_report.tex` |
| Tóm tắt cho GVHD | `docs/reports/summary/summary_report.tex` |

### Sau khi build, luôn kiểm tra

```powershell
Select-String -Path thesis.log -Pattern "Overfull|Undefined|LaTeX Warning: Reference"
```

- `Overfull \hbox` → bảng/công thức tràn lề, phải thu hẹp cột hoặc xuống dòng.
- `Undefined control sequence` → thiếu `\usepackage`.
- `Reference ... undefined` → chạy `pdflatex` thêm lần nữa.

### Bẫy thường gặp

- **Tiếng Việt mất dấu** → thiếu `\usepackage[T5]{fontenc}` và `\usepackage[utf8]{inputenc}`.
- **Không tìm thấy hình** → `.tex` dùng `\graphicspath{{figures/}}`, hình phải nằm
  trong `figures/` cùng cấp file `.tex`, và phải `cd` vào đúng thư mục trước khi build.

---

## 4. Chu trình đầy đủ khi đổi tham số

Ví dụ đổi gain của ADP-FT trong `simulations/sm1_params.m`:

```powershell
# 1. Chạy lại mô phỏng bị ảnh hưởng
& $MATLAB -batch "cd('$REPO\simulations'); sim_thesis"

# 2. Xuất lại hình
& $MATLAB -batch "cd('$REPO\plotting'); export_all_figures"

# 3. Build lại luận văn
$env:PATH = "C:\Users\LEGION\AppData\Local\Programs\MiKTeX\miktex\bin\x64;$env:PATH"
Set-Location "$REPO\docs\thesis"
pdflatex -interaction=nonstopmode thesis.tex; pdflatex -interaction=nonstopmode thesis.tex
```

```powershell
# 4. Cập nhật bảng số liệu trong thesis.tex + PROGRESS.md bằng kết quả mới, rồi commit
git add simulations/sm1_params.m
git commit -m "tune(adp-ft): giam robust gain cho dual-loop"
git add docs/ PROGRESS.md
git commit -m "docs(thesis): cap nhat figure va bang so lieu theo gain moi"
```

> **Bước 4 hay bị quên.** Số trong bảng LaTeX được gõ tay, không tự sinh từ `.mat`.
> Đổi tham số mà quên sửa bảng → luận văn mâu thuẫn với hình.

---

## 5. Trước khi nộp thầy

- [ ] Chạy lại toàn bộ chuỗi ở §4, không dùng hình cũ.
- [ ] Bảng số liệu trong `.tex` khớp với `PROGRESS.md`.
- [ ] `.log` sạch `Overfull` và `Undefined reference`.
- [ ] Tiếng Việt đủ dấu, kể cả trong caption hình và tiêu đề bảng.
- [ ] Hình không nằm giữa công thức và phần giải thích ký hiệu.
- [ ] Legend không che nội dung plot.
- [ ] Trích dẫn đúng: công thức của Wang / Fierro không được nhận là đóng góp
      (xem [CLAUDE.md §2](../../CLAUDE.md)).
- [ ] `git status` sạch, đã commit PDF sản phẩm.
