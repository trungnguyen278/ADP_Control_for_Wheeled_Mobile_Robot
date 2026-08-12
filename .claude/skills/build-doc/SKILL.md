---
name: build-doc
description: Compile luận văn hoặc báo cáo LaTeX sang PDF bằng MiKTeX, rồi kiểm tra log tìm lỗi Overfull hbox và undefined reference. Dùng khi cần build thesis.pdf, sm1_report.pdf, sm2_report.pdf, summary_report.pdf sau khi sửa file .tex. Trigger - "build luận văn", "compile latex", "chạy pdflatex", "xuất PDF", "build thesis", "build báo cáo".
---

# Build tài liệu LaTeX

## Đường dẫn MiKTeX

```
C:\Users\LEGION\AppData\Local\Programs\MiKTeX\miktex\bin\x64
```

## Bảng tài liệu

| Tài liệu | Thư mục | File chính |
|---|---|---|
| Luận văn | `docs/thesis/` | `thesis.tex` |
| Báo cáo SM1 | `docs/reports/sm1/` | `sm1_report.tex` |
| Báo cáo SM2 | `docs/reports/sm2/` | `sm2_report.tex` |
| Tóm tắt GVHD | `docs/reports/summary/` | `summary_report.tex` |

## Lệnh

**Phải `cd` vào thư mục chứa `.tex`** (vì `\graphicspath{{figures/}}` là tương đối)
và **chạy `pdflatex` 2 lần** (lần 1 sinh `.aux`, lần 2 mới giải đúng `\ref` và mục lục).

```powershell
$env:PATH = "C:\Users\LEGION\AppData\Local\Programs\MiKTeX\miktex\bin\x64;$env:PATH"
Set-Location "c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot\docs\thesis"
pdflatex -interaction=nonstopmode thesis.tex
pdflatex -interaction=nonstopmode thesis.tex
```

> `-interaction=nonstopmode` để không bị treo chờ nhập khi gặp lỗi.
> pdflatex trả exit code khác 0 kể cả khi chỉ có warning — đừng vội kết luận thất bại,
> phải đọc log.

## Kiểm tra log — BẮT BUỘC sau mỗi lần build

```powershell
Select-String -Path thesis.log -Pattern "Overfull|Underfull|Undefined control|LaTeX Warning: Reference"
```

| Thông báo | Nguyên nhân | Xử lý |
|---|---|---|
| `Overfull \hbox` | Bảng/công thức tràn lề | Thu hẹp cột, `\resizebox`, hoặc xuống dòng |
| `Undefined control sequence` | Thiếu `\usepackage` | Thêm package vào preamble |
| `LaTeX Warning: Reference ... undefined` | Chưa chạy đủ 2 lần | Chạy `pdflatex` thêm lần nữa |
| `File ... not found` | Sai đường dẫn hình | Kiểm tra hình có trong `figures/` cùng cấp `.tex` không |

Báo lại số trang cuối cùng của PDF cho user.

## Quy tắc nội dung (kiểm tra trước khi báo xong)

- **Tiếng Việt phải CÓ DẤU.** Preamble cần `\usepackage[utf8]{inputenc}` và
  `\usepackage[T5]{fontenc}`. Mất dấu = lỗi nghiêm trọng, GVHD yêu cầu cứng.
- **Không đặt hình giữa công thức và phần giải thích ký hiệu** — đặt ở điểm chuyển
  đoạn tự nhiên (cuối mục, trước subsection mới).
- **Legend không che nội dung plot** — đặt ngoài vùng vẽ.
- Với hình TikZ: kiểm tra tọa độ, nhãn không đè mũi tên/trục.

## Git

- **Commit**: `.tex`, `.pdf` sản phẩm, hình trong `figures/`.
- **Không commit**: `.aux`, `.log`, `.out`, `.toc` (đã có trong `.gitignore`).
- Commit message: `docs(thesis): ...` hoặc `docs(sm2): ...` — xem
  `docs/guides/commit-convention.md`.
