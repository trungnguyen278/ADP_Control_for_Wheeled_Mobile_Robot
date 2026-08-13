# CLAUDE.md

Hướng dẫn cho Claude Code khi làm việc trên repo này.

---

## 1. Bối cảnh dự án

Luận văn Thạc sĩ — Điều khiển & Tự động hóa, ĐHBK Hà Nội.
Học viên: Nguyễn Thành Trung. GVHD: PGS.TS. Nguyễn Hoài Nam.

Repo vừa là **nơi phát triển mô phỏng MATLAB**, vừa là **kho lưu trữ luận văn LaTeX**.
Mục tiêu chính là lưu trữ có hệ thống + tái tạo được kết quả, không phải sản phẩm phần mềm.

---

## 2. RANH GIỚI: bài báo gốc vs. đóng góp luận văn

> **Đây là mục quan trọng nhất của file này.** Luận văn khởi đầu bằng việc tái hiện một
> bài báo, sau đó mở rộng. Nhầm lẫn ranh giới này = tuyên bố sai về tính mới → lỗi học thuật.

**Bài báo gốc** (`references/Adaptive_Dynamic_Programming-Based_Fixed-Time_Optimal_Control_for_Wheeled_Mobile_Robot.pdf`
— giữ nguyên tên file tải về, không rút gọn):

> C. Wang, H. Zhan, Q. Guo, T. Li, "Adaptive Dynamic Programming-Based Fixed-Time
> Optimal Control for Wheeled Mobile Robot," *IEEE RA-L*, vol. 10, no. 1, pp. 176–183, 2025.

| | Bài báo Wang et al. 2025 | Luận văn (mở rộng) |
|---|---|---|
| Mô hình | Chỉ **kinematic** 3 trạng thái `[x,y,θ]` | **Full model 5 trạng thái** `[x,y,θ,v,ω]` |
| Đầu ra điều khiển | Gán trực tiếp `u = [v, ω]` | **Mô-men** `τ = [τ_R, τ_L]` |
| Kiến trúc | 1 vòng | **Dual-loop**: ADP ngoài + dynamic BS/PI trong |
| Nhiễu | Không xét | **Nhiễu sin tần số cao** cộng vào mô-men, 2 kênh |
| So sánh | — | **5 phương pháp** × 3 quỹ đạo × robustness (khối lượng, nhiễu) |
| Luật ADP eq.(16)-(18) | Đề xuất gốc | **Giữ nguyên**, chỉ tune lại gain cho dual-loop |

**Quy tắc khi viết luận văn / báo cáo:**

- Công thức `(16)-(18)`, phần chứng minh hội tụ fixed-time, cấu trúc critic-only
  → **của Wang et al.**, phải trích dẫn, KHÔNG được trình bày như đóng góp.
- Mô hình dynamic `M·η̇ = B·τ − F(η)` → **của Fierro & Lewis (1997)**, phải trích dẫn.
- Đóng góp thực sự của luận văn = **ghép nối** (dual-loop), **tune lại gain**,
  **phân tích robustness**, **so sánh 5 phương pháp**. Chỉ nhận phần này là mới.
- Khi Claude thêm nội dung mới vào `docs/thesis/thesis.tex`, phải tự hỏi:
  *"cái này của Wang, của Fierro, hay của Trung?"* — rồi đặt trích dẫn cho đúng.

Chi tiết gain đã tune lại: xem `memory/feedback_adp_ft_tuning.md`.

---

## 3. Quy tắc ngôn ngữ (bắt buộc)

| Nơi | Ngôn ngữ | Lý do |
|---|---|---|
| `docs/thesis/*.tex`, `docs/reports/**/*.tex` | **Tiếng Việt CÓ DẤU** | Yêu cầu cứng của GVHD. Dùng `\usepackage[T5]{fontenc}` |
| File `.md` (README, guides, memory) | Tiếng Việt có dấu | Dễ đọc |
| Commit message | Tiếng Việt **KHÔNG dấu** | Tránh lỗi encoding trên Windows/GitHub |
| Comment trong `.m` | Tiếng Việt **KHÔNG dấu** | MATLAB editor hay vỡ font UTF-8 |
| Tên biến, tên file | `snake_case` tiếng Anh | Nhất quán với code hiện có |

Không bao giờ mặc định viết tiếng Anh cho tài liệu nộp thầy.

---

## 4. Cấu trúc thư mục

```
models/         Mô hình WMR (kinematic, dynamic, full 5-state, tham số)
controllers/    5 bộ điều khiển vòng ngoài (kinematic) + 1 vòng trong (dynamic)
simulations/    Script chạy mô phỏng + tham số + quỹ đạo tham chiếu
plotting/       Script vẽ + xuất figure PDF
results/        File .mat — KHÔNG commit (xem §6)
docs/thesis/    Luận văn LaTeX (sản phẩm chính)
docs/reports/   Báo cáo seminar sm1/, sm2/, summary/
docs/guides/    Tài liệu quy ước repo
references/     Bài báo gốc + tài liệu tham khảo PDF
memory/         Bộ nhớ dài hạn của Claude (xem §8)
```

Đặc tả đầy đủ: [docs/guides/structure.md](docs/guides/structure.md)

**Không tự ý di chuyển `models/`, `controllers/`, `simulations/`, `plotting/`** —
các script MATLAB hardcode `addpath('../models', ...)`, đổi chỗ sẽ làm hỏng toàn bộ mô phỏng.

---

## 5. Chạy mô phỏng & build tài liệu

MATLAB R2023a, batch mode (không GUI):

```powershell
& "C:\Program Files\MATLAB\R2023a\bin\matlab.exe" -batch "cd('<repo>\simulations'); sim_thesis"
```

Build LaTeX (MiKTeX, chạy `pdflatex` 2 lần cho cross-reference):

```powershell
$env:PATH = "C:\Users\LEGION\AppData\Local\Programs\MiKTeX\miktex\bin\x64;$env:PATH"
pdflatex -interaction=nonstopmode thesis.tex; pdflatex -interaction=nonstopmode thesis.tex
```

Quy trình đầy đủ (sim → export figure → compile): [docs/guides/workflow.md](docs/guides/workflow.md)

**Timeout**: `sim_thesis.m` chạy ~5 phút. Luôn đặt timeout ≥ 300000 ms.

---

## 6. Quy tắc git

- **KHÔNG BAO GIỜ commit `results/*.mat`.** Các file này 80–300 MB, GitHub chặn cứng
  file > 100 MB. Chúng tái tạo được bằng cách chạy lại `simulations/sim_*.m`.
  `.gitignore` đã chặn — đừng dùng `git add -f` để lách.
- **KHÔNG commit** file build LaTeX (`.aux`, `.log`, `.out`, `.toc`).
- **CÓ commit** file PDF sản phẩm (`thesis.pdf`, `*_report.pdf`) và PDF hình trong
  `docs/**/figures/` — cần cho lưu trữ và để compile được.
- Commit theo Conventional Commits, mô tả tiếng Việt không dấu:
  `feat(controllers): them ADP fixed-time controller`

Đặc tả đầy đủ + bảng scope: [docs/guides/commit-convention.md](docs/guides/commit-convention.md)

---

## 7. Quy tắc viết LaTeX

- **Không đặt hình giữa công thức và phần giải thích ký hiệu.** Đặt hình ở điểm chuyển
  đoạn tự nhiên (cuối mục, trước subsection mới).
- **Legend không được che nội dung plot.** Đặt ra ngoài vùng vẽ (bên phải hoặc dưới).
- Với hình TikZ: kiểm tra tọa độ để nhãn không đè lên mũi tên/trục.
- Sau khi sửa `.tex`, luôn compile lại và kiểm tra `Overfull \hbox` trong `.log`.

---

## 8. Bộ nhớ (memory)

`memory/` trong repo **chính là** thư mục auto-memory của Claude
(`~/.claude/projects/<slug>/memory` là một directory junction trỏ vào đây).

Hệ quả:
- Memory được version trong git — sửa memory là một thay đổi cần commit.
- `memory/MEMORY.md` là chỉ mục, mỗi memory 1 file, link nội bộ bằng `[[ten-slug]]`.
- Commit memory dùng scope `memory`: `docs(memory): cap nhat tien do sau sprint`

Đọc memory hiện có trước khi hỏi lại Trung những thứ đã ghi (gain đã tune, yêu cầu
của thầy Nam, cách chạy MATLAB...).

---

## 9. Theo dõi tiến độ

`PROGRESS.md` là nhật ký tiến độ, **cập nhật mỗi khi hoàn thành một mốc công việc**:
ngày, nội dung, kết quả số (Jc, z_rms), mã commit.

---

## 10. Cách làm việc với Trung

- Giao tiếp tiếng Việt.
- **Giải thích lý thuyết trước khi code** — Trung muốn hiểu công thức rồi mới xem implementation.
- Đưa code đầy đủ chạy được, không pseudocode.
- Khi Trung paste lỗi → phân tích nguyên nhân gốc, không vá triệu chứng.
- Trung ít thời gian: ưu tiên chạy được + đúng số liệu, đừng viết doc dài không ai đọc.
