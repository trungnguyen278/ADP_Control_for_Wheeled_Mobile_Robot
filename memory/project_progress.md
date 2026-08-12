---
name: project-progress
description: Thesis deliverable status — thesis v2 43pp submitted to advisor 2026-05-26; repo restructured 2026-08-12
metadata:
  type: project
---

## Tiến độ tính đến 2026-08-12

### ĐÃ HOÀN THÀNH
- Tuần 1-3: Mô hình + SM1 Actor-Critic + Báo cáo SM1
- Sprint 2026-05-06: 5 phương pháp x full model, tune tham số
- Sprint 2026-05-25: SM2 sim, thesis sim, robustness analysis, export figures, viết báo cáo
- 2026-05-26: Cải thiện luận văn v2 — logo trường, 8+ hình TikZ, sửa figure placement, mở rộng nội dung

### DELIVERABLES (compiled PDF, tiếng Việt có dấu)
- **thesis.pdf** — 43 trang, 6 chương, 20+ hình (gồm 8 TikZ + 12 MATLAB PDF)
- **summary_report.pdf** — 13 trang (đã bỏ mục 7.2 việc cần làm + 7.3 timeline)
- **sm2_report.pdf** — 15 trang (Critic-only + Concurrent Learning)
- **19 figures PDF** — exported từ MATLAB .mat data

### SỬA LỖI HÌNH (2026-05-26)
- Figure placement: dời hình ra khỏi giữa công thức và giải thích (ADP structure, architecture, CL concept)
- Reference trajectories: legend ra ngoài plot (bên phải) tránh che quỹ đạo
- Body frame error (Hình 2.3): thiết kế lại 3 lần — tách θ arc, zx arrow, X_B label
- Parameter tuning table: thu hẹp cột tránh overfull hbox

### CHƯA LÀM
- [ ] Cải thiện SMC (z_rms còn cao, xem xét NTSMC)
- [ ] Chuẩn bị slide bảo vệ
- [ ] Hoàn thiện luận văn sau phản hồi thầy

### TÁI CẤU TRÚC REPO (2026-08-12)
- `results/*.mat` (565MB) đã gỡ khỏi git tracking — GitHub chặn cứng file >100MB,
  commit cũ 2b6cfef không push được. Giờ tái tạo bằng `sim_*.m`.
- Thư mục mới: `references/` (bài báo gốc), `docs/reports/{sm1,sm2,summary}/`, `docs/guides/`
- Thêm `CLAUDE.md`, `.claude/settings.json`, 3 skills: `run-sim`, `build-doc`, `checkpoint`
- `memory/` giờ là junction từ `~/.claude/projects/<slug>/memory` → memory được version trong git
- Commit theo Conventional Commits, mô tả tiếng Việt không dấu

### TRẠNG THÁI HIỆN TẠI
Đã gửi báo cáo cho thầy Nam qua Teams (2026-05-26). Chưa ghi nhận phản hồi.

**Why:** Goal "báo cáo nộp cho thầy cùng báo cáo luận văn bản v1" — đã hoàn thành và gửi.
**How to apply:** Đây là bản v2. Dự kiến sẽ có chỉnh sửa sau khi thầy review.
Khi có feedback mới của thầy → ghi vào memory `feedback_*` rồi cập nhật file này.

Related: [[feedback-vietnamese]], [[feedback-advisor-nam]], [[feedback-adp-ft-tuning]]
