---
name: project-progress
description: Luận văn đã nộp bản v2 cho GVHD 2026-05-26, đang chờ phản hồi (tính đến 2026-08-12 vẫn chưa có), chưa có lịch bảo vệ
metadata:
  type: project
---

## Trạng thái tính đến 2026-08-12

**Đang chờ phản hồi GVHD.** Gửi bản v2 cho thầy Nam qua Teams ngày 2026-05-26.
Đến 2026-08-12 vẫn chưa có phản hồi — đã 2,5 tháng. Repo không có commit nào
trong khoảng này (ngoài đợt tái cấu trúc 2026-08-12).

**Chưa có lịch bảo vệ.** Kế hoạch gốc 16 tuần (02→06/2026) đã quá hạn và không
còn dùng làm mốc thời gian.

### Đã nộp (PDF tiếng Việt có dấu)
- `docs/thesis/thesis.pdf` — 43 trang, 6 chương, 20 hình
- `docs/reports/summary/summary_report.pdf` — 13 trang
- `docs/reports/sm2/sm2_report.pdf` — 15 trang
- `docs/reports/sm1/sm1_report.pdf` — 10 trang (báo cáo 2026-03-28)

Toàn bộ code mô phỏng + 19 figure PDF đã xong.

### Còn lại
Bị chặn bởi GVHD: hoàn thiện luận văn theo phản hồi, chuẩn bị slide bảo vệ.
Làm được ngay: cải thiện SMC (z_rms 0.276 còn cao, xem xét NTSMC).

**Why:** Trung hỏi lại tiến độ sau thời gian dài không đụng tới dự án — cần biết
ngay là đang chờ ai, và việc gì làm được mà không phải chờ.

**How to apply:** Khi thầy Nam phản hồi → tạo memory `feedback_*` mới cho nội dung
góp ý, rồi cập nhật file này. Đừng giả định đã bảo vệ xong hay đã có phản hồi
nếu không có bằng chứng — kiểm tra `git log` và ngày sửa file trước khi kết luận.

Related: [[feedback-advisor-nam]], [[feedback-vietnamese]], [[feedback-adp-ft-tuning]]
