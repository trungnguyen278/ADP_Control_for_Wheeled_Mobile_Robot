---
name: project-progress
description: Thầy Nam chỉ có bản v2 (26/05/2026, chưa phản hồi); bản 78 trang (13/08) chưa gửi; 26/10/2026 phát hiện lỗi ma trận B nên phải chạy lại toàn bộ kết quả trước khi đi tiếp
metadata:
  type: project
---

## Cập nhật 2026-10-26

Sau 2,5 tháng không động tới, Trung hỏi lại tiến độ. Kiểm tra cho thấy:
- Bản mới nhất: `thesis.pdf` 78 trang, build 2026-08-13, chưa ghi nhận đã gửi thầy.
  Thầy Nam vẫn chỉ có v2 (43 trang, commit `7fedea9`); 5 tháng chưa phản hồi.
- Bản 78 trang tự mâu thuẫn: đợt thêm ghép nonholonomic (tối 13/08) chưa lan hết sang
  Ch.5–6 (vd. `thesis.tex:1564`, `:2354`, `:2366`, `:2226`, `:2439`, `:2474–2489`).
  Báo cáo tóm tắt còn cũ hơn (vẫn ghi "ADP-FT tốt nhất").
- **Quan trọng hơn:** mô hình có lỗi ma trận B → xem [[feedback-b-matrix-error]].
  Phải sửa mô hình và chạy lại trước, sửa chữ sau.

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
