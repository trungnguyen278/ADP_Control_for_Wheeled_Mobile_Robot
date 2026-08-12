---
name: feedback-baseline-fairness
description: SMC baseline trong luận văn bị tune tệ 7x; khi tune công bằng thì SMC/NTSMC thắng ADP-FT ở mọi mức nhiễu và ADP-FT sụp ở sai lệch khối lượng
metadata:
  type: project
---

Phát hiện 2026-08-12 khi sweep gain (140+ lần chạy full model, circle T=60s).

## Vấn đề

Baseline SMC trong luận văn dùng `lambda=3, eta=1.0` → Jc=555 (20% nhiễu).
Sweep cho `lambda=0.7, eta=0.05` → **Jc=79.9**. Tốt hơn **7.0 lần chỉ nhờ đổi gain**.

Khi tune công bằng:
- **Jc**: SMC(tuned) 79.9 < NTSMC(tuned) 80.2 < ADP-FT 85.2 < BS 90.1 < CL 101.1
  → SMC thắng ADP-FT ở **mọi** mức nhiễu 0/20/40/60%.
- **z_rms 5s cuối**: SMC(tuned) 0.00011 vs ADP-FT 0.046 → ADP-FT kém ~400 lần.
- **Sai lệch khối lượng** (controller tưởng m=10): ADP-FT **sụp ở 14 kg**
  (Jc=4283, z_rms=3.03, mất bám) còn SMC/NTSMC giữ Jc≈265, z_rms≈1e-4.
  14 kg NẰM TRONG Phần B của luận văn → dữ liệu này đã có sẵn trong luận văn.

## NTSMC không giúp gì

Optimum của NTSMC nằm ở `beta→∞`, tức **tắt hẳn** số hạng terminal. `alpha_s` không
ảnh hưởng gì (4 giá trị cho kết quả giống hệt chữ số). Cải thiện đến từ **giảm gain**,
không phải từ cấu trúc terminal.

**Why:** Đây là lỗ hổng lớn nhất có thể bị hỏi khi bảo vệ: "thầy tune gain SMC thế nào?".
Nếu không chuẩn bị trước thì kết luận trung tâm của luận văn bị lung lay ngay tại hội đồng.

**How to apply:**
- KHÔNG được viết "ADP-FT có Jc tốt nhất" hay "NTSMC tốt hơn SMC nhờ hội tụ hữu hạn".
- Hai lập luận CÒN bảo vệ được cho ADP-FT: (1) đạt kết quả tương đương mà không cần
  sweep offline 140 lần chạy và không cần biết mô hình; (2) điểm bán là *bảo đảm*
  hội tụ cố định thời gian, không phải Jc nhỏ nhất. Nên đặt lại tuyên bố theo 2 hướng này.
- Gain SMC trong `ctrl_params.m` CHƯA sửa — đổi nó sẽ thay toàn bộ bảng số liệu, hình
  và kết luận của luận văn đã nộp. Là quyết định của Trung.
- Số liệu chi tiết: xem PROGRESS.md mục 2026-08-12.

Related: [[project-progress]], [[feedback-adp-ft-tuning]], [[feedback-advisor-nam]]
