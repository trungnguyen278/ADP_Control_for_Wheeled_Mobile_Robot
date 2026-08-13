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
- Lập luận duy nhất CÒN bảo vệ được: ADP-FT đạt kết quả **cùng bậc** mà không cần
  sweep offline 140 lần chạy và không cần biết mô hình. Lập luận thứ hai trước đây
  ("điểm bán là bảo đảm hội tụ cố định thời gian") **KHÔNG dùng được nữa** — xem
  [[feedback-fixedtime-unverified]], tính chất đó không đạt trên dual-loop.

## Cập nhật 2026-08-13: đã đưa vào luận văn

Quyết định của Trung: giữ nguyên gain SMC mặc định, nhưng **thêm hẳn một mục
so sánh công bằng** (§5.10 của luận văn) thay vì chỉ ghi caveat. Số liệu chạy lại:

| | circle | line | figure8 |
|---|---|---|---|
| SMC thông dụng (λ=3, η=1) | 555,5 | 64,7 | 2457,6 |
| SMC đã tune (λ=0,7, η=0,05) | **79,9** | **5,5** | **8,5** |
| ADP-FT | 85,2 | 8,5 | 93,2 |

Theo mức nhiễu (circle): SMC tune 61,8 / 79,9 / 107,3 / 150,9 — thắng ADP-FT
(70,7 / 85,2 / 111,8 / 157,7) ở **mọi** mức. z_rms: 1,07e-4 vs 4,63e-2 (433 lần).

Luận văn đã đặt lại tuyên bố trung tâm cho đúng. Bài học phương pháp luận được
nâng thành một đóng góp: *chất lượng tune tham số có thể lấn át khác biệt bản chất
giữa các phương pháp* — cảnh báo áp dụng cho mọi nghiên cứu so sánh bộ điều khiển.

**Việc còn thiếu:** chưa khảo sát độ bền của SMC-đã-tune với khối lượng, z0, nhiễu
2 kênh. Đây là hướng số 1 ở §6.3 vì nó quyết định kết luận cuối của đề tài.

Related: [[project-progress]], [[feedback-adp-ft-tuning]], [[feedback-advisor-nam]]
