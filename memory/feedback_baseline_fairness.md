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

## Cập nhật 2026-08-13 (tối): độ bền của SMC-đã-tune — thắng toàn diện

Đã chạy SMC-tune qua đúng 3 bộ thí nghiệm áp cho ADP-FT (Phần I của
`sim_thesis_extra.m`). Kết quả: **ưu thế là bền vững, không phải cục bộ**.

| Khối lượng (Jc) | 10kg | 12kg | 14kg | 16kg |
|---|---|---|---|---|
| SMC tune | **79,9** | **148,6** | 267,0 | **530,0** |
| BS | 90,1 | 156,4 | **264,2** | 619,8 |
| ADP-FT+chiếu | 85,2 | 157,2 | 624,0 | 8109,5 |

| z_rms theo z0 | 0,245 | 0,612 | 1,225 | 2,375 | 3,500 |
|---|---|---|---|---|---|
| SMC tune | 1,07e-4 | 1,07e-4 | 1,07e-4 | 1,07e-4 | 1,07e-4 |
| ADP-FT+chiếu | 4,6e-2 | 1,84 | 1,3e-1 | 2,85 | 1,1e-3 |

Nhiễu độc lập 2 kênh (Jc): SMC tune 61,8/77,2/96,4/123,3 — thấp nhất mọi mức.

**Nghịch lý cần nhớ:** SMC không hề tuyên bố hội tụ cố định thời gian nhưng
z_rms của nó **không đổi tới 3 chữ số** qua cả 5 điều kiện đầu — tức độc lập
điều kiện đầu tốt hơn hẳn ADP-FT vốn thiết kế cho đúng mục tiêu đó.

**Lập luận bào chữa PHẢI BỎ:** "ADP-FT không cần tune tham số". Sai — nó có 10
tham số (λ, μ, α, β, ρ, Γ, κ₁, κ₂, W(0), uo_max) so với 2 của SMC.

**Lập luận CÒN lại (chỉ một):** khả năng thích nghi khi hàm chi phí Q, R thay
đổi — SMC phải quét lại từ đầu, ADP-FT nhận Q,R làm đầu vào. Nhưng đây mới là
lập luận về cấu trúc, CHƯA có số liệu. Đã đặt thành hướng số 1 ở §6.3.

Luận văn (bản 75 trang) đã phát biểu thẳng kết quả âm tính này.

Related: [[feedback-projection-operator]], [[feedback-fixedtime-unverified]]

Related: [[project-progress]], [[feedback-adp-ft-tuning]], [[feedback-advisor-nam]]
