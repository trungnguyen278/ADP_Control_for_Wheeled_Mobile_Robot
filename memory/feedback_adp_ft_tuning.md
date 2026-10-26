---
name: feedback-adp-ft-tuning
description: Gain ADP-FT hiện dùng (2026-10-26) chọn bằng quy trình chỉnh gain đều nhau cho mọi PP; tham số gốc Wang không phát tán nhưng bám kém trên dual-loop
metadata:
  type: feedback
---

Gain ADP-FT trong `simulations/ctrl_params.m` (từ 2026-10-26, mô hình B đúng):

- λ = μ = α = [0.04; 0.04; 0.025], β = [0.075; 0.075; 0.04], ρ = 0.1
- Γ = I, κ₁ = 0.04, κ₂ = 0.01, W(0) = [1.5; 1.5; 1.5; 0; 0; 0], u_o,max = 1.5
- ghép nonholonomic c_zy = 1.0, ς = 0.2; toán tử chiếu bật; luật cập nhật `'wang'` (eq.17 gốc)

Đây là bộ gain danh nghĩa cũ (λ μ α 0.08/0.05, β 0.15/0.08, c_zy 2, W0 3) nhân k* = 1/2,
chọn bằng `simulations/sweep_gains.m` — cùng quy trình cho cả 5 PP (SMC 1/8, BS 1/4, CL 1/4,
ADP-AC 1/2), tiêu chí tổng Jc 3 quỹ đạo ở nhiễu 20%.

**Why:** thầy/hội đồng sẽ hỏi "so sánh có công bằng không". Trước 2026-10-26 gain ADP-FT
được chỉnh kỹ còn SMC dùng giá trị thông dụng → kết luận lệch. Nhận định cũ "gain gốc của
Wang làm phát tán trên dual-loop" là do lỗi ma trận B ([[feedback-b-matrix-error]]); với B
đúng, tham số gốc KHÔNG phát tán, chỉ bám kém (Jc ×3.6/16.2/45.7), chủ yếu vì thiếu kênh
ngang, và trên hình số 8 lệnh vận tốc chạm giới hạn 18.7% thời gian.

**How to apply:** đổi gain của một PP thì phải chạy lại `sweep_gains.m` cho MỌI PP và cập
nhật Bảng 5.2 luận văn. Đánh đổi đã biết: k* = 1/2 tối ưu ở z0 chuẩn nhưng ở ‖z0‖ = 3.5
chậm hơn gain k = 1 (55 s vs 44 s) — nguyên nhân chính vẫn là luật học (đóng băng: 19 s).

Related: [[ket-qua-cuoi]], [[feedback-baseline-fairness]], [[feedback-advisor-nam]]
