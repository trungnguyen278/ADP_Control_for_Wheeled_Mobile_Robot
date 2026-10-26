---
name: feedback-adp-ft-tuning
description: Gain ADP-FT hiện dùng (2026-10-26) chọn bằng quy trình chỉnh gain đều nhau cho mọi PP (với bước 2 đầy đủ dùng chung); tham số gốc Wang không phát tán nhưng bám kém
metadata:
  type: feedback
---

Gain ADP-FT trong `simulations/ctrl_params.m` (từ 2026-10-26, mô hình B đúng):

- λ = μ = α = [0.04; 0.04; 0.025], β = [0.075; 0.075; 0.04], ρ = 0.1
- Γ = I, κ₁ = 0.04, κ₂ = 0.01, W(0) = [1.5; 1.5; 1.5; 0; 0; 0], u_o,max = 1.5
- ghép nonholonomic c_zy = 1.0, ς = 0.2; toán tử chiếu bật; luật cập nhật `'wang'` (eq.17 gốc)

Đây là bộ gain danh nghĩa cũ (λ μ α 0.08/0.05, β 0.15/0.08, c_zy 2, W0 3) nhân k* = 1/2,
chọn bằng `simulations/sweep_gains.m` — cùng quy trình cho cả 5 PP (SMC 1/8, BS 1/4, CL 1/4,
ADP-AC 1/4), tiêu chí tổng Jc 3 quỹ đạo ở nhiễu 20%, chạy lại sau khi chuyển sang bước 2
đầy đủ (K_c = M; K_c KHÔNG được tune theo PP chính — xem [[ket-qua-cuoi]]).

**Why:** thầy/hội đồng sẽ hỏi "so sánh có công bằng không". Trước 2026-10-26 gain ADP-FT
được chỉnh kỹ còn SMC dùng giá trị thông dụng → kết luận lệch. Nhận định cũ "gain gốc của
Wang làm phát tán trên dual-loop" là do lỗi ma trận B ([[feedback-b-matrix-error]]); với B
đúng và β gốc đúng (diag(1e5, 1e5, 1)), tham số gốc KHÔNG phát tán, chỉ bám kém
(Jc ×23.4/13.3/7.0 trên tròn/thẳng/số 8, sai số xác lập 0.5–1.7 cm).

**How to apply:** đổi gain của một PP thì phải chạy lại `sweep_gains.m` cho MỌI PP và cập
nhật Bảng 5.2 luận văn. Đánh đổi đã biết: k* = 1/2 tối ưu ở z0 chuẩn nhưng ở ‖z0‖ = 3.5
chậm hơn gain k = 1 (42.3 s vs 36.3 s) — nguyên nhân chính vẫn là luật học (đóng băng: 15.8 s).

Related: [[ket-qua-cuoi]], [[feedback-baseline-fairness]], [[feedback-advisor-nam]]
