---
name: ket-qua-cuoi
description: Số liệu và kết luận CUỐI (2026-10-26 khuya, bộ backstepping hai bước với bước 2 đầy đủ K_c = M, gain chỉnh đều nhau) — ADP-FT thứ 2/5 sau SMC, ngang NTSMC; ghép nonholonomic quyết định; học critic không giúp, còn hại khi z0 lớn
metadata:
  type: project
---

Chạy bằng Octave 8.4 (cùng code MATLAB), T = 60 s, nhiễu đồng pha 20%, τ_max = 1 N·m (tạm).
Bước 2 đầy đủ dùng chung cho mọi PP: K_d = 20I, K_c = M = diag(10, 0.5), K_α = K_β = 10I,
p/q = 17/19 ([[feedback-backstepping-framing]]). Gain bước 1 chọn bằng `sweep_gains.m`
(một hệ số k cho toàn bộ gain mỗi PP): SMC 1/8, ADP-FT 1/2, BS 1/4, CL 1/4, ADP-AC 1/4.
Luận văn đã viết lại theo các số này (Ch.4–6, 82 trang); bảng tổng hợp ở README.md.

| PP | Jc tròn / thẳng / số 8 | Tổng | t(‖z‖<0.01) tròn |
|---|---|---|---|
| SMC | 1.214 / 0.661 / 1.962 | 3.837 | 10.1 s |
| NTSMC | 1.237 / 0.838 / 2.055 | 4.130 | 9.1 s |
| ADP-FT | 1.318 / 0.790 / 2.033 | 4.140 | 6.0 s (nhanh nhất) |
| BS | 1.277 / 0.877 / 2.136 | 4.290 | 9.9 s |
| CL | 1.675 / 1.504 / 2.920 | 6.099 | 31.1 s |
| ADP-AC | 3.760 / 5.139 / 5.051 | 13.951 | 37.5 s |

Kết luận chính (đều có trong luận văn, báo cáo trung thực):
- ADP-FT thứ hai trong năm PP (NTSMC là biến thể SMC), cao hơn SMC 7,9%, ngang NTSMC.
- Bước 2 đầy đủ so với luật Fierro & Lewis (Phần M, Mục 5.6): Jc −1.2% (ADP-FT), −0.5% (BS),
  +0.01% (SMC); z_rms −22% (5.36e-4 → 4.20e-4); z0 lớn: Jc −12…−35%. K_c = 10M cho ADP-FT
  3.833 ≈ SMC 3.844 nhưng BS +13% → giữ K_c = M (chọn từ cấu trúc Lyapunov, không tune theo PP chính).
- z_rms ≈ 4.1–4.3e-4 m = sàn nhiễu; dự đoán z_rms ≈ 2d₀/(√2·r·m·ω_n·√(ω_n²+K_eff²)) với
  K_eff = K_d + K_α·N(δv) (hàm mô tả), khớp < 7%. Khối lượng 16 kg, nhiễu 60% (3 kiểu): Jc đổi
  < 3% → độ bền thuộc về bước 2.
- Ghép nonholonomic quyết định (thiếu → Jc ×3.2–5.6, kẹt ở điểm cân bằng giả). Toán tử chiếu
  không bao giờ kích hoạt (λmin(P) > 0.12). Bỏ số hạng fixed-time bước 1 (ADP-UUB) giảm Jc 1.4–1.8%.
- Critic: W(t) chỉ suy giảm theo σ-mod (‖W(60)‖ 0.494 vs 0.492 nếu bỏ số hạng học); đóng băng
  tốt hơn 2.6–2.8%; ở ‖z0‖ = 3.5 luật Wang làm chậm: t(0.05) 42.3 s vs đóng băng 15.8 s,
  Bellman 19.3 s, gain k = 1 36.3 s, BS 22.0 s. W(0) = 0 vẫn hội tụ nhưng Jc ×2.3–3.2.
- Q không có trong luật (đổi Q → y hệt); đổi R hai chiều đều xấu. Tham số gốc Wang
  (β = diag(1e5, 1e5, 1)): không phát tán, Jc ×23.4/13.3/7.0 (tròn/thẳng/số 8), sai số xác lập
  0.5–1.7 cm. Luật gốc có điểm cân bằng giả (0, z_y, 0) trên quỹ đạo thẳng khi Ŵ(0) = 0 →
  [[feedback-wang-gap]].
- Fixed-time không bảo toàn được khi |v| ≤ 0.5 (thời gian quá độ buộc tăng theo ‖z0‖).

**Why:** đây là bộ số chốt sau khi chuyển sang bước 2 đầy đủ; mọi số cũ hơn (kể cả bộ
"3.836 / 4.192 / 4.310" với bước 2 Fierro & Lewis) đều lỗi thời.

**How to apply:** khi viết slide/bài báo hay trả lời thầy, lấy số ở đây hoặc chạy lại
`sim_thesis.m` (Bảng A, A2, A3) và `sim_thesis_extra.m` (Phần D–M). Đừng dùng số trong
các memory feedback-* cũ (đánh dấu ⚠).

Related: [[feedback-b-matrix-error]], [[feedback-nonholonomic-coupling]], [[project-progress]]
