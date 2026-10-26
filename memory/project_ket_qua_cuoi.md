---
name: ket-qua-cuoi
description: Số liệu và kết luận CUỐI (2026-10-26, mô hình B đúng, gain chỉnh đều nhau) — ADP-FT thứ 2 sau SMC; ghép nonholonomic quyết định; học critic không giúp, còn hại khi z0 lớn
metadata:
  type: project
---

Chạy bằng Octave 8.4 (cùng code MATLAB), T = 60 s, nhiễu đồng pha 20%, τ_max = 1 N·m (tạm).
Gain chọn bằng `sweep_gains.m` (một hệ số k cho toàn bộ gain mỗi PP): SMC 1/8, ADP-FT 1/2,
BS 1/4, CL 1/4, ADP-AC 1/2. Luận văn đã viết lại theo các số này (Ch.5–6, 72 trang).

| PP | Jc tròn / thẳng / số 8 | Tổng | t(‖z‖<0.01) tròn |
|---|---|---|---|
| SMC | 1.213 / 0.663 / 1.960 | 3.836 | 10.0 s |
| ADP-FT | 1.334 / 0.803 / 2.056 | 4.192 | 8.4 s (nhanh nhất) |
| NTSMC | 1.257 / 0.857 / 2.079 | 4.193 | 9.9 s |
| BS | 1.307 / 0.898 / 2.104 | 4.310 | 10.7 s |
| CL | 1.598 / 1.347 / 2.832 | 5.778 | 25.6 s |
| ADP-AC | 4.708 / 4.219 / 5.407 | 14.334 | 60.0 s |

Kết luận chính (đều có trong luận văn, báo cáo trung thực):
- z_rms ≈ 5.3e-4 = sàn nhiễu, dự đoán bằng z_rms = 2d₀/(√2·r·m·ω_n·√(ω_n²+K_d²)), khớp < 4%.
  Khối lượng 16 kg, nhiễu 60% (3 kiểu): Jc đổi < 4% → robustness thuộc về vòng trong.
- Ghép nonholonomic quyết định (thiếu → Jc ×3–5, kẹt |z_y| ≈ 2.4 m). Toán tử chiếu không
  bao giờ kích hoạt (λmin(P) > 0.12). Bỏ số hạng fixed-time (ADP-UUB) giảm Jc 1.5–2%.
- Critic: W(t) chỉ suy giảm theo σ-mod (‖W(60)‖ 0.494 vs 0.492 nếu bỏ số hạng học); đóng
  băng tốt hơn 3%; ở ‖z0‖ = 3.5 luật Wang làm chậm: t(0.05) 55.1 s vs đóng băng 19.0 s,
  Bellman 32.0 s, BS 23.4 s. W(0) = 0 vẫn hội tụ nhưng Jc ×2.4–3.6.
- Q không có trong luật (đổi Q → y hệt); đổi R hai chiều đều xấu. Tham số gốc Wang: không
  phát tán, Jc ×3.6/16.2/45.7, lệnh vận tốc chạm giới hạn 3% (tròn) / 18.7% (số 8).
- Fixed-time không bảo toàn được khi |v| ≤ 0.5 (cận dưới ≈ z_x0/0.2 s).

**Why:** đây là bộ số chốt sau khi sửa lỗi B; mọi số cũ hơn trong memory đều lỗi thời.

**How to apply:** khi viết slide/bài báo hay trả lời thầy, lấy số ở đây hoặc chạy lại
`sim_thesis.m` (Bảng A, A2, A3) và `sim_thesis_extra.m` (Phần D–L). Đừng dùng số trong
các memory feedback-* cũ (đánh dấu ⚠).

Related: [[feedback-b-matrix-error]], [[feedback-nonholonomic-coupling]], [[project-progress]]
