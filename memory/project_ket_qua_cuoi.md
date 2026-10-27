---
name: ket-qua-cuoi
description: Số liệu và kết luận CUỐI (2026-10-26 khuya, vòng 2 đầy đủ, MATLAB) — ADP-FT ngang NTSMC, sau SMC 7,9%; ghép nonholonomic quyết định; học critic không giúp; số hạng chéo vượt v_max
metadata:
  type: project
---

Chạy bằng MATLAB R2023a (trùng Octave tới 4 chữ số), T = 60 s, nhiễu đồng pha 20%, τ_max = 1 N·m
(tham số mô phỏng đã chốt), vòng 2 đầy đủ K_d = 20I, K_c = M, K_α = K_β = 10I. Gain (`sweep_gains.m`, kiểm lại bằng
MATLAB): SMC 1/8, ADP-FT 1/2, Kanayama 1/4, CL 1/4, ADP-AC 1/4 (vòng 2 cũ: ADP-AC 1/2).

| PP | Jc tròn / thẳng / số 8 | Tổng | t(‖z‖<0.01) tròn |
|---|---|---|---|
| SMC | 1.214 / 0.661 / 1.962 | 3.837 | 10.1 s |
| NTSMC | 1.237 / 0.838 / 2.055 | 4.130 | 9.1 s |
| ADP-FT | 1.318 / 0.790 / 2.033 | 4.140 | 6.0 s (nhanh nhất) |
| Kanayama | 1.277 / 0.877 / 2.136 | 4.290 | 9.9 s |
| CL | 1.675 / 1.504 / 2.920 | 6.099 | 31.1 s |
| ADP-AC | 3.760 / 5.139 / 5.051 | 13.951 | 37.5 s |

Kết luận chính (luận văn 73 trang sau khi rút gọn Ch.5 ngày 27/10; phân tích hàm mô tả và khảo sát
Q, R chỉ còn trong repo, không còn trong luận văn):

- ADP-FT ngang NTSMC (chênh 0,2% < độ phân giải lưới gain) → trình bày là "nhóm thứ hai", hạng 3
  có chú thích. Kém SMC 7,9%.
- Sàn nhiễu z_rms ≈ 4,1–4,3e-4. Công thức cũ (chỉ K_d) KHÔNG còn đúng: phải dùng hàm mô tả
  K_eq = K_d + 1,021·K_α·E^(p/q−1) + ¾K_β·E², E = 2d₀/(rm√(ω_n²+K_eq²)) → K_eq = 35,7, z_rms
  4,05e-4, khớp < 3% cả 3 dạng nhiễu. Tương quan τ_R/−d = K_eq/√(ω_n²+K_eq²) = 0,76 (cũ 0,55).
- Phần M (vòng 2): số hạng FT hạ z_rms 21–23%; t_e ≤ 0,17 s; K_c = M đổi Jc nhỏ (ADP-FT −1,2%).
  K_c = 10M: ADP-FT 3,833 < SMC nhưng Kanayama +12,8% → tương đương chỉnh gain vòng 1 không đều, giữ K_c = M.
- **Số hạng chéo đi vòng qua bộ kẹp η_d**: khi z_x lớn, v thực tới 0,57–0,62 m/s (> v_max 0,5);
  vòng 2 cũ ≤ 0,52. Hàng "cận dưới vật lý" ở bảng thời gian quá độ theo z₀ (Mục 5.5.4) vì thế chỉ còn là tham chiếu.
- Tham số gốc Wang (β = diag(1e5,1e5,1)): Jc ×23,4/13,3/7,0, Ju chiếm 58–78%, z_y dư 1,7–3,2 cm
  khi z_x, z_θ ≈ 0 — xác nhận Bổ đề 4.1(b) trên mô hình đầy đủ.
- Thiếu ghép nonholonomic: Jc ×3,2–5,6. Ở ‖z0‖ = 3,5 KHÔNG "kẹt quanh 1,4 m": |z_x|, |z_θ| nhỏ từ giây 20,
  còn |z_y| giảm đều 2,3 → 1,0 m từ giây 20 đến 60 (~3 cm/s); 1,4 m chỉ là trung bình 30–60 s, z_rms 5 s cuối
  = 1,06 (kiểm lại từ thesis_extra.mat ngày 27/10 khuya). Chiếu không kích hoạt (λmin ≥ 0,121).
- Kiểm thêm từ .mat (27/10 khuya): Ju thấp nhất là Kanayama trên cả 3 quỹ đạo; Ju/Jc < 9% trừ ADP-AC
  (17–24%); ≥ 97,2% Jc tích lũy trong 10 s đầu; ‖W(T)‖ = 0,49; SMC hội tụ cả 5 z0 (Phần I2).
- Critic: đóng băng tốt hơn học 2,6–2,8%; ở ‖z0‖ = 3,5: Wang 42,3 s, đóng băng 15,8 s, SMC 17,0 s,
  Kanayama 22,0 s, Bellman 19,3 s (nhưng 39,1 s ở 2,375 — không nhất quán). ADP-UUB giảm Jc 1,4–1,8%.
- Khối lượng 16 kg / nhiễu 60%: Jc đổi < 3% mọi PP.

**Why:** đây là bộ số chốt sau khi có vòng 2 đầy đủ; mọi số cũ hơn trong memory đều lỗi thời.

**How to apply:** khi viết slide/bài báo hay trả lời thầy, lấy số ở đây hoặc chạy lại
`sim_thesis.m` + `sim_thesis_extra.m`; script trích số dùng lần này (`extract_numbers.m`) nằm
trong scratchpad phiên 2026-10-26, không có trong repo. Đừng dùng số trong memory feedback-* cũ (⚠).

Related: [[feedback-b-matrix-error]], [[feedback-nonholonomic-coupling]], [[feedback-backstepping-framing]], [[project-progress]]
