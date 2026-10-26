---
name: feedback-b-matrix-error
description: Ma trận B của mô hình động lực học sai vật lý (dùng Jacobian động học) — cơ cấu chấp hành yếu 800 lần, phần lớn kết luận Ch.5–6 là sản phẩm phụ của lỗi này
metadata:
  type: project
---

Phát hiện 2026-10-26 khi kiểm chứng lại kết quả bằng GNU Octave 8.4. Trước khi sửa gì,
đã tái tạo 12/12 số chính của luận văn (BS 90,08; ADP-FT 89,56, z_rms 7,82e-5; SMC tune 79,87;
14 kg: 527,21...) — sai khác < 0,1%. Số liệu không bị chép sai; vấn đề nằm ở mô hình.

## Lỗi

`models/wmr_params.m:50` và `docs/thesis/thesis.tex:465` dùng
`B = [r/2, r/2; r/(2L), -r/(2L)]` — đây là Jacobian động học (vận tốc bánh → v, ω),
không phải ma trận mô-men. Đúng theo Fierro & Lewis (1997), suy từ công ảo (B = J^-T):

```
B = (1/r) * [1, 1; L, -L]      % F = (τR+τL)/r,  N = L(τR−τL)/r
```

Thử thứ nguyên: `thesis.tex:467` ghi `F = (r/2)(τR+τL)` → N·m², không phải N.

Hệ quả: kênh v yếu 800 lần (r²/2), kênh ω yếu 18 lần. Với τ_max = 20 N·m robot 10 kg
chỉ có lực tối đa 1 N → gia tốc ≤ 0,1 m/s². Việc đổi τ_max 5→20 và fc 0,3→0,05
(PROGRESS, mục Ghi chú kỹ thuật) là vá triệu chứng của chính lỗi này.

## Hậu quả đo được (circle, nhiễu 20%, T = 60 s, mô hình hiện tại)

- 94–98% J_c rơi vào 10 s đầu; lệnh mô-men bão hòa ~68% thời gian trong 10 s đầu;
  mọi phương pháp đều cần 3,5 s mới đạt 95% v_r → J_c đo "ai chịu bão hòa tốt hơn".
- Nhiễu 20% chỉ gây dao động vị trí ~2e-5 m (hở mạch) — gần như không có tác dụng.

## Chạy lại với B đúng, τ_max = 1 N·m (bão hòa < 2%)

| Kết luận luận văn (B sai) | B đúng |
|---|---|
| ADP-FT sai số bám nhỏ nhất (7,8e-5) | ADP-FT 5,32e-4 ≈ SMC tune 5,37e-4 ≈ BS 5,57e-4 |
| ADP-FT yếu với khối lượng (14 kg: 527 vs BS 264) | 14 và 16 kg: ADP-FT 1,41 = BS 1,41; SMC tune 1,21 |
| ADP-FT kém BS 25% ở nhiễu 60% | 1,41 vs 1,40 |
| ADP-FT yếu trên hình số 8 (75,3 vs BS 12,1) | ADP-FT 2,11 < BS 2,76; SMC tune 1,96 |
| Tham số gốc Wang làm phát tán (J_c 4746) | Không phát tán (circle 4,86), nhưng kém (hình 8: 94,0) |
| J_c circle ~80–90 | J_c circle: SMC tune 1,19 / BS 1,39 / ADP-FT 1,40 / CL 7,71 |

Với τ_max = 0,5 và 20 bức tranh định tính không đổi (BS: 1,45 và 1,46).

## Chạy lại đầy đủ bằng script (2026-10-26, sau khi sửa)

Số đầy đủ ở PROGRESS.md mục 2026-10-26. Ba phát hiện cần nhớ:
- **Ablation (Phần D, G, J):** chỉ ghép nonholonomic tạo khác biệt (Jc tròn 8,09 → 1,40).
  Critic đóng băng tốt hơn 6%, bỏ số hạng fixed-time tốt hơn 5–6%, hai luật cập nhật như
  nhau (‖W(T)‖ = 0,586 cả hai — σ-mod chiếm ưu thế, W chỉ suy giảm), toán tử chiếu không
  tác dụng. ⇒ Hiệu năng "ADP-FT" đến từ số hạng robust + ghép, không đến từ ADP.
- **Độ bền (Phần B, C, F, I):** khối lượng 10–16 kg và nhiễu 0–60% không còn phân biệt
  các phương pháp (Jc đổi < 2%) — vòng trong hấp thụ gần hết.
- **Tham số gốc Wang:** không phát tán nhưng bám kém (sai số 0,35–0,71 m ở 4/5 z0).
- **Đổi Q, R (Phần K) — âm tính:** Q ×0,1–×10 làm Je, Ju đổi < 3%; R ×0,1 hoặc ×10 làm
  xấu cả Je lẫn Ju. Lập luận cuối cùng cho ADP ("tự thích nghi khi đổi hàm chi phí",
  Mục 6.3–6.4 luận văn) không đứng được.
- SM1, SM2 chỉ dùng mô hình động học → không bị lỗi B.

## Cái vẫn đứng vững khi B đúng

- **Thiếu kênh zy của luật Wang** ([[feedback-nonholonomic-coupling]]): cấu hình gốc (không
  ghép, không chiếu) vẫn mất bám ở z0 #3 và #5 (z_rms 2,68 và 2,12); có ghép → 5/5,
  hình 8: 10,04 → 2,11. Đây là đóng góp thật.
- ADP-FT hội tụ nhanh hơn BS ở cả 5 z0: ts(0,05) = 3,9 / 5,1 / 6,1 / 11,7 / 19,1 s
  so với BS 8,5 / 13,2 / 14,4 / 16,0 / 27,7 s.
- SMC tune có J_c thấp nhất ([[feedback-baseline-fairness]]), nhưng khoảng cách nhỏ (~15%).

## Hai lỗi khái niệm (không phụ thuộc B)

1. **Tiêu chí "t_settle max/min ≈ 1" sai.** Fixed-time (Polyakov) chỉ đòi T(z0) ≤ T_max,
   không đòi T(z0) hằng số. Hệ fixed-time lý tưởng ẋ = −|x|^0,5·sgn(x) − |x|^1,5·sgn(x)
   (T_max = π) cho tỉ số 2,35 (về 0) hoặc 3,59 (tol 0,05) trên cùng dải ‖z0‖ = 0,245–3,5
   — tệ hơn cả BS (2,87). Ngoài ra, trong bảng `tab:z0_tsettle` ADP-FT chậm hơn BS ở 4/5 z0;
   tỉ số thấp chỉ vì chậm đều. Có bão hòa v_max thì fixed-time toàn cục là bất khả về vật lý.
2. **Critic học gần như không đóng góp.** Đóng băng W = W0 (Γ = 0), với B đúng:
   circle 1,31 vs 1,40; hình 8 1,98 vs 2,11; thẳng 0,73 vs 0,78; quét z0 khớp ±7%.
   (Với B sai, học có giúp trên thẳng/hình 8: 24,5→10,8 và 105,9→75,3.)

**Why:** Phát hiện khi Trung hỏi "kết quả ổn chưa, phải ổn đúng thì mới đi tiếp". Lỗi B có
từ mô hình gốc (02/2026), nên cả bản v2 đã gửi thầy Nam cũng mang lỗi này.

**How to apply:**
- **Đã sửa trong code 2026-10-26** (Trung: "sửa trước, phần viết note lại làm sau"):
  B đúng + τ_max tạm 1 N·m (`9b3fcf0`), quét lại SMC giữ 0,7/0,05 (`d9a9532`),
  Phần J ablation cơ chế + đóng băng critic + "Wang gốc" (`5babecd`, commit sau đó).
  Số mới: PROGRESS.md mục 2026-10-26.
- **Luận văn CHƯA viết lại.** Checklist đầy đủ: PROGRESS.md → VIEC CAN LAM mục 2.
  Không sửa chữ trước khi Trung chốt τ_max và chạy lại trên MATLAB (số sẽ đổi).
- Còn chờ Trung quyết: τ_max thật của motor; có tune lại gain ADP-FT/BS trên mô hình
  mới không (gain hiện tại chọn trên mô hình sai).
- Hình trong `docs/thesis/figures/` vẫn của mô hình cũ — phải xuất lại bằng MATLAB.

Related: [[feedback-nonholonomic-coupling]], [[feedback-baseline-fairness]], [[feedback-fixedtime-unverified]], [[feedback-adp-ft-tuning]], [[project-progress]]
