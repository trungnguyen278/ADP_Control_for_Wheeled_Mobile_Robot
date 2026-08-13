---
name: feedback-nonholonomic-coupling
description: Nguyên nhân thật của mất bám là ADP-FT thiếu kênh tác động lên sai số ngang; thêm ghép c_zy*vr*tanh(zy/sat) khắc phục hoàn toàn
metadata:
  type: project
---

Phát hiện 2026-08-13 bằng chẩn đoán theo bước thời gian (không phải quét mù).
Đã thành §4.7.5 và §5.7–5.8 của luận văn.

## Cơ chế hỏng

Bốn số hạng robust tác động lên **từng thành phần z độc lập**. Khi sai số dồn vào
kênh ngang (zy lớn, zx và zθ ≈ 0) thì cả `uo_adp` lẫn `robust` đều ≈ 0 → hệ kẹt ở
**điểm cân bằng giả**, robot chạy song song quỹ đạo. Đo được: tại ‖z0‖=2,375, từ
giây 30 trở đi ‖z‖ = 2,85 nhưng ‖uo‖ = 0,085.

Đây là cascade nonholonomic (§2.5 luận văn đã mô tả): zy không có kênh điều khiển
trực tiếp, chỉ hội tụ gián tiếp qua `vr·sin(zθ)` — đòi hỏi zθ ≠ 0.
BS xử lý bằng `k2·vr·zy`; SMC bằng mặt trượt `zθ + c·zy`. **Wang không có.**

## Giải pháp

```matlab
robust(3) = robust(3) + s.ft_c_zy * vr * tanh(zy / s.ft_czy_sat);
```
với `c_zy = 2.0`, `czy_sat = 0.2`.

Hàng 2 của `g_pinv` là `[0, -zx/(zx²+1), -1/(zx²+1)]` nên số hạng này cho
`uo2 += c·vr·tanh(zy/sat)/(zx²+1)` — cùng dạng và dấu với `k2·vr·zy` của BS.
Dạng tanh tránh xung lớn; nhân `vr` nên tự tắt khi robot dừng.

## Kết quả

| | trước | sau |
|---|---|---|
| max z_rms qua 5 z0 | 2,85 (hỏng 2/5) | **8,2e-5** (ổn định 5/5) |
| z_rms circle | 4,63e-2 | **7,8e-5** — thấp nhất trong cả 6 phương pháp |
| t_settle max/min | — | **1,68** (BS 2,87) |
| Jc figure8 | 93,2 | **75,3** |
| Jc 14kg / 16kg | 624 / 8110 | **527 / 4238** |
| Jc circle | 85,2 | 89,6 (giá phải trả) |
| Jc 60% nhiễu | 157,7 | 196,5 (chattering) |

## Lưu ý khi tune

Quét 12 cấu hình (c × sat). Các cấu hình cho Jc thấp hơn (c=1,0–1,5 với sat lớn)
đều **mất bám** (z_rms = 2,85). sat lớn hơn (0,4–0,6) làm hỏng độ bền khối lượng
(14kg: 527 → 1076–1582). **Đừng tune lại theo Jc mà bỏ qua z_rms và mass.**

**Why:** Đây là đóng góp kỹ thuật mạnh nhất của luận văn — tìm ra thiếu sót cấu
trúc của phương pháp gốc, giải thích bằng lý thuyết nonholonomic sẵn có trong
luận văn, và khắc phục có bằng chứng.

**How to apply:** Khi bảo vệ, nhấn: ADP-FT giờ có **sai số bám nhỏ nhất trong tất
cả phương pháp**, ổn định mọi điều kiện đầu, t_settle ít phụ thuộc z0 hơn BS.
Nhưng KHÔNG nói đã đạt hội tụ cố định thời gian — tỉ số 1,68 khác 1 rõ ràng.

Related: [[feedback-projection-operator]], [[feedback-fixedtime-unverified]], [[feedback-baseline-fairness]]
