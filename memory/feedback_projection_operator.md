---
name: feedback-projection-operator
description: Toán tử chiếu giữ V̂ xác định dương — giảm 6,9 lần chi phí ở sai lệch khối lượng 40%, nhưng không cứu được vấn đề miền hút
metadata:
  type: project
---

Đề xuất và kiểm chứng ngày 2026-08-13, đã thành §4.7.4 và §5.8 của luận văn.
Bật bằng `s.ft_proj = true` (mặc định từ commit `8551f3c`).

## Ý tưởng

`V̂ = Wᵀφ(z) = zᵀP(W)z` với

```
P(W) = [ W1     W4/2   W6/2
         W4/2   W2     W5/2
         W6/2   W5/2   W3   ]
```

Ràng buộc `P(W) ⪰ eps_p·I` là tập lồi → chiếu được. Kiểm tra bằng Sylvester (rẻ),
chỉ khi vi phạm mới eig + kẹp trị riêng. Thêm chặn `‖W‖ ≤ W_max`.
Nguồn: Ioannou & Sun (1996) §4.4; Kamalapurkar et al. (2018) ch.4.

## Kết quả — được và không được

| | không chiếu | có chiếu |
|---|---|---|
| Jc 3 quỹ đạo (z0 chuẩn) | 85,2 / 8,5 / 93,2 | **không đổi một chữ số** |
| Jc 14 kg | 4283 | **624,0** (6,9 lần) |
| z_rms 14 kg | 3,03 | **0,172** |
| z0=0,612 | 3,79 | 1,84 (vẫn hỏng) |
| z0=2,375 | 2,31 | 2,85 (xấu đi) |
| z0=3,500 | 3,75 | **1,1e-3** (cứu được) |

Số z0 ổn định vẫn 3/5. **Giả thuyết "W mất xác định dương là nguyên nhân mất bám"
chỉ đúng MỘT PHẦN.**

Phần còn lại quy cho **năng lực biểu diễn của cơ sở bậc hai cố định**: mỗi z0 cho
quỹ đạo thăm một vùng khác nhau, W hội tụ về xấp xỉ tốt cục bộ cho vùng đó. Đây là
hạn chế bản chất của xấp xỉ tuyến tính theo tham số — phải đổi cấu trúc xấp xỉ
(RBF / deep NN), không sửa được bằng ràng buộc trên W.

**Why:** Đây là đóng góp kỹ thuật thật sự của Trung (không có trong Wang), có số liệu
chứng minh, và quan trọng là *đi kèm tuyên bố đúng về giới hạn của nó*.

**How to apply:** Khi trình bày, luôn nói cả hai vế — cứu được khối lượng 40%,
KHÔNG cứu được miền hút. Đừng để nó thành "đã giải quyết xong vấn đề".

Related: [[feedback-fixedtime-unverified]], [[feedback-ablation-results]]
