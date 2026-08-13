---
name: feedback-disturbance-one-channel
description: Nhiễu d=[1;1] đồng pha bị ma trận B triệt tiêu hoàn toàn ở kênh vận tốc góc — yêu cầu "nhiễu cả 2 kênh" của thầy Nam thực chất chưa đạt
metadata:
  type: project
---

Trong `sim_thesis.m:265` và `sim_sm1_full.m`, nhiễu được cộng vào mô-men hai bánh
với **cùng biên độ, cùng pha**:

```matlab
d_t = s.dist_amp * p.tau_max * sin(s.dist_freq*t) * [1; 1];
```

Nhưng plant nhận `B·(τ + d)` với `B = [r/2, r/2; r/(2L), −r/(2L)]`, nên:

```
B·[1; 1] = [r; 0] = [0.05; 0]      <-- kênh ω = 0 chính xác
B·[1;−1] = [0; 1/L] = [0; 0.3333]
```

**Nhiễu đồng pha không sinh ra một chút mô-men quay nào.** Toàn bộ "nhiễu 20/40/60%
τ_max" chỉ tác động lên kênh vận tốc dài v: lực 0.2 N, gia tốc 0.02 m/s².

Yêu cầu #2 của thầy Nam (2026-03-28) là "nhiễu cộng vào mô-men, **cả 2 kênh**".
PROGRESS.md đang đánh dấu "DA LAM" — đúng theo nghĩa cộng vào 2 bánh, sai theo
nghĩa kích thích 2 kênh động lực.

Kiểm chứng bằng cách chạy lại với `d = a*[1;-1]` (circle, 20%):

| Method | none | [1;1] (đang dùng) | [1;−1] |
|---|---|---|---|
| BS | 70.7 | 90.1 | 78.4 |
| ADP-FT | 70.7 | 85.2 | 73.4 |
| SMC | 475.0 | 555.5 | 434.4 |

**Why:** Đây là yêu cầu trực tiếp của GVHD, và toàn bộ Phần C (robustness nhiễu) —
chỗ ADP-FT thắng — đứng trên kịch bản nhiễu chỉ nửa vời. Nếu bị hỏi tại hội đồng
mà không chuẩn bị thì kết quả mạnh nhất của luận văn mất giá trị.

**How to apply:** Dùng nhiễu hai thành phần độc lập, khác pha/tần số, ví dụ
`d = a*[sin(w t); sin(w t + π/2)]` hoặc hai tần số khác nhau, rồi chạy lại Phần C.
Không sửa lén: nếu giữ nguyên kịch bản cũ thì phải nói rõ trong luận văn rằng nhiễu
chỉ tác động lên kênh tịnh tiến.

Related: [[feedback-advisor-nam]], [[feedback-baseline-fairness]], [[project-progress]]
