---
name: feedback-wang-gap
description: Khoảng hở trong chứng minh Theorem 1 của Wang et al. — bước (19)→(20) dùng g·g† = I₃; với luật gốc, W(0)=0 trên quỹ đạo thẳng, z = (0, z_y, 0) là điểm cân bằng (đã kiểm chứng)
metadata:
  type: project
---

Bổ đề 4.1 của luận văn (Mục 4.2.3):

- Với mọi `u_o`: `d/dt(½zᵀz) = v_r z_y sin z_θ − ζᵀu_o`, `ζ = −gᵀz = [z_x; z_θ]`. Điều khiển chỉ
  tác động qua z_x, z_θ; z_y chỉ có trong số hạng trôi.
- Wang viết `zᵀ g g† r` thành `zᵀ r` (coi g·g† = I₃), nhưng g ∈ R³ˣ² nên g·g† = I₃ − n·nᵀ,
  n = [0, 1, −z_x]/√(1+z_x²). r_y chỉ vào u_o qua tích z_x·r_y. Bất đẳng thức (25) không thể đúng
  tại (z, W̃) = ((0, z_y, 0), 0) khi z_y lớn.
- Với luật (16)-(18), cập nhật (17), Ŵ(0) = 0, ω_r = 0: mọi (0, z_y⁰, 0) là điểm cân bằng.
  Kiểm chứng Octave (động học thuần, đúng tham số Wang): z_y = 0.5 giữ nguyên 60 s,
  |z_x|, |z_θ| < 1e-11 (script `wang_kin_eq.m` trong scratchpad phiên 2026-10-26, không có trong repo).

**Why:** đây là cơ sở lý thuyết cho số hạng ghép nonholonomic ([[feedback-nonholonomic-coupling]]),
trước đó chỉ phát hiện qua mô phỏng. Hội tụ của z_y khi có số hạng ghép vẫn CHƯA được chứng minh
(cần V₁ kiểu Kanayama) — ghi là hạn chế, hướng phát triển.

**How to apply:** trình bày tôn trọng ("khoảng hở trong chứng minh", không phải "bài báo sai"),
không phủ nhận mô phỏng của bài báo (ở đó z_x ≠ 0 ban đầu, β_y = 1e5 ghép mạnh qua z_x·r_y).
Tham số gốc đúng: β = diag(1e5, 1e5, 1) ([[feedback-wang-citation-mismatch]]).
