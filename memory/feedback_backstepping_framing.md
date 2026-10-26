---
name: feedback-backstepping-framing
description: Từ 27/10 gọi là "vòng 1 / vòng 2" (không gọi "bước"); bộ điều khiển hai vòng thiết kế theo backstepping, chứng minh ổn định theo từng vòng rồi toàn hệ; K_c = M
metadata:
  type: feedback
---

Từ 2026-10-27, luận văn và báo cáo tóm tắt gọi là **vòng 1 / vòng 2** (Trung chọn), không gọi
"bước 1 / bước 2" nữa:

- **Vòng 1 — vòng ngoài (động học, hệ 3 phương trình):** vận tốc ảo `η_d = u_f + u_o(z, W)` —
  ADP-FT của Wang (hoặc luật so sánh).
- **Vòng 2 — vòng trong (động lực học, hệ 2 phương trình):**
  `B·τ = M(η̇_d + K_d e + K_α sig(e)^{p/q} + K_β e³) + F(η) + K_c·[z_x; z_θ]`, `e = η_d − η`.
- Hai vòng thiết kế theo **thủ tục backstepping**: vòng 2 suy ra từ hàm Lyapunov của vòng 1 (số hạng
  chéo `K_c·ζ = −K_c gᵀz` triệt `ζᵀe`). "Mỗi bước của backstepping thiết kế một vòng."
- Chứng minh trình bày **theo từng vòng rồi toàn hệ** (Mục 4.4): vòng 1 (Mục 4.2.3, Bổ đề 4.1),
  vòng 2 + toàn hệ (Định lý 4.1, `V = V₁ + V₂`, `V₂ = ½eᵀK_c⁻¹Me`), thời gian hội tụ vòng 2 (Hệ quả 4.1).
  Báo cáo tóm tắt có bản chứng minh rút gọn (Mục 2.4).

**Why:** Trung hỏi "điều khiển BC cần 2 vòng nghĩa là hai hệ phương trình đúng chứ" và "cần chứng minh,
ghi vòng 1 2 chứ sao ghi bước" — thầy Nam yêu cầu "mô phỏng 2 vòng" + "BC bậc 2", nên cách gọi phải
khớp với yêu cầu đó. Trước đó (26/10) một phiên Claude đã đổi sang "bước" vì Trung thấy khó hiểu khi mỗi
vòng là một bộ điều khiển riêng — điểm đó vẫn giữ: hai vòng KHÔNG thiết kế độc lập.

**How to apply:**

- Viết: "bộ điều khiển hai vòng thiết kế theo backstepping"; 5 phương pháp chỉ khác nhau ở vòng 1, vòng 2
  dùng chung. Chữ "bước" chỉ dùng cho thủ tục backstepping nói chung, bước tích phân, bước trong chứng minh.
- Đừng quay về "dual-loop độc lập + ISS phác thảo / tách thang thời gian" — Định lý 4.1 đã thay.
- Đừng tăng `K_c` để có lợi cho ADP-FT: K_c lớn tương đương chỉnh gain vòng 1 không đều
  (vi phạm [[feedback-baseline-fairness]]).
- Script đổi tên có loại trừ (giữ "phép chiếu qua hai bước"): scratchpad phiên 27/10, không có trong repo.

Related: [[feedback-advisor-nam]], [[feedback-wang-gap]], [[project-thesis]]
