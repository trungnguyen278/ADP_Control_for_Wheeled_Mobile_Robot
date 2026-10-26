---
name: feedback-backstepping-framing
description: Trung muốn trình bày MỘT bộ backstepping hai bước (không phải hai bộ điều khiển "vòng ngoài/vòng trong" riêng), chứng minh lại thay vì chỉ trích, được phép sửa phần điều khiển của bài báo
metadata:
  type: feedback
---

Từ 2026-10-26 (tối), bộ điều khiển trong luận văn là **một bộ backstepping hai bước**:

- Bước 1 (động học): vận tốc ảo `η_d = u_f + u_o(z, W)` — ADP-FT của Wang (hoặc luật so sánh).
- Bước 2 (động lực học): `B·τ = M(η̇_d + K_d e + K_α sig(e)^{p/q} + K_β e³) + F(η) + K_c·[z_x; z_θ]`,
  `e = η_d − η`. Số hạng chéo `K_c·ζ = −K_c gᵀz` triệt `ζᵀe` trong `V̇₁` → hàm Lyapunov chung
  `V = V₁ + ½eᵀK_c⁻¹Me` (Định lý 4.1). Chọn `K_c = M` → `V = V₁ + ½eᵀe`.
- "Vòng ngoài/vòng trong" chỉ còn là cách gọi vị trí tín hiệu trên sơ đồ.

**Why:** Trung đọc báo cáo thấy khó hiểu khi mỗi vòng một bộ điều khiển riêng và danh sách 5
phương pháp ở "vòng ngoài" quá dài; ý của thầy Nam về "BC bậc 2" là backstepping đúng nghĩa
cho hệ bậc hai. Trung nói: "tôi tham khảo bài báo nhưng vẫn chứng minh lại", "bài báo chỉ là tham
khảo, có thể thay đổi phần điều khiển như thêm 1 vòng để BC đúng chuẩn" — nhưng vẫn trích dẫn đầy đủ.

**How to apply:**
- Viết/giải thích: một bộ điều khiển, hai bước; 5 phương pháp chỉ khác nhau ở bước 1, bước 2 dùng chung.
- Khi dựng lại chứng minh của bài báo: trích dẫn tại chỗ phần của Wang; phần mở rộng/sửa mới là
  của luận văn (xem bảng phân định Mục 4.6 luận văn).
- Đừng đổi lại thành "dual-loop/ISS phác thảo" — Định lý 4.1 đã thay lập luận tách thang thời gian.
- Đừng tăng `K_c` để có lợi cho ADP-FT: K_c lớn tương đương chỉnh gain bước 1 không đều
  (vi phạm [[feedback-baseline-fairness]]).

Related: [[feedback-advisor-nam]], [[feedback-wang-gap]], [[project-thesis]]
