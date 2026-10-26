---
name: feedback-simulation-only
description: Luận văn chỉ thiết kế bộ điều khiển + chứng minh + kiểm chứng bằng mô phỏng; không có robot/motor thật — đừng đề nghị "chốt thông số thật" hay xử lý giới hạn an toàn phần cứng
metadata:
  type: feedback
---

Luận văn là **thiết kế bộ điều khiển, chứng minh và kiểm chứng bằng mô phỏng** — không có robot
hay motor thật. Tham số robot (m = 10 kg, I = 0,5 kg·m², r = 0,05 m, L = 0,15 m) và τ_max = 1 N·m
là tham số mô phỏng Trung đã chốt (26/10/2026); thiếu thực nghiệm đã ghi là hạn chế ở Mục 6.2.

**Why:** 27/10/2026 mình liệt kê "chốt τ_max thật của motor" và "có bão hòa số hạng chéo để vận tốc
thực không vượt v_max không" như việc Trung phải quyết; Trung trả lời: "Làm gì có thật, chúng ta đang
thiết kế bộ điều khiển và mô phỏng chứng minh mà". Ghi chú "τ_max tạm, chờ motor thật" trước đó là
hiểu nhầm của một phiên Claude.

**How to apply:** đừng đưa vào danh sách việc các quyết định kiểu phần cứng (thông số motor thật, an
toàn khi vượt giới hạn). Đặc tính mô phỏng đáng chú ý (vd. v thực vượt v_max do số hạng chéo) thì ghi
trong luận văn như một tính chất của thiết kế, không biến thành việc phải sửa.

Related: [[project-progress]], [[ket-qua-cuoi]]
