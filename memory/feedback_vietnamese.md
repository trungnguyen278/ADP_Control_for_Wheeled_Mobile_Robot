---
name: feedback-vietnamese
description: Luận văn chỉ dùng tiếng Việt có dấu; thuật ngữ tiếng Anh giữ nguyên phải có trong DANH MỤC THUẬT NGỮ, không viết kèm ngoặc tiếng Anh; hình MATLAB không chứa chữ có dấu
metadata:
  type: feedback
---

Mọi báo cáo và luận văn đều phải viết bằng tiếng Việt có dấu. Riêng luận văn (Trung nhắc 27/10/2026):
**chỉ dùng tiếng Việt**; thuật ngữ chuyên môn tiếng Anh được giữ nguyên (backstepping, Critic, Actor,
Concurrent Learning, σ-modification, nonholonomic, gradient, affine, lemniscate...) nhưng phải có trong
bảng **DANH MỤC THUẬT NGỮ** (đặt sau danh mục từ viết tắt), giống cách làm với bảng viết tắt.

**Why:** Ban đầu tài liệu viết không dấu nên bị nhắc phải có dấu (T5 fontenc). Ngày 27/10 Trung nhắc
thêm: luận văn tiếng Việt thì chỉ dùng tiếng Việt, từ tiếng Anh nào giữ lại phải có bảng chú thích.

**How to apply:**
- LaTeX dùng T5 fontenc. Không viết kiểu "thuật ngữ Việt (English term)" — đối chiếu Việt–Anh đã nằm ở
  bảng thứ hai của DANH MỤC THUẬT NGỮ. Thêm từ tiếng Anh mới vào luận văn thì thêm luôn vào bảng.
- Cách dịch đã thống nhất: gain → hệ số; robust → bền vững; feedforward/feedback → truyền thẳng/phản
  hồi; vector → véc-tơ; body/world frame → hệ tọa độ thân robot/quán tính; transient → quá độ;
  chattering → rung; computed torque → mô-men tính toán; Lemma/Theorem/Remark → Bổ đề/Định lý/Nhận xét.
- Hình MATLAB: exportgraphics dạng vector chỉ hỗ trợ Latin-1, chữ như "Đ", "ư", "ẳ" bị thay bằng
  "#". Không đặt chữ tiếng Việt trong hình; dùng ký hiệu ($q_r$, $A$) hoặc (a)(b)(c) và ghi tên ở chú
  thích LaTeX.
- Thay theo từ bằng script thì phải che \label/\ref và tên nút TikZ (đã từng làm vỡ nút (online),
  (history) của hình CL).

- Đã bỏ trang ABSTRACT tiếng Anh (27/10, Trung chỉ ra nó vẫn còn). Nếu mẫu của trường yêu cầu
  Abstract thì khôi phục từ commit 53e6415.

Related: [[project-thesis]], [[feedback-advisor-nam]], [[feedback-defensible-scope]]
