---
name: feedback-self-contained
description: Luận văn phải tự đứng được — Wang et al. chỉ là mẫu tham khảo; không kể lại bài báo ("phương trình (xx) của bài báo"), chứng minh viết đầy đủ tại chỗ từ lý thuyết cơ bản, chỉ trích dẫn [n] ở chỗ kế thừa
metadata:
  type: feedback
---

Luận văn không xoay quanh bài báo Wang et al. 2025. Wang chỉ là **mẫu tham khảo** cho luật vòng 1:
luật được trình bày như thiết kế của luận văn, ráp từ lý thuyết cơ bản (ADP, ổn định cố định thời gian,
backstepping, σ-modification), mọi chứng minh viết đầy đủ trong luận văn, và chỉ đặt trích dẫn [n] ở chỗ
thật sự kế thừa (quy tắc ranh giới ở CLAUDE.md §2 vẫn giữ).

**Why:** 27/10/2026 Trung: "bạn đang đề cập đến bài báo của Wang quá nhiều, điều này ảnh hưởng đến thầy
cô chấm (thường chỉ đọc các bài báo lớn và lý thuyết cơ bản)... chỉ lấy mẫu tham khảo là Wang thôi chứ vẫn
phải chứng minh đầy đủ các cách làm". Đếm lúc đó: 65 lần "Wang" + 32 lần "bài báo", dồn ở Chương 4.

**How to apply:**
- Không viết "phương trình (xx)/Định lý/Nhận xét/phụ lục của bài báo", "luật của Wang", "tham số gốc của
  Wang" — dùng số công thức của luận văn, "bộ tham số tham khảo [n]".
- Chứng minh không được dựa vào "bài báo đã chứng minh": 27/10 đã thêm Bổ đề 3.1 (cố định thời gian dạng
  thực dụng, có chứng minh) và Bổ đề 4.2 (chặn sai số trọng số). Cùng đêm đó hạ về mức thạc sĩ: Bổ đề 4.2
  chỉ còn phát biểu + ý tưởng chứng minh + trích Ioannou & Sun; phần cuối Định lý 4.1 (cần bất đẳng thức
  "Bổ đề 2 của Wang") đã bỏ ([[feedback-defensible-scope]]). Tự đứng được ≠ chứng minh mọi chi tiết:
  kết quả chuẩn thì trích dẫn, chỉ chứng minh đầy đủ phần của luận văn.
- Sau sửa: chữ "Wang" chỉ còn ở danh mục tài liệu; 17 trích dẫn [n] ở các chỗ kế thừa.
- Còn đúng một chỗ chưa có chứng minh: hội tụ của z_y ở vòng 1 khi có số hạng ghép (đang ghi là hạn chế,
  kiểm chứng bằng mô phỏng). Chứng minh đầy đủ đòi hỏi thiết kế lại vòng 1 theo hàm Lyapunov kiểu Kanayama.

Related: [[feedback-defensible-scope]], [[feedback-wang-gap]], [[feedback-verify-premises]], [[project-thesis]]
