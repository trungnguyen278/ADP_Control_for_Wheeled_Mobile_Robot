---
name: feedback-defensible-scope
description: Nội dung ở mức thạc sĩ mà Trung tự bảo vệ được — không viết như bài báo; kết quả phụ chỉ phát biểu + ý tưởng + trích dẫn; kết quả bất lợi để ở Hạn chế; lỗi trình bày nhỏ cứ để; Ch.5 không dựng công thức mới
metadata:
  type: feedback
---

Luận văn chỉ chứa những gì Trung tự giải thích và bảo vệ được trước hội đồng. Chương 5 (kết quả mô
phỏng) trình bày số liệu, điểm mạnh và điểm yếu, kèm 1–2 câu "vì sao" dẫn về công thức đã có ở
Chương 2–4 — không dựng lý thuyết mới trong chương kết quả.

**Why:** 27/10/2026 Trung thấy Chương 5 dài (26/67 trang thân bài) và hỏi "giải thích từ các công thức
thì có đi ngược không"; đồng ý rằng phải tự bảo vệ được ("lâu mới quay lại làm nên tôi cũng quên
nhiều thứ lắm"). Đã rút Chương 5 còn 15 trang (luận văn 86 → 73 trang), bỏ phân tích hàm mô tả
(K_eq, hàm Gamma) và mục Q, R — các kết quả đó vẫn đúng, còn ghi ở README và PROGRESS.md.

**How to apply:** trước khi thêm phân tích sâu (nhất là công thức mới) vào luận văn, tự hỏi: Trung có
giải thích được không, nó có phục vụ trực tiếp một đóng góp hay một điểm yếu cần báo cáo không. Khi
rút gọn, không cắt bằng chứng của các đóng góp và không giấu kết quả bất lợi. Khi giải thích cho
Trung, nói ngắn và nhắc lại bối cảnh vì Trung đã quên nhiều chi tiết.

**Mức thạc sĩ, không phải bài báo (Trung, 27/10 đêm):** "đây là luận văn thạc sĩ nên yêu cầu cũng
không quá cao đâu (bạn viết cao quá mà tôi không bảo vệ được cũng hỏng đấy)". Đánh giá từ góc người
chấm + người bảo vệ, Trung chọn hạ mức:

- Kết quả phụ, chuẩn (chặn sai số trọng số của σ-modification): phát biểu không hằng số cụ thể +
  "Ý tưởng chứng minh" vài dòng + trích dẫn — không chứng minh tách trường hợp ra hằng số kiểu 28κ₂‖W*‖⁴.
- Không giữ kết quả không dùng tới (đã bỏ phần cuối Định lý 4.1: hằng số a, b, γ, T_max — Bổ đề 4.1
  cho thấy nó không áp dụng cho ADP-FT).
- Không đưa khái niệm ngoài phạm vi vào thân bài (phép chiếu trực giao gg† = I − nnᵀ, bộ lọc lệnh,
  điều khiển mặt động); tên chúng chỉ để ở Hướng phát triển.
- Kết quả bất lợi cho chính phương pháp (Critic học không giúp, số hạng cố định thời gian ở vòng 1
  không giúp) giữ số liệu ở Mục 5.5 nhưng đặt ở Hạn chế với lời vừa phải ("chưa cải thiện"), không làm
  kết luận chính, không khuyến nghị "đóng băng Critic" — tránh câu hỏi "vậy ADP đóng góp gì?".
- Lỗi trình bày nhỏ (ngắt trang, chia đoạn) cứ để: "thầy cũng có thể tìm lỗi khi chấm chứ không có
  thì thầy sẽ tìm kỹ hơn". Đừng tốn lượt build để đánh bóng.
- Gom sửa nhỏ vào cùng đợt với sửa lớn để build một lần.

**Chốt phạm vi (Trung, 27/10 đêm):** "cơ bản tôi không muốn thêm nhiều để dễ học lại còn bảo vệ thôi".
Từ đây luận văn coi như chốt nội dung: không thêm phần mới, chỉ sửa lỗi khi Trung chỉ ra. Không đề
xuất lại việc thiết kế vòng 1 theo hàm Lyapunov kiểu Kanayama để chứng minh hội tụ z_y (đã để ở Hướng
phát triển). Việc hữu ích tiếp theo là giúp Trung ôn để bảo vệ, không phải viết thêm.

Thí nghiệm cũng theo tinh thần luận văn nghiên cứu (Trung, 27/10): không khảo sát riêng bất định
khối lượng ("liên quan thực tế"); sai lệch tham số tương đương nhiễu cộng vào mô-men
(−B⁻¹ΔMη̇, nhiễu khớp) nên gộp vào nhiễu đầu vào — ghi ở (A2) Mục 4.4.

Related: [[feedback-verify-premises]], [[user-profile]], [[project-progress]]
