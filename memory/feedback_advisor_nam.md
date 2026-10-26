---
name: feedback-advisor-nam
description: Yêu cầu của PGS.TS. Nguyễn Hoài Nam sau báo cáo SM1 (28/03/2026, chat) — nhiễu mô-men ở vòng trong, so sánh SMC/NTSMC, bắt buộc vòng kinetic, BC bậc 2
metadata:
  type: feedback
---

Nguồn: ảnh chụp đoạn chat Trung gửi ngày 2026-10-26 (tin nhắn ngày 28/03/2026).
Bản luận văn v2 và báo cáo tóm tắt (26/05/2026) được làm SAU đoạn chat này để đáp ứng
các yêu cầu dưới đây. Thầy chưa đọc hai bản đó.

Nguyên văn ý của thầy:
1. "Cho thêm disturbance dạng sin với tần số cao và biên độ 20% tín hiệu điều khiển lớn
   nhất". "Disturbance cộng vào tín hiệu đầu vào".
2. Trung hỏi "nhiễu cộng vào cả 2 kênh (v và ω) hay chỉ 1 kênh" → thầy trả lời:
   "Vòng trong. Mô men e nhé. Kinetic." Tức là nhiễu ở **mức mô-men của vòng trong**;
   thầy **không** yêu cầu cụ thể "cả 2 kênh v, ω".
3. "So sánh với trượt": "SMC cơ bản hoặc NTSMC".
4. "Mô phỏng phải có vòng điều khiển kinetic nữa" / "Thêm vòng trong nữa".
5. "Vòng ngoài bậc nhất e thiết kế BC như nào — BC phải hệ bậc 2" → backstepping phải
   gồm cả bước động lực học (vòng trong), không chỉ BC động học bậc 1.

**Why:** Thầy muốn kiến trúc điều khiển đúng (hai vòng, đầu ra mô-men) và phân tích
bền vững trước khi sang phần chính của luận văn.

**How to apply:**
- Mọi mô phỏng dùng kiến trúc hai vòng; nhiễu cộng vào mô-men τ (vòng trong).
  Cách hiện tại — `d = A·τ_max·sin(30t)·[1;1]` cộng vào mô-men hai bánh — đáp ứng đúng
  yêu cầu. Kịch bản nhiễu độc lập từng bánh (Phần F) là kiểm tra thêm, không bắt buộc.
- "20% tín hiệu điều khiển lớn nhất" đang hiểu là 20% τ_max (cách hiểu bảo thủ).
- SMC cơ bản và NTSMC đều đã có; BC bậc 2 = Kanayama (ngoài) + Fierro & Lewis (trong).

Related: [[feedback-disturbance-one-channel]], [[project-progress]]
