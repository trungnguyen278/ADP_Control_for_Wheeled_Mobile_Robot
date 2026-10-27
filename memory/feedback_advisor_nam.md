---
name: feedback-advisor-nam
description: Yêu cầu của PGS.TS. Nguyễn Hoài Nam (chat 28/03/2026) — đọc theo KHÁI NIỆM CƠ BẢN (sách giáo khoa), không theo cách bài báo dùng; nhiễu mô-men vòng trong, SMC/NTSMC, vòng kinetic, BC bậc 2
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

**Nguyên tắc đọc (Trung nhắc 27/10/2026):** các yêu cầu của thầy xoay quanh **khái niệm cơ
bản** (định nghĩa sách giáo khoa), không phải khái niệm mở rộng hay cách gọi lỏng trong bài
báo. Ví dụ: "BC" mà Wang et al. so sánh (eq. 31) là luật Kanayama một vòng, nhưng BC đúng nghĩa
phải có hai bước/hai vòng cho hệ bậc 2.

**Why:** Thầy muốn kiến trúc điều khiển đúng (hai vòng, đầu ra mô-men) và phân tích
bền vững trước khi sang phần chính của luận văn.

**How to apply:**
- Kiểm tra mỗi yêu cầu bằng định nghĩa gốc của khái niệm (BC: mỗi bước có hàm Lyapunov;
  SMC cho hệ bậc n: s = (d/dt + λ)^{n−1}e, chuyển mạch lấn át nhiễu khớp; NTSMC: định nghĩa
  cho hệ bậc 2, Feng 2002), không chấp nhận "gọi tên giống" là đạt.
- Nhiễu `d = 0,2·τ_max·sin(30t)·[1;1]` cộng vào mô-men: đạt (mọi PP chạm τ_max = 1 N·m trong
  giây đầu, nên 20% τ_max đúng bằng 20% max|τ|).
- Vòng 2 hiện tại = luật mô-men có số hạng chéo + FT (không còn là F&L thuần), xem
  [[feedback-backstepping-framing]].

**Review 27/10/2026 theo khái niệm cơ bản** (chờ Trung quyết hướng sửa):
- Đạt: nhiễu (vị trí, dạng, biên độ), vòng kinetic, thủ tục BC hai bước (số hạng chéo, V chung).
- Hở: bước 1 của BC với ADP-FT không có hàm Lyapunov chặt (Bổ đề 4.1, z_y chỉ kiểm chứng mô
  phỏng) — sửa thật phải đổi cấu trúc g†r(z) của Wang.
- Chưa đạt: SMC, NTSMC đang thiết kế cho hệ động học bậc 1 ở vòng 1 (luận văn tự ghi NTSMC gốc
  là cho hệ bậc 2); nhiễu mô-men không tới khâu chuyển mạch, Mục 5.4 tự kết luận thí nghiệm nhiễu
  không phân biệt được luật vòng 1. Cách sửa đề xuất: SMC (và/hoặc NTSMC) bậc 2 ở tầng mô-men,
  mặt trượt s = η_d(z) − η, chuyển mạch lấn át biên nhiễu.
- Nhãn "BS" cho luật Kanayama là đúng kiểu gọi lỏng thầy đã bắt → đề xuất đổi "Kanayama".

Related: [[feedback-disturbance-one-channel]], [[feedback-backstepping-framing]], [[project-progress]]
