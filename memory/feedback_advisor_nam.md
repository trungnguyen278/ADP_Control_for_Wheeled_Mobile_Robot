---
name: feedback-advisor-nam
description: Yêu cầu của PGS.TS. Nguyễn Hoài Nam (chat 28/03/2026, phản hồi báo cáo SM1) — nhiễu mô-men vòng trong, SMC/NTSMC, vòng kinetic, BC bậc 2; cách đọc đã kiểm chứng: 1 yêu cầu sửa khái niệm, 4 yêu cầu bổ sung thành phần chuẩn
metadata:
  type: feedback
---

Nguồn: ảnh chụp đoạn chat Trung gửi ngày 2026-10-26 (tin nhắn ngày 28/03/2026), là phản hồi cho
báo cáo SM1 cùng ngày. Bản luận văn v2 và báo cáo tóm tắt (26/05/2026) được làm SAU đoạn chat này
để đáp ứng các yêu cầu dưới đây. Thầy chưa đọc hai bản đó.

**v2 chưa đáp ứng ý 5** (kiểm chứng 29/10 trên commit `7fedea9`): v2 vẫn có mục "4.3 Vòng ngoài 1:
Backstepping (BS)" (luật Kanayama trên vòng ngoài bậc nhất — đúng chỗ thầy hỏi) và "4.2 Vòng trong: Dynamic
Backstepping" = luật mô-men tính toán thiết kế riêng, chứng minh chỉ với V₂ = ½eᵀMe, ổn định toàn hệ dựa vào
tách thang thời gian (K_d = 20). Bản hiện tại mới đáp ứng (vòng 2 suy từ V₁, Định lý 4.1) → khi nộp bản in,
đây là điểm chính (Trung chốt 29/10).

Nguyên văn ý của thầy (cột phải: SM1 đã trình bày gì, `docs/reports/sm1/sm1_report.tex`):
1. "Cho thêm disturbance dạng sin với tần số cao và biên độ 20% tín hiệu điều khiển lớn
   nhất". "Disturbance cộng vào tín hiệu đầu vào". — SM1 không có nhiễu.
2. Trung hỏi "nhiễu cộng vào cả 2 kênh (v và ω) hay chỉ 1 kênh" → thầy trả lời:
   "Vòng trong. Mô men e nhé. Kinetic." Tức là nhiễu ở **mức mô-men của vòng trong**;
   thầy **không** yêu cầu cụ thể "cả 2 kênh v, ω".
3. "So sánh với trượt": "SMC cơ bản hoặc NTSMC". — SM1 chỉ so "BC" với ADP.
4. "Mô phỏng phải có vòng điều khiển kinetic nữa" / "Thêm vòng trong nữa". — SM1 chỉ có mô hình
   động học.
5. "Vòng ngoài bậc nhất e thiết kế BC như nào — BC phải hệ bậc 2". — SM1 gọi luật Kanayama là
   "BC" (trích Wang 2025), chỉ đưa công thức, không có các bước thiết kế.

**Cách đọc (kiểm chứng 27/10/2026).** Trung đề xuất "yêu cầu của thầy xoay quanh khái niệm cơ bản,
không phải khái niệm mở rộng (BC của bài báo 1 vòng, BC cơ bản phải 2 vòng)". Đối chiếu SM1: đúng
phần lớn — cả 5 ý kéo luận văn về chuẩn nền tảng thay vì cách bài báo làm, rõ nhất ở ý 5. Ba điều
chỉnh:
- BC cơ bản = thiết kế đệ quy theo Lyapunov qua ≥ 2 bước trên hệ dạng chuỗi bậc ≥ 2, cho ra MỘT
  luật; "2 vòng" chỉ là cách đọc khi chuỗi là động học → động lực học. Mô hình động học không thuần
  bậc 1 (z_y bậc 2 theo ω qua z_θ): BC chỉ trên động học vẫn có (Jiang & Nijmeijer 1997); luật
  Kanayama đọc được thành BC 2 bước (k₂v_r z_y là số hạng chéo với V = ½(z_x²+z_y²) + (1−cos z_θ)/k₂).
- Chỉ ý 5 là sửa khái niệm; ý 1–4 là bổ sung thành phần chuẩn còn thiếu ở SM1 → tiêu chí đạt là
  "có, đúng chỗ thầy chỉ", KHÔNG soi định nghĩa rồi suy thêm yêu cầu.
- Không cấm phần mở rộng: thầy chấp nhận NTSMC; "SMC cơ bản" là đặt mức tối thiểu.

**Why:** Thầy muốn kiến trúc điều khiển đúng (hai vòng, đầu ra mô-men) và phân tích
bền vững trước khi sang phần chính của luận văn.

**How to apply:**
- Mọi mô phỏng dùng kiến trúc hai vòng; nhiễu `d = A·τ_max·sin(30t)·[1;1]` cộng vào mô-men — đạt
  (mọi PP chạm τ_max = 1 N·m trong giây đầu nên 20% τ_max = 20% max|τ|). Kịch bản vi sai/độc lập
  là kiểm tra thêm.
- Kết quả review 27/10 (theo cách đọc trên): 5/5 yêu cầu đạt; ý 5 kèm hạn chế đã ghi (bước 1 với
  ADP-FT không có V₁ chặt, Bổ đề 4.1). Hai chỗ gọi tên lỏng — đúng kiểu lỗi ý 5 — ĐÃ SỬA 27/10: nhãn
  "BS" → "Kanayama" (luận văn, tóm tắt, hình); mô tả NTSMC ở Mục 4.5.2 (mặt trượt đầu cuối nhanh
  của Yu & Man trên chuỗi z_θ → z_y, chống kỳ dị bằng nối tuyến tính; giữ tên NTSMC kèm định nghĩa).
- Lượt review đầu 27/10 kết luận "SMC/NTSMC chưa đạt, phải làm SMC tầng mô-men" — ĐÃ RÚT LẠI, do
  áp tiền đề khi chưa kiểm chứng ([[feedback-verify-premises]]).
- Vòng 2 hiện tại = luật mô-men có số hạng chéo + FT, xem [[feedback-backstepping-framing]].
- **Không ghi "theo yêu cầu của giáo viên hướng dẫn" trong luận văn** (Trung, 27/10): góp ý của thầy
  giúp hoàn thiện, không phải nội dung luận văn. Lời cảm ơn vẫn giữ.

Related: [[feedback-disturbance-one-channel]], [[feedback-backstepping-framing]], [[project-progress]]
