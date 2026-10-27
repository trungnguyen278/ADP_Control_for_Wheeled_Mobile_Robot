---
name: project-thesis
description: Tên đề tài "Ứng dụng điều khiển ADP cho xe tự hành"; nội dung: ADP-FT cho WMR vi sai, hoàn thiện Wang et al. 2025 thành bộ điều khiển hai vòng thiết kế theo backstepping điều khiển bằng mô-men
metadata:
  type: project
---

**Tên đề tài đã đăng ký** (Trung, 27/10/2026; dùng nguyên văn ở bìa, lời cam đoan, báo cáo tóm tắt):
"Ứng dụng điều khiển ADP cho xe tự hành". Nội dung xét xe tự hành ở dạng robot di động hai bánh vi sai.

**Nội dung**: Hoàn thiện ADP-based Fixed-time Optimal Control (Wang et al., IEEE RA-L 2025) thành
một bộ điều khiển hai vòng thiết kế theo backstepping cho WMR vi sai nhận lệnh mô-men (mô hình 5 trạng thái).

**Đóng góp chính** (từ 2026-10-26 tối, xem [[feedback-backstepping-framing]]):
1. Vòng 1 = ADP-FT của Wang (giữ eq.(16)-(18)) + ghép nonholonomic + toán tử chiếu + gain mới
2. Vòng 2 = luật mô-men trên mô hình Fierro & Lewis (`controllers/dynamic_backstepping.m`),
   thêm số hạng chéo K_c·[z_x; z_θ] và số hạng cố định thời gian → hàm Lyapunov chung (Định lý 4.1).
   KHÔNG phải "PI robust" — mô tả cũ sai, đã từng dẫn tới file rỗng `inner_loop_pi.m`.
3. Phân tích lại chứng minh Wang: khoảng hở kênh ngang ([[feedback-wang-gap]])

**2 Seminar trước luận văn**:
- SM1: Actor-Critic ADP truyền thống (cần PE, 2 mạng)
- SM2: Critic-only + Concurrent Learning (bỏ PE, 1 mạng)

**Why:** Kế hoạch gốc 16 tuần (02→06/2026) đã quá hạn; tiến độ thực tế linh hoạt
do Trung ít thời gian. Trạng thái hiện tại xem [[project-progress]].
**How to apply:** Ưu tiên code chạy được, test pass, rồi mới polish.
