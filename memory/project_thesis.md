---
name: project-thesis
description: ADP Fixed-Time Optimal Control dual-loop for differential-drive WMR, extending Wang et al. 2025 with dynamic model
metadata:
  type: project
---

**Đề tài**: Mở rộng ADP-based Fixed-time Optimal Control (Wang et al., IEEE RA-L 2025) từ kinematic-only sang dual-loop kinematic + dynamic cho WMR vi sai.

**Đóng góp chính**:
1. Thêm dynamic model (Fierro & Lewis 1997): M·η̇ = B·τ − F(η)
2. Dual-loop: Vòng ngoài ADP → (v_d,ω_d) → Vòng trong dynamic backstepping → (τR,τL) → Dynamic
   (`controllers/dynamic_backstepping.m`. KHÔNG phải "PI robust" — mô tả cũ sai,
   đã từng dẫn tới file rỗng `inner_loop_pi.m`, xóa ở commit 4e4c2e0)
3. ADP giữ nguyên eq.(16)-(18) Wang, chỉ thêm vòng trong + plant dynamic

**2 Seminar trước luận văn**:
- SM1: Actor-Critic ADP truyền thống (cần PE, 2 mạng)
- SM2: Critic-only + Concurrent Learning (bỏ PE, 1 mạng)

**Why:** Kế hoạch gốc 16 tuần (02→06/2026) đã quá hạn; tiến độ thực tế linh hoạt
do Trung ít thời gian. Trạng thái hiện tại xem [[project-progress]].
**How to apply:** Ưu tiên code chạy được, test pass, rồi mới polish.
