---
name: Thesis project overview
description: ADP Fixed-Time Optimal Control dual-loop for differential-drive WMR, extending Wang et al. 2025 with dynamic model
type: project
---

**Đề tài**: Mở rộng ADP-based Fixed-time Optimal Control (Wang et al., IEEE RA-L 2025) từ kinematic-only sang dual-loop kinematic + dynamic cho WMR vi sai.

**Đóng góp chính**:
1. Thêm dynamic model (Fierro & Lewis 1997): M·η̇ = B·τ − F(η)
2. Dual-loop: Vòng ngoài ADP → (v_d,ω_d) → Vòng trong PI robust → (τR,τL) → Dynamic
3. ADP giữ nguyên eq.(16)-(18) Wang, chỉ thêm vòng trong + plant dynamic

**2 Seminar trước luận văn**:
- SM1: Actor-Critic ADP truyền thống (cần PE, 2 mạng)
- SM2: Critic-only + Concurrent Learning (bỏ PE, 1 mạng)

**Why:** Kế hoạch 16 tuần, nhưng thực tế linh hoạt do Trung ít thời gian.
**How to apply:** Ưu tiên code chạy được, test pass, rồi mới polish.
