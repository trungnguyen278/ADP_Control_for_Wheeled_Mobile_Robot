---
name: feedback-advisor-nam
description: PGS.TS. Nguyen Hoai Nam feedback on SM1 results (2026-03-28) - requires disturbance, SMC, 2-loop architecture
metadata:
  type: feedback
---

Feedback tu thay Nguyen Hoai Nam sau bao cao SM1 (2026-03-28):

1. Them disturbance dang sin, tan so cao, bien do 20% tin hieu dieu khien lon nhat.
2. Disturbance cong vao mo-men (vong trong), ca 2 kenh.
3. So sanh voi SMC co ban hoac NTSMC (Non-singular Terminal SMC) o vong ngoai kinematic.
4. Mo phong phai co 2 vong: kinematic controller (ngoai) + dynamic controller (trong).
5. Backstepping phai bac 2 (khong chi kinematic BC bac 1).
6. Output vong trong la mo-men tau, khong phai van toc.

**Why:** Thay muon thay robustness analysis va kien truc controller dung truoc khi chuyen sang phan chinh luan van.
**How to apply:** Moi mo phong deu phai dung kien truc 2 vong (kinematic + dynamic). Luon co kich ban nhieu. SMC la baseline so sanh bat buoc.
