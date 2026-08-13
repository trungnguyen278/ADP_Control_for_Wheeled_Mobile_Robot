---
name: feedback-wang-citation-mismatch
description: Luận văn trích dẫn sai bài báo Wang 2025 ở 3 chỗ - tên tác giả, công thức eq.(17), và cột tham số gốc trong Bảng 4.1
metadata:
  type: project
---

Đối chiếu `docs/thesis/thesis.tex` với bài báo gốc trong `references/`
(`Adaptive_Dynamic_Programming-Based_Fixed-Time_Optimal_Control_for_Wheeled_Mobile_Robot.pdf`)
ngày 2026-08-13. Ba sai lệch, đều nằm ở phần trích dẫn bài báo nền tảng.

Lưu ý: `CLAUDE.md` và `README.md` đã ghi ĐÚNG tên tác giả từ trước — chỉ riêng
bibliography trong các file `.tex` là sai.

## 1. Tên tác giả sai (lan ra cả 4 tài liệu)

Thật: **Chen Wang, Haoran Zhan, Qing Guo, Tieshan Li**, "Adaptive Dynamic
Programming-Based Fixed-Time Optimal Control for Wheeled Mobile Robot".

Đang ghi: "Y. Wang, Z. Li, et al." trong `thesis.tex`, `sm1_report.tex`,
`sm2_report.tex`; riêng `summary_report.tex:440` còn bịa thêm "C. Yang" và
đổi luôn tên bài báo thành "ADP-based fixed-time optimal tracking control...".

## 2. eq.(17) trong luận văn KHÔNG phải eq.(17) của Wang

| | Công thức |
|---|---|
| Wang eq.(17) | `Ẇ = ½Γ[∇φ·B·R⁻¹·Bᵀ·z − κ₁Ŵ − κ₂(ŴᵀŴ)Ŵ]` |
| Luận văn §4.7.2 + `adp_fixed_time.m:110` | `Ẇ = −½Γ[σ̄·ε + κ₁W + κ₂(WᵀW)W]`, với `ε` là sai số Bellman |

Wang **không** dùng sai số Bellman trong luật cập nhật (xem Remark 2 của bài báo:
họ cố tình tránh gradient descent vì nó kéo theo điều kiện PE). Luận văn thay
bằng gradient descent trên Bellman error — đây là thay đổi có chủ đích của Trung
(PROGRESS 2026-05-11: "sua weight update (Bellman error gradient)", Jc 4385→123),
nhưng đang bị trình bày như là công thức của Wang.

Hệ quả kép: (a) trích dẫn sai; (b) chứng minh fixed-time của Wang (Theorem 1,
eq.19–28) dựa trên eq.(17) gốc nên **không còn áp dụng được** cho luật đã đổi.

## 3. Bảng 4.1 — cột "Wang" sai toàn bộ

Tham số thật của Wang (bài báo trang 180, mục IV-A):

| | Luận văn ghi là "Wang" | Wang thực tế |
|---|---|---|
| λ = μ = α | [0.3, 0.3, 0.3] | **diag(1, 1, 0.4)** |
| β | [1.0, 1.0, 1.0] | **diag(1e5, 1e5, 1e5)** |
| ρ | 0.05 | **10** |
| κ₁ | 0.02 | **0.1** (κ₂ cũng = 0.1) |
| Γ | 2I₆ | 2I₆ (đúng) |
| W(0) | — | **[0,0,0,0,0,0]ᵀ** |

Cột "Wang" hiện tại thực chất là giá trị **trung gian của chính Trung** sau bước
tune thứ nhất, không phải giá trị gốc. Kéo theo hai tuyên bố sai:
- "giảm gains 3–5 lần": thực tế β giảm ~666.000 lần, ρ giảm 100 lần.
- "cần warm start W₀≠0" ghi là hạn chế chung của ADP — nhưng Wang chạy được với
  W(0)=0 nhờ β=1e5 giữ ổn định khi critic chưa học. Warm start là hệ quả của
  việc luận văn giảm β, không phải hạn chế cố hữu.

**Why:** Bảng 4.1 chính là chỗ luận văn tuyên bố "đóng góp chính". Cột đối chứng
sai làm đóng góp đó không kiểm chứng được, và hội đồng chỉ cần mở bài báo là thấy.

**How to apply:** Sửa cột "Wang" theo đúng bài báo; đổi câu "giảm 3–5 lần" thành
số thật; tách eq.(17) đã sửa ra thành đóng góp riêng có tên (thay vì giấu dưới
tên Wang) và nói rõ chứng minh gốc không còn phủ được luật mới.

Related: [[project-thesis]], [[feedback-adp-ft-tuning]], [[feedback-fixedtime-unverified]]
