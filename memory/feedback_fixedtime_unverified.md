---
name: feedback-fixedtime-unverified
description: ADP-FT mất bám ở 3/5 điều kiện đầu trên kiến trúc hai vòng — tính hội tụ cố định thời gian không tồn tại trong triển khai; nguyên nhân đã được chẩn đoán
metadata:
  type: project
---

Kiểm chứng bằng MATLAB ngày 2026-08-13 (circle, T=60s, nhiễu 20%). Đã đưa vào
luận văn thành Mục 5.7.

## Kết quả: không phải "hội tụ chậm" mà là MẤT BÁM

Sai số xác lập `z_rms` (5s cuối) theo điều kiện đầu:

| ‖z0‖ | 0,245 | 0,612 | 1,225 | 2,375 | 3,500 |
|---|---|---|---|---|---|
| BS | 2,5e-4 | 2,0e-4 | 1,1e-4 | 1,5e-4 | 2,9e-3 |
| ADP-FT | 4,6e-2 | **3,79** | 1,3e-1 | **2,31** | **3,75** |
| CL | 5,8e-2 | **14,0** | 1,5e-1 | **3,45** | 5,4e-1 |

ADP-FT hỏng 3/5, CL hỏng 4/5, BS ổn định 5/5. Phụ thuộc **phi đơn điệu**: hỏng ở
0,612 nhưng ổn ở 1,225 → không phải ngưỡng đơn giản.

Nguyên nhân gốc: `W(T)` hội tụ về các giá trị khác nhau tùy z0, có trường hợp
**đổi dấu** (tại ‖z0‖=2,375: W₁ = −1,486 ứng với `zx²`) → `V̂ = Wᵀφ` mất tính xác
định dương → chính sách suy ra từ nó không còn ổn định hóa.

## Chẩn đoán: cái gì cứu được, cái gì không

| Can thiệp | Kết quả |
|---|---|
| Warm start W0 = [6,6,6] | Cứu được z0=0,612; nhưng Jc tăng trên cả 3 quỹ đạo (line 8,5→12,5) → **không đáng** |
| W0 = [0,0,0] | Hỏng toàn bộ 5/5 → warm start là bắt buộc |
| Nới clamp 1,5 → 3,0 | Cải thiện nhẹ, vẫn hỏng → clamp không phải nguyên nhân chính |
| Luật cập nhật `wang` | **Ổn định 3/5 thay vì 2/5** → luật cập nhật là yếu tố mạnh nhất |

## Điều còn bảo vệ được

Theorem 1 của Wang chỉ đảm bảo hội tụ vào **lân cận** gốc (eq.28), tức *practical*
fixed-time — không phải ‖z‖ = 0. Và chứng minh đó phát biểu cho hệ đơn vòng, không
phủ được kiến trúc hai vòng có clamp + bão hòa mô-men + nhiễu `g(z)e_η`. Nên kết
quả này **nhất quán** với lý thuyết gốc, không mâu thuẫn.

**Why:** Đây là tuyên bố nằm trên trang bìa. Phải có câu trả lời chuẩn bị sẵn.

**How to apply:** Luận văn (bản 2026-08-13) đã xử lý: §3.4 định nghĩa practical
fixed-time, §4.8 phân tích cascade và nói rõ giới hạn, §5.7 trình bày đầy đủ kết quả
này, §6.2 đưa lên thành hạn chế số 1. **Không được** quay lại tuyên bố hội tụ cố
định thời gian cho hệ hai vòng.

Related: [[feedback-wang-citation-mismatch]], [[feedback-ablation-results]], [[feedback-baseline-fairness]]
