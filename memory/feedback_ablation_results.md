---
name: feedback-ablation-results
description: Ablation cho thấy các số hạng fixed-time cải thiện 25 lần trên figure-8; còn luật Bellman vs Wang là đánh đổi hai chiều không có phương án thắng tuyệt đối
metadata:
  type: project
---

Hai thí nghiệm ablation chạy ngày 2026-08-13, đã đưa vào luận văn Mục 5.6.
Script: `simulations/sim_thesis_extra.m`.

## 1. ADP-UUB — các số hạng fixed-time có giá trị thật

ADP-UUB = ADP-FT bỏ `kappa2`, bỏ `beta·z³`, và cho `p/q = 1` (Wang eq.32–33).
Cấu hình bằng tham số, không sửa controller: `s.ft_kappa2=0; s.ft_beta=[0;0;0];
s.ft_p = s.ft_q`.

| Quỹ đạo | ADP-FT | ADP-UUB | Chênh |
|---|---|---|---|
| circle | **85,2** | 109,8 | −22% |
| line | 8,5 | **7,6** | +12% |
| figure8 | **93,2** | 2357,6 | **−96% (25 lần)** |

Các số hạng fixed-time là **cơ chế bảo hiểm cho trường hợp khó**, không phải cơ chế
tối ưu hóa chung — trên quỹ đạo dễ chúng là chi phí thừa. Không có chúng, ADP-UUB
trên figure-8 tệ ngang SMC (2357 vs 2457).

## 2. Luật cập nhật: Bellman vs Wang eq.(17) — đánh đổi hai chiều

Bật bằng `s.ft_update_law = 'wang' | 'bellman'` (thêm vào `adp_fixed_time.m`
2026-08-13; mặc định `'bellman'`, giữ nguyên hành vi cũ).

| Luật | Jc tròn | Jc thẳng | Jc số 8 | z0 ổn định | Jc tại 14kg |
|---|---|---|---|---|---|
| Bellman (mặc định) | **85,2** | **8,5** | 93,2 | 2/5 | **4283** |
| Wang nguyên bản | 89,4 | 9,2 | **62,8** | **3/5** | 6495 |

**Không có phương án thắng tuyệt đối.** Bellman thắng 3 tiêu chí (circle, line,
robust khối lượng), Wang thắng 2 (figure-8, miền hút). Giữ Bellman làm mặc định vì
circle + robust khối lượng là hai tiêu chí trọng tâm của đề tài.

Quan trọng: mức cải thiện `Jc 4385 → 123` ghi trong PROGRESS 2026-05-11 **không
phải** do đổi luật cập nhật — phần lớn đến từ tune gain. Đừng quy nhầm lần nữa.

**Why:** Hai ablation này là bằng chứng trực tiếp cho câu hỏi "đóng góp của anh là
gì và nó đáng giá bao nhiêu" — câu hội đồng chắc chắn hỏi.

**How to apply:** Khi trình bày, nói rõ luật Bellman là *lựa chọn có đánh đổi*, không
phải cải tiến thuần túy. Nếu bị hỏi về figure-8, câu trả lời là: hạn chế đó một phần
do luật cập nhật, đổi sang luật gốc thì Jc còn 62,8 (tốt hơn cả CL) nhưng mất 34%
độ bền khối lượng.

Related: [[feedback-fixedtime-unverified]], [[feedback-wang-citation-mismatch]]
