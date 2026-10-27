---
name: project-progress
description: 27/10/2026 đã chạy lại bằng MATLAB, đổi cách gọi sang vòng 1/vòng 2, sửa bookmark PDF (luận văn 86 tr, tóm tắt 13 tr, PR #3); review yêu cầu của thầy: 5/5 đạt, còn sửa tên "BS", "NTSMC" — rồi gửi thầy Nam (thầy chỉ có v2, chưa đọc)
metadata:
  type: project
---

## Cập nhật 2026-10-27 (tối): review yêu cầu của thầy

Kiểm chứng tiền đề "thầy xoay quanh khái niệm cơ bản" với báo cáo SM1: đúng phần lớn, có 3 điều chỉnh
([[feedback-advisor-nam]]). Review theo cách đọc đã chỉnh: 5/5 yêu cầu đạt (BC kèm hạn chế bước 1
đã ghi). Còn hai chỗ gọi tên lỏng: nhãn "BS" cho luật Kanayama, tên/mô tả "NTSMC" (Mục 4.5.2).
Kết luận lượt đầu ("SMC/NTSMC chưa đạt, làm SMC tầng mô-men") đã rút lại. Chưa sửa luận văn —
chờ Trung đồng ý cách đọc.

## Cập nhật 2026-10-27: cách gọi vòng 1/vòng 2, chứng minh theo từng vòng, bookmark PDF

- Đổi "bước 1/2" → "vòng 1/2" toàn bộ luận văn + tóm tắt ([[feedback-backstepping-framing]]); Mục 4.4
  sắp chứng minh theo vòng 1 → vòng 2 + toàn hệ; báo cáo tóm tắt Mục 2 viết lại đúng thứ tự thiết kế,
  có chứng minh rút gọn.
- Sửa bookmark PDF (chương sau bị gộp vào Chương 1 do anchor trùng) và anchor công thức trùng giữa các
  chương. Mọi thay đổi nằm trên nhánh `docs/ket-qua-buoc-2-matlab` (PR #3), chưa merge.
- Còn lại (Trung): đọc lại Ch.4–6 và Mục 2 báo cáo tóm tắt, merge PR #3, gửi thầy Nam bản mới kèm
  giải thích lỗi B và vòng 2. Không còn thông số nào phải "chốt theo phần cứng": luận văn chỉ thiết kế
  và mô phỏng ([[feedback-simulation-only]]); việc v thực vượt v_max đã ghi trong luận văn, không sửa.

## Cập nhật 2026-10-26 (khuya): vòng 2 đầy đủ, chạy lại bằng MATLAB, build xong

- PR #1 (vòng 2 đầy đủ, Chương 4 mới) đã merge vào master. Phiên local kéo về, chạy lại
  `sim_thesis`, `sim_sm1_full`, `sim_thesis_extra`, `sweep_gains` bằng MATLAB R2023a; xuất 13 hình
  luận văn; KHÔNG chạy lại `sim_sm2` (giữ hình SM2 đã nộp).
- Viết lại Ch.5 (thêm Mục 5.6 vòng 2, Mục 5.4 dùng hàm mô tả), Ch.6, tóm tắt VI/EN, lời cam đoan,
  phần kết quả báo cáo tóm tắt; README, workflow, CLAUDE.md (đường dẫn MATLAB/MiKTeX mới).
  Số liệu: [[ket-qua-cuoi]]. Build MiKTeX: 0 lỗi, 0 ref treo, 0 Overfull.
- Kết quả đổi so với bản 72 trang: ADP-FT không còn "thứ hai" mà ngang NTSMC (4,140 vs 4,130).
- Còn lại (Trung): đọc lại Ch.4–6, gửi thầy Nam bản mới kèm giải thích lỗi B và vòng 2.
  (Ghi "quyết τ_max thật" ở bản ghi này là hiểu nhầm — xem cập nhật 27/10.)

## Cập nhật 2026-10-26 (tối): đã viết lại xong

- Luận văn viết lại theo mô hình B đúng + gain chỉnh đều (Ch.1.4, 4.3–4.8, Ch.5, Ch.6, tóm
  tắt VI/EN): 72 trang, compile sạch bằng TeX Live. Báo cáo tóm tắt viết lại phần kết quả.
  Số liệu chốt: [[ket-qua-cuoi]].
- Khung: "hoàn thiện ADP-FT của Wang et al. cho robot điều khiển bằng mô-men"; ADP-FT vẫn là
  phương pháp chính, các kết quả bất lợi (học critic không giúp, fixed-time không bảo toàn,
  SMC đã chỉnh thấp nhất) báo cáo đầy đủ.
- Còn lại (Trung): chạy MATLAB `sim_sm2; sim_sm1_full; sim_thesis; sim_thesis_extra` rồi
  `export_all_figures` (hình trong repo VẪN là mô hình cũ), build MiKTeX, đọc lại, commit
  PDF, gửi thầy kèm giải thích lỗi B. τ_max = 1 N·m (tham số mô phỏng, Trung đã chốt chiều 26/10).

## Cập nhật 2026-10-26 (chiều): quyết định của Trung

- Thầy Nam đã nhận v2 + báo cáo tóm tắt (26/05, làm SAU chat 28/03 — xem
  [[feedback-advisor-nam]]) nhưng **chưa đọc**. Trung muốn hoàn thiện rồi in ra nộp.
- ADP-FT **vẫn là phương pháp chính**, kiến trúc hai vòng là bắt buộc (yêu cầu của thầy).
- Giữ tham số robot (m=10, I=0,5, r=0,05, L=0,15), τ_max = 1 N·m, bỏ nhãn Pioneer 3-DX.
- Chỉnh gain nhẹ, đều nhau cho mọi phương pháp (`simulations/sweep_gains.m`).
- Trung tự chạy MATLAB (xuất hình) + MiKTeX (build PDF); Claude viết lại `.tex`.

## Cập nhật 2026-10-26

Sau 2,5 tháng không động tới, Trung hỏi lại tiến độ. Kiểm tra cho thấy:
- Bản mới nhất: `thesis.pdf` 78 trang, build 2026-08-13, chưa ghi nhận đã gửi thầy.
  Thầy Nam vẫn chỉ có v2 (43 trang, commit `7fedea9`); 5 tháng chưa phản hồi.
- Bản 78 trang tự mâu thuẫn: đợt thêm ghép nonholonomic (tối 13/08) chưa lan hết sang
  Ch.5–6 (vd. `thesis.tex:1564`, `:2354`, `:2366`, `:2226`, `:2439`, `:2474–2489`).
  Báo cáo tóm tắt còn cũ hơn (vẫn ghi "ADP-FT tốt nhất").
- **Quan trọng hơn:** mô hình có lỗi ma trận B → xem [[feedback-b-matrix-error]].
  Trung chọn: sửa code + chạy lại trước (đã làm 2026-10-26), phần viết ghi thành
  checklist trong PROGRESS.md (VIEC CAN LAM mục 2) để làm sau.

## Trạng thái tính đến 2026-08-12

**Đang chờ phản hồi GVHD.** Gửi bản v2 cho thầy Nam qua Teams ngày 2026-05-26.
Đến 2026-08-12 vẫn chưa có phản hồi — đã 2,5 tháng. Repo không có commit nào
trong khoảng này (ngoài đợt tái cấu trúc 2026-08-12).

**Chưa có lịch bảo vệ.** Kế hoạch gốc 16 tuần (02→06/2026) đã quá hạn và không
còn dùng làm mốc thời gian.

### Đã nộp (PDF tiếng Việt có dấu)
- `docs/thesis/thesis.pdf` — 43 trang, 6 chương, 20 hình
- `docs/reports/summary/summary_report.pdf` — 13 trang
- `docs/reports/sm2/sm2_report.pdf` — 15 trang
- `docs/reports/sm1/sm1_report.pdf` — 10 trang (báo cáo 2026-03-28)

Toàn bộ code mô phỏng + 19 figure PDF đã xong.

### Còn lại
Bị chặn bởi GVHD: hoàn thiện luận văn theo phản hồi, chuẩn bị slide bảo vệ.
Làm được ngay: cải thiện SMC (z_rms 0.276 còn cao, xem xét NTSMC).

**Why:** Trung hỏi lại tiến độ sau thời gian dài không đụng tới dự án — cần biết
ngay là đang chờ ai, và việc gì làm được mà không phải chờ.

**How to apply:** Khi thầy Nam phản hồi → tạo memory `feedback_*` mới cho nội dung
góp ý, rồi cập nhật file này. Đừng giả định đã bảo vệ xong hay đã có phản hồi
nếu không có bằng chứng — kiểm tra `git log` và ngày sửa file trước khi kết luận.

Related: [[feedback-advisor-nam]], [[feedback-vietnamese]], [[feedback-adp-ft-tuning]]
