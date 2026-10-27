---
name: feedback-figure-placement
description: Hình không chen giữa công thức và giải thích; chú giải ngoài vùng vẽ, căn trái; nhãn/mũi tên không đè nhau — soát bằng mắt ở 200–300 dpi bằng pdftoppm
metadata:
  type: feedback
---

Không đặt hình giữa công thức và phần giải thích ký hiệu. Đặt hình tại điểm chuyển đoạn tự nhiên (cuối mục, trước subsection mới). Chú giải không che nội dung đồ thị: đặt ngoài vùng vẽ (bên phải hoặc dưới), chữ căn trái. Trên hình TikZ, nhãn không đè lên mũi tên/trục/nhãn khác.

**Why:** Trung phàn nàn hai lần. Lần đầu: "nhiều hình bị chèn giữa phần giải thích ký hiệu và nội dung". Lần hai (27/10/2026): "các ký tự bị đè lên nhau... nhiều hình khác cũng bị", chú giải "căn lề chưa đều" (pgfplots mặc định căn giữa từng ô), mũi tên hình Concurrent Learning đè nhau. Lượt sửa đầu chỉ tính tọa độ rồi xem cả trang ở ~100 dpi nên vẫn sót: nhãn ω nằm trên viền thân robot, z_θ đè mũi tên θ_r, (v, ω) lửng giữa hai đường hồi tiếp, đầu mũi tên đè góc rẽ. Chỉ phóng to mới thấy.

**How to apply:**

- Sau khi sửa hình: `pdftoppm -r 300 -f N -l N -x .. -y .. -W .. -H .. -png thesis.pdf out` (có sẵn trong MiKTeX) rồi xem ảnh cắt; tìm trang bằng pypdf, tìm "Hình X.Y:".
- Nhãn góc: góc hẹp (< 40°) thì đặt nhãn ra ngoài góc, đừng đặt ở giữa. Nhãn đường hồi tiếp: đặt về phía không có đường khác. Nhánh rẽ vào hai khối: cho hai khối cùng rộng, đoạn ngang ≥ 0,5 cm.
- pgfplots: `\node` đặt ngoài [ymin, ymax] bị cắt mất (nhãn T_f = π ở Hình 3.2 từng biến mất). `legend cell align=left` đặt ở `\pgfplotsset` toàn cục.
- Cỡ chữ gốc 13pt (`\changefontsizes{13pt}`): `\small` ≈ 12pt, `\footnotesize` chỉ nhỏ hơn ~6%, nên đổi cỡ chữ không cứu được hình chật. Trước khi xếp tọa độ hình nhiều khối, đo bề rộng chữ bằng `\settowidth` + `\typeout` với chính preamble luận văn, rồi thử trên tài liệu độc lập (vài giây) trước khi ghép vào thesis.tex.
- Hình 4.3 là sơ đồ ĐẦY ĐỦ bộ điều khiển (Trung 27/10: "chưa có 1 hình nào cho bộ điều khiển của chúng ta 1 cách đầy đủ"). Đổi/thêm/bớt thành phần nào của bộ điều khiển thì phải sửa cả hình này.
- MATLAB: tín hiệu tần số cao trên cửa sổ dài thành dải màu đặc, và đường vẽ sau che đường vẽ trước. Dùng cửa sổ phóng to vài chu kỳ (Hình 5.6). Chữ trên hình chỉ dùng Latin-1 (xem [[feedback-vietnamese]]).

Related: [[feedback-vietnamese]], [[project-progress]]
