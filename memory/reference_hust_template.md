---
name: reference-hust-template
description: Mẫu slide chính thức của ĐHBK Hà Nội ("HUST | Powerpoint Template" 2022) — nơi tải, cách tải bằng curl, màu/font/ảnh nền; máy Trung không có PowerPoint nên slide làm bằng Beamer + XeLaTeX
metadata:
  type: reference
---

Trung yêu cầu slide bảo vệ "template nhớ lấy của trường" (27/10/2026). Mẫu chính thức:

- Trang: hust.edu.vn → Hệ thống nhận diện thương hiệu (NDTH) → "HUST | Powerpoint Template" (37 MB,
  5 file `hust_powerpoint_template_part01..05.pptx`; part01 là bộ 16:9). Trang cũ "HUST PPT template 2022
  RED 16x9" nay chuyển về trang danh sách. Cùng mục có sổ tay nhận diện thương hiệu, bộ logo, bộ font.
- Tải bằng curl: GET trang
  `https://hust.edu.vn/vi/he-thong-nhan-dien-thuong-hieu/He-thong-NDTH/hust-powerpoint-template.html` để
  nhận cookie, rồi GET `https://hust.edu.vn/index.php?language=vi&nv=he-thong-nhan-dien-thuong-hieu&op=down&filename=hust_powerpoint_template_part01.pptx`
  với cùng cookie + Referer (nút tải là JS `nv_download_file`, CMS NukeViet).
- Thiết kế: ảnh nền toàn slide 8000×4500 — nền đỏ (~#B71F24) có dải lụa cam + nét vẽ tòa nhà cho bìa/cảm
  ơn; trang nội dung nền trắng, dải lụa ở góc, băng đỏ #CD1628 dưới cùng (cao 8% slide) có logo/chữ
  "ĐẠI HỌC BÁCH KHOA HÀ NỘI". Font: Lato (MiKTeX có Lato TTF; XeLaTeX hiển thị đủ dấu tiếng Việt).
- Máy Trung không có PowerPoint lẫn LibreOffice → không tự kiểm tra được .pptx; slide làm bằng Beamer +
  XeLaTeX dùng đúng ảnh nền/logo/màu/font của mẫu (`docs/slides/hust/`), ra PDF. Nếu cần .pptx sửa được
  thì phải có công cụ dựng ảnh để soát trước.

Related: [[feedback-map-first]], [[project-progress]]
