# results/

Thư mục chứa kết quả mô phỏng dạng `.mat`.

## ⚠ Các file `.mat` KHÔNG có trong git

Chúng bị `.gitignore` chặn **có chủ đích**: mỗi file 80–300 MB, và GitHub chặn cứng
mọi file lớn hơn 100 MB. Trước đây repo từng commit chúng, làm `.git` phình lên 578 MB
và không push lên GitHub được.

Toàn bộ đều **tái tạo được** bằng cách chạy lại script mô phỏng.

## Tái tạo

`cd` vào `simulations/` rồi chạy script tương ứng:

```powershell
$MATLAB = "C:\Program Files\MATLAB\R2023a\bin\matlab.exe"
$REPO   = "c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot"
& $MATLAB -batch "cd('$REPO\simulations'); sim_thesis"
```

| File | Sinh từ | Kích thước | Thời gian |
|---|---|---|---|
| `test_openloop_results.mat` | `simulations/test_openloop.m` | ~50 KB | ~10 s |
| `sm1_results.mat` | `simulations/sim_sm1.m` | ~92 MB | ~1 phút |
| `sm1_full_results.mat` | `simulations/sim_sm1_full.m` | ~82 MB | ~3 phút |
| `sm2_results.mat` | `simulations/sim_sm2.m` | ~118 MB | ~2 phút |
| `thesis_results.mat` | `simulations/sim_thesis.m` | ~300 MB | ~5 phút |

## Dùng để làm gì

Các script trong `plotting/` đọc `.mat` từ đây và xuất hình PDF vào `docs/`:

- `export_sm1_figures.m` ← `sm1_results.mat`
- `export_all_figures.m` ← `sm2_results.mat`, `thesis_results.mat`, `sm1_full_results.mat`

Nếu `export_*` báo không tìm thấy file, chạy lại `sim_*` tương ứng theo bảng trên.

Chi tiết quy trình: [../docs/guides/workflow.md](../docs/guides/workflow.md)
