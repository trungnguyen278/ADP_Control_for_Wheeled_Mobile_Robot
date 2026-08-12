# Quy ước commit

Chuẩn: [Conventional Commits](https://www.conventionalcommits.org/), **mô tả bằng
tiếng Việt KHÔNG dấu** (tránh lỗi encoding trên Windows console và GitHub).

---

## Cú pháp

```
<type>(<scope>): <mo ta ngan, khong dau, khong viet hoa dau, khong cham cuoi>

[than bai — tuy chon, giai thich TAI SAO chu khong phai LAM GI]

[footer — tuy chon]
```

Ví dụ:

```
feat(controllers): them ADP fixed-time controller theo Wang et al. eq.16-18
fix(adp-ft): sua weight update dung Bellman error gradient thay vi gradient descent
docs(thesis): mo rong chuong 4, them 8 hinh TikZ minh hoa kien truc
sim(thesis): chay robustness 3 quy dao x 5 phuong phap
chore(repo): them gitignore, bo results/*.mat khoi tracking
```

---

## Bảng `type`

| Type | Dùng khi | Ví dụ |
|---|---|---|
| `feat` | Thêm controller / mô hình / kịch bản mô phỏng mới | `feat(controllers): them SMC vong ngoai` |
| `fix` | Sửa lỗi sai công thức, sai dấu, sai chỉ số | `fix(models): sua dau ma sat Coulomb trong wmr_dynamics` |
| `tune` | Chỉ đổi giá trị tham số, không đổi cấu trúc | `tune(adp-ft): giam robust gain 3x cho dual-loop` |
| `sim` | Chạy mô phỏng, cập nhật số liệu kết quả | `sim(thesis): chay lai phan C voi 4 muc nhieu` |
| `docs` | Luận văn, báo cáo, README, guides, memory | `docs(sm2): them phan phan tich Bellman error` |
| `refactor` | Đổi cấu trúc code/thư mục, không đổi hành vi | `refactor(repo): gom bao cao vao docs/reports` |
| `chore` | Cấu hình, gitignore, settings, dọn dẹp | `chore(claude): them skill build-doc` |

> `tune` và `sim` không có trong Conventional Commits gốc — thêm vào vì đây là repo
> nghiên cứu, việc "chỉ đổi tham số rồi chạy lại" xảy ra rất thường xuyên và cần
> phân biệt rõ với `fix` (sửa sai) và `feat` (thêm mới).

---

## Bảng `scope`

**Theo module code:**

| Scope | Tương ứng |
|---|---|
| `models` | `models/` |
| `controllers` | `controllers/` (nhiều bộ cùng lúc) |
| `adp-ft`, `cl`, `smc`, `bs`, `adp-ac` | Một controller cụ thể |
| `simulations` | `simulations/` |
| `plotting` | `plotting/` |

**Theo tài liệu:**

| Scope | Tương ứng |
|---|---|
| `thesis` | `docs/thesis/` |
| `sm1`, `sm2`, `summary` | `docs/reports/*/` |
| `guides` | `docs/guides/` |
| `memory` | `memory/` |

**Khác:** `repo` (cấu trúc tổng thể), `claude` (`.claude/`), `deps` (môi trường).

Scope có thể bỏ nếu thay đổi trải rộng: `chore: don dep file tam`.

---

## Quy tắc nội dung

1. **Một commit = một ý.** Đừng gộp "thêm controller + viết báo cáo + chạy sim"
   vào một commit như `2b6cfef` cũ (37 file, 3 loại việc khác nhau).

2. **Mô tả ở thể mệnh lệnh, không quá 72 ký tự.**
   ✅ `them quy dao figure-8` — ❌ `da them quy dao figure-8 vao ref_trajectory`

3. **Có số liệu thì đưa vào thân bài.** Đây là repo nghiên cứu, con số là kết quả:

   ```
   tune(adp-ft): giam robust gain cho kien truc dual-loop

   Gain goc cua Wang danh cho kinematic-only. Trong dual-loop, vong trong
   da khu nhieu nen gain ngoai phai nho hon 3-5x, neu khong se phan hoi duong.

   ft_lambda/mu/alpha: 0.3  -> 0.08
   ft_beta:            1.0  -> 0.15
   Jc (circle, 20% dist): 152 -> 85.2
   ```

4. **Không commit khi mô phỏng đang chạy dở** — `results/*.mat` không vào git nên
   commit code trước, chạy sim sau, rồi commit `PROGRESS.md` với số liệu.

5. **Cập nhật `PROGRESS.md` cùng commit tạo ra kết quả**, không để dồn cuối tuần.

---

## Không bao giờ commit

- `results/*.mat` — 80–300 MB, GitHub chặn cứng > 100 MB. Đã có `.gitignore`.
  **Không dùng `git add -f` để lách.**
- File build LaTeX: `.aux`, `.log`, `.out`, `.toc`, `.synctex.gz`.
- `.claude/settings.local.json` — permission cá nhân theo máy.

**Có** commit: `thesis.pdf`, `*_report.pdf`, và mọi PDF/PNG trong `docs/**/figures/`.

---

## Mẫu thường dùng

| Tình huống | Commit |
|---|---|
| Viết xong 1 chương luận văn | `docs(thesis): viet chuong 3 thiet ke bo dieu khien` |
| Sửa hình bị đặt sai chỗ | `docs(thesis): doi hinh ra khoi giua cong thuc va giai thich` |
| Xuất lại figure sau khi chạy sim | `docs(thesis): cap nhat 12 figure tu ket qua moi` |
| Đổi gain rồi chạy lại | `tune(cl): tang cl_uo_max len 2.0` + `sim(sm2): chay lai voi gain moi` |
| Ghi nhận feedback của thầy | `docs(memory): ghi feedback GVHD ve phan robustness` |
