---
name: checkpoint
description: Chốt một mốc công việc của luận văn - cập nhật PROGRESS.md, ghi memory nếu có điều đáng nhớ, rồi commit theo Conventional Commits tiếng Việt không dấu. Dùng khi vừa hoàn thành một sprint, chạy xong mô phỏng có kết quả mới, viết xong một chương, hoặc nhận feedback từ GVHD. Trigger - "chốt tiến độ", "checkpoint", "ghi tiến độ", "commit lại", "xong rồi lưu lại", "cập nhật PROGRESS".
---

# Chốt mốc công việc

Chạy tuần tự 4 bước. Không bỏ bước nào.

## Bước 1 — Xem đã thay đổi những gì

```bash
git status --short
git diff --stat
```

Phân loại thay đổi theo nhóm để commit tách bạch (xem Bước 4).

## Bước 2 — Cập nhật PROGRESS.md

Thêm mục mới vào phần **LỊCH SỬ TIẾN ĐỘ**, theo mẫu:

```markdown
### YYYY-MM-DD — <Tieu de sprint>
- [x] <viec da lam cu the>
- [x] <viec da lam cu the>
  - <chi tiet ky thuat, gia tri tham so truoc -> sau>
```

Nếu có số liệu mới → cập nhật luôn bảng **Kết quả so sánh** (Jc, z_rms) trong cùng file.
Nếu xong/thêm việc → cập nhật mục **VIỆC CẦN LÀM**.

> Ngày phải là ngày tuyệt đối `YYYY-MM-DD`, không viết "hôm nay", "tuần này".

## Bước 3 — Ghi memory nếu đáng nhớ

Ghi vào `memory/` khi có:

- Feedback mới từ PGS.TS. Nguyễn Hoài Nam → `feedback_*.md`
- Bài học tuning không hiển nhiên từ code (vì sao gain phải nhỏ, clamp nào bắt buộc)
- Ràng buộc/quyết định của dự án không suy ra được từ git history

**Không ghi memory** những thứ code hoặc `PROGRESS.md` đã nói (cấu trúc thư mục,
lịch sử commit, việc đã sửa lỗi gì).

Format mỗi memory:

```markdown
---
name: <slug-kebab-case>
description: <một dòng tóm tắt>
metadata:
  type: user | feedback | project | reference
---

<nội dung>

**Why:** <tại sao điều này quan trọng>
**How to apply:** <áp dụng thế nào lần sau>

Related: [[slug-memory-khac]]
```

Rồi thêm 1 dòng vào `memory/MEMORY.md`: `- [Tiêu đề](ten_file.md) — mô tả ngắn`.

Trước khi tạo file mới, **kiểm tra memory đã có** — cập nhật file cũ thay vì tạo trùng.
`memory/project_progress.md` gần như luôn cần cập nhật.

## Bước 4 — Commit tách bạch

Một commit = một ý. Đừng gộp code + tài liệu + kết quả vào một commit.

Thứ tự thường dùng:

```bash
git add models/ controllers/ simulations/ plotting/
git commit -m "feat(controllers): <mo ta khong dau>"

git add docs/
git commit -m "docs(thesis): <mo ta khong dau>"

git add PROGRESS.md memory/
git commit -m "docs(memory): cap nhat tien do sprint <ngay>"
```

**Type**: `feat` `fix` `tune` `sim` `docs` `refactor` `chore`
**Scope**: `models` `controllers` `adp-ft` `cl` `smc` `bs` `simulations` `plotting`
`thesis` `sm1` `sm2` `summary` `guides` `memory` `repo` `claude`

Mô tả **tiếng Việt KHÔNG dấu**, thể mệnh lệnh, ≤ 72 ký tự.
Có số liệu thì đưa vào thân bài commit.

Đặc tả đầy đủ: `docs/guides/commit-convention.md`

## Kiểm tra cuối

- [ ] `git status` sạch
- [ ] Không có file `.mat` nào bị commit (`git show --stat HEAD | grep -i mat`)
- [ ] Không có `.aux/.log/.out/.toc` nào bị commit
- [ ] `PROGRESS.md` có mục cho hôm nay
