# Cấu trúc thư mục

Đặc tả chuẩn cho repo. Khi thêm file mới, đặt đúng chỗ theo bảng dưới.

---

## Sơ đồ tổng thể

```
ADP_Control_for_Wheeled_Mobile_Robot/
│
├── CLAUDE.md                   Hướng dẫn cho Claude Code
├── README.md                   Giới thiệu dự án (cho người đọc)
├── PROGRESS.md                 Nhật ký tiến độ (cập nhật liên tục)
├── .gitignore
│
├── models/                     ── TẦNG 1: Mô hình đối tượng ──
│   ├── wmr_params.m            Tham số robot (Pioneer 3-DX)
│   ├── wmr_kinematics.m        ODE kinematic:  [ẋ, ẏ, θ̇]
│   ├── wmr_dynamics.m          ODE dynamic:    [v̇, ω̇]
│   └── wmr_full_model.m        ODE ghép 5 trạng thái: [ẋ, ẏ, θ̇, v̇, ω̇]
│
├── controllers/                ── TẦNG 2: Bộ điều khiển ──
│   │  ── Vòng ngoài (kinematic): q → η_d = [v_d, ω_d] ──
│   ├── backstepping_controller.m   Backstepping (baseline)
│   ├── actor_critic_adp.m          SM1: Actor-Critic ADP (2 mạng, cần PE)
│   ├── smc_kinematic.m             SMC thường (baseline theo yêu cầu GVHD)
│   ├── critic_only_cl.m            SM2: Critic-only + Concurrent Learning
│   ├── adp_fixed_time.m            LUẬN VĂN: ADP fixed-time (Wang eq.16-18)
│   │  ── Vòng trong (dynamic): η_d → τ = [τ_R, τ_L] ──
│   └── dynamic_backstepping.m      Backstepping bậc 2 (Fierro & Lewis 1997)
│
├── simulations/                ── TẦNG 3: Kịch bản mô phỏng ──
│   ├── sm1_params.m            Tham số controller + mô phỏng (dùng chung)
│   ├── ref_trajectory.m        Sinh quỹ đạo: circle | line | figure8
│   ├── test_openloop.m         Kiểm tra mô hình vòng hở (4 test)
│   ├── sim_sm1.m               SM1: AC vs BS, kinematic-only
│   ├── sim_sm1_full.m          5 phương pháp × 2 điều kiện, full model
│   ├── sim_sm2.m               SM2: BS vs AC+PE vs AC-PE vs CL, kinematic
│   └── sim_thesis.m            LUẬN VĂN: 3 quỹ đạo + robustness khối lượng/nhiễu
│
├── plotting/                   ── TẦNG 4: Vẽ & xuất hình ──
│   ├── plot_sm1_results.m      Vẽ trên màn hình (SM1)
│   ├── plot_sm1_full.m         Vẽ trên màn hình (full model)
│   ├── plot_sm2.m              Vẽ trên màn hình (SM2)
│   ├── plot_thesis.m           Vẽ trên màn hình (luận văn)
│   ├── export_sm1_figures.m    → docs/reports/sm1/figures/*.pdf
│   └── export_all_figures.m    → docs/reports/sm2/figures/ + docs/thesis/figures/
│
├── results/                    ── Dữ liệu sinh ra (KHÔNG commit) ──
│   ├── README.md               Bảng: file nào sinh từ script nào
│   └── *.mat                   80–300 MB mỗi file, bị .gitignore chặn
│
├── docs/                       ── Tài liệu viết ──
│   ├── guides/                 Quy ước repo (file này)
│   │   ├── structure.md
│   │   ├── commit-convention.md
│   │   └── workflow.md
│   ├── thesis/                 LUẬN VĂN (sản phẩm chính)
│   │   ├── thesis.tex
│   │   ├── thesis.pdf
│   │   └── figures/            PDF từ MATLAB + PNG (logo, ảnh minh họa)
│   └── reports/                Báo cáo seminar
│       ├── sm1/                Seminar 1 — Actor-Critic
│       ├── sm2/                Seminar 2 — Critic-only + CL
│       └── summary/            Báo cáo tóm tắt nộp GVHD
│
├── references/                 ── Tài liệu tham khảo PDF ──
│   └── Wang2025_ADP_FixedTime_WMR.pdf    Bài báo gốc
│
├── memory/                     ── Bộ nhớ dài hạn Claude (junction) ──
│   ├── MEMORY.md               Chỉ mục
│   └── *.md                    Mỗi file = 1 memory
│
└── .claude/
    ├── settings.json           Permission dùng chung (commit)
    ├── settings.local.json     Permission cá nhân (gitignore)
    └── skills/                 Skill riêng của dự án
```

---

## Nguyên tắc đặt file

| Loại file | Đặt ở | Ghi chú |
|---|---|---|
| Mô hình toán học của robot | `models/` | Chỉ ODE thuần, không có logic điều khiển |
| Luật điều khiển | `controllers/` | Hàm thuần: vào trạng thái → ra tín hiệu điều khiển |
| Kịch bản chạy | `simulations/` | Script (không phải hàm), tự `save` vào `results/` |
| Vẽ hình | `plotting/` | `plot_*` vẽ màn hình, `export_*` ghi PDF vào `docs/` |
| Tham số | `models/wmr_params.m` (robot), `simulations/sm1_params.m` (controller) | Không hardcode số trong script mô phỏng |
| PDF tham khảo | `references/` | Không để trong `docs/` |

### Quy tắc đặt tên

- File và biến: `snake_case`, không dấu, tiếng Anh.
- Controller: `<phuong_phap>[_<pham_vi>].m` — vd `smc_kinematic.m`, `adp_fixed_time.m`.
- Script mô phỏng: `sim_<muc_dich>.m`. Script test: `test_<doi_tuong>.m`.
- Kết quả: `results/<ten_sim>_results.mat` — trùng tên với script sinh ra nó.

---

## Ràng buộc kỹ thuật (đừng phá)

1. **`addpath` là đường dẫn tương đối cứng.** Mọi script trong `simulations/` và
   `plotting/` đều gọi `addpath('../models', '../controllers', ...)`.
   → Chúng **chỉ chạy được khi `cd` vào đúng thư mục chứa chúng**.
   → Không được đổi tên hay di chuyển 4 thư mục code.

2. **`export_*.m` ghi thẳng vào `docs/`.** Nếu đổi cấu trúc `docs/`, phải sửa biến
   `fig_dir` trong `plotting/export_sm1_figures.m` và `plotting/export_all_figures.m`.

3. **`.tex` dùng `\graphicspath{{figures/}}`.** Hình phải nằm trong thư mục `figures/`
   cùng cấp với file `.tex`. Di chuyển thư mục báo cáo thì hình đi theo, không cần sửa `.tex`.

---

## Nợ kỹ thuật đã biết

- `plotting/plot_sm1_results.m` và `plot_sm1_full.m` trùng lặp nhiều đoạn vẽ.
- `simulations/sm1_params.m` giữ tham số cho **tất cả** phương pháp (kể cả luận văn),
  tên file không còn phản ánh phạm vi — cân nhắc đổi thành `ctrl_params.m`.
