# ADP Fixed-Time Optimal Control for Wheeled Mobile Robot

Luận văn Thạc sĩ — Điều khiển & Tự động hóa
Đại học Bách khoa Hà Nội | GVHD: PGS.TS. Nguyễn Hoài Nam
Học viên: Nguyễn Thành Trung

---

## Tổng quan

Mở rộng phương pháp **ADP-based Fixed-time Optimal Control** (Wang et al., IEEE RA-L 2025)
từ mô hình kinematic sang **dual-loop kinematic + dynamic**, tích hợp quán tính và ma sát
thực tế cho WMR kiểu vi sai.

### Bài báo gốc

> C. Wang, H. Zhan, Q. Guo, and T. Li, "Adaptive Dynamic Programming-Based Fixed-Time
> Optimal Control for Wheeled Mobile Robot," *IEEE Robotics and Automation Letters*,
> vol. 10, no. 1, pp. 176–183, Jan. 2025.
> → `references/Wang2025_ADP_FixedTime_WMR.pdf`

### Ranh giới: bài báo vs. luận văn

| | Wang et al. 2025 | Luận văn |
|---|---|---|
| Mô hình | Kinematic 3 trạng thái | **Full model 5 trạng thái** |
| Đầu ra | `u = [v, ω]` gán trực tiếp | **Mô-men `τ = [τ_R, τ_L]`** |
| Kiến trúc | 1 vòng | **Dual-loop** (ADP ngoài + dynamic BS trong) |
| Nhiễu | Không xét | **Sin tần số cao, 20% τ_max, 2 kênh** |
| So sánh | — | **5 phương pháp × 3 quỹ đạo + robustness** |

Luật ADP eq.(16)–(18) **giữ nguyên của Wang**, luận văn chỉ tune lại gain cho dual-loop.
Chi tiết ranh giới trích dẫn: [CLAUDE.md §2](CLAUDE.md).

### Đóng góp chính

| # | Vấn đề (từ SM1) | Giải pháp | Tài liệu |
|---|---|---|---|
| 1 | Actor-Critic cần PE → rung hệ thống | Critic-only + Concurrent Learning (bỏ PE) | SM2 |
| 2 | UUB → hội tụ chậm, không đảm bảo thời gian | Fixed-time convergence | Luận văn |
| 3 | Kinematic bỏ qua quán tính, ma sát | Dual-loop trên full model + nhiễu | Luận văn |

---

## Bắt đầu

**Yêu cầu:** MATLAB R2023a+, MiKTeX (để build LaTeX).

```powershell
$MATLAB = "C:\Program Files\MATLAB\R2023a\bin\matlab.exe"
$REPO   = "c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot"

# 1. Chạy mô phỏng chính (~5 phút) — sinh results/thesis_results.mat
& $MATLAB -batch "cd('$REPO\simulations'); sim_thesis"

# 2. Xuất figure PDF vào docs/
& $MATLAB -batch "cd('$REPO\plotting'); export_all_figures"

# 3. Build luận văn
$env:PATH = "C:\Users\LEGION\AppData\Local\Programs\MiKTeX\miktex\bin\x64;$env:PATH"
Set-Location "$REPO\docs\thesis"
pdflatex -interaction=nonstopmode thesis.tex; pdflatex -interaction=nonstopmode thesis.tex
```

> Script MATLAB dùng `addpath` tương đối — **bắt buộc `cd` vào đúng thư mục** trước khi chạy.
> `results/*.mat` không có trong git (80–300 MB), phải chạy sim để tái tạo.

---

## Cấu trúc thư mục

```
models/        Mô hình WMR: kinematic, dynamic, full 5-state, tham số robot
controllers/   5 bộ vòng ngoài (BS, ADP-AC, SMC, CL, ADP-FT) + 1 vòng trong (dynamic BS)
simulations/   Kịch bản mô phỏng + tham số controller + quỹ đạo tham chiếu
plotting/      Vẽ màn hình (plot_*) và xuất PDF vào docs/ (export_*)
results/       File .mat — KHÔNG commit, tái tạo bằng sim_*.m
docs/thesis/   Luận văn LaTeX (sản phẩm chính)
docs/reports/  Báo cáo seminar: sm1/, sm2/, summary/
docs/guides/   Quy ước repo
references/    Bài báo gốc + tài liệu tham khảo PDF
memory/        Bộ nhớ dài hạn của Claude Code
```

Đặc tả đầy đủ: [docs/guides/structure.md](docs/guides/structure.md)

---

## Tài liệu

| | |
|---|---|
| [CLAUDE.md](CLAUDE.md) | Hướng dẫn cho Claude Code — đọc trước khi nhờ AI sửa repo |
| [PROGRESS.md](PROGRESS.md) | Nhật ký tiến độ + bảng kết quả so sánh |
| [docs/guides/structure.md](docs/guides/structure.md) | Cấu trúc thư mục, quy tắc đặt file |
| [docs/guides/commit-convention.md](docs/guides/commit-convention.md) | Quy ước commit |
| [docs/guides/workflow.md](docs/guides/workflow.md) | Quy trình sim → figure → PDF |
| [results/README.md](results/README.md) | Cách tái tạo file `.mat` |

---

## Mô hình toán học

**Kinematic** (eq.1 bài gốc):

```
ẋ = v·cos(θ)      ẏ = v·sin(θ)      θ̇ = ω
```

**Dynamic** (Fierro & Lewis 1997):

```
M·η̇ = B·τ − F(η)

M = diag(m, I)
B = [r/2, r/2; r/(2L), −r/(2L)]
F(η) = [fv·v + fc·tanh(v/ε); fω·ω + fcω·tanh(ω/ε)]
```

**Controller ADP fixed-time** (eq.16–18 bài gốc):

```
u  = uf + uo
uf = [vr·cos(zθ), ωr]                                      feedforward
uo = ûo − g†(λ·tanh(z/ρ) + μz + αz^{p/q} + βz³)            feedback
ûo = −½ R⁻¹ gᵀ ∇φᵀ(z) Ŵ                                    critic-only ADP

Ŵ̇ = −½Γ(σ̄·ε + κ₁Ŵ + κ₂(ŴᵀŴ)Ŵ)                            adaptive update
```

**Dual-loop** (đóng góp luận văn):

```
Vòng ngoài (1 kHz): z → ADP Fixed-time → (v_d, ω_d)
Vòng trong (1 kHz): (v_d, ω_d) → Dynamic Backstepping → (τ_R, τ_L)
Plant:              τ + d(t) → wmr_full_model → [x, y, θ, v, ω]
```

---

## Tham số

**Robot** (`models/wmr_params.m`, Pioneer 3-DX):

| Tham số | Ký hiệu | Giá trị | Đơn vị |
|---|---|---|---|
| Khối lượng | m | 10 | kg |
| Mô-men quán tính | I | 0.5 | kg·m² |
| Bán kính bánh | r | 0.05 | m |
| Nửa khoảng cách trục | L | 0.15 | m |
| Ma sát nhớt | f_v / f_ω | 0.5 / 0.1 | N·s/m, N·m·s/rad |
| Ma sát Coulomb | f_c / f_cω | 0.05 / 0.02 | N, N·m |
| Mô-men max/bánh | τ_max | 20 | N·m |

**Controller ADP-FT** (`simulations/sm1_params.m` — **đã tune cho dual-loop**,
khác giá trị gốc của Wang dành cho kinematic-only):

```
p = 17, q = 19                      Q = diag(10,10,10),  R = diag(2,2)
φ(z) = [zx², zy², zθ², zx·zy, zy·zθ, zx·zθ]ᵀ    (l = 6)
Γ = 1·I₆                            κ₁ = 0.04,  κ₂ = 0.01
λ = μ = α = [0.08, 0.08, 0.05]      β = [0.15, 0.15, 0.08]
ρ = 0.1                             uo_max = 1.5   (clamp — bắt buộc)
Ŵ(0) = [3, 3, 3, 0, 0, 0]ᵀ          (warm start)
```

> Gain robust nhỏ hơn bài gốc 3–5×: vòng trong đã khử nhiễu, giữ gain gốc sẽ gây phản hồi
> dương và phát tán ở mức nhiễu ≥ 40%. Xem `memory/feedback_adp_ft_tuning.md`.

**Nhiễu:** `dist_amp = 0.2` (20% τ_max), `dist_freq = 30 rad/s` (~4.8 Hz), cộng vào cả 2 kênh mô-men.

---

## Kết quả

So sánh trên quỹ đạo tròn, T = 60 s (chi tiết trong [PROGRESS.md](PROGRESS.md)):

| Phương pháp | Jc (không nhiễu) | Jc (20% nhiễu) | z_rms(5s) |
|---|---|---|---|
| **ADP-FT** | **70.7** | **85.2** | 0.060 |
| BS | 70.7 | 90.1 | 0.000 |
| CL | 87.5 | 101.1 | 0.086 |
| ADP-AC | 107 | 121 | 0.136 |
| SMC | 475 | 556 | 0.276 |

Robustness theo mức nhiễu (Jc, quỹ đạo tròn):

| Nhiễu | BS | CL | ADP-FT |
|---|---|---|---|
| 0% | 70.7 | 87.5 | **70.7** |
| 20% | 90.1 | 101.1 | **85.2** |
| 40% | 116 | 124 | **112** |
| 60% | 157 | 162 | 158 |

---

## Tài liệu tham khảo

1. Wang et al. (2025) — ADP fixed-time WMR, IEEE RA-L *(`references/`)*
2. Fierro & Lewis (1997) — Mô hình dynamic WMR, backstepping 2 lớp
3. Vamvoudakis & Lewis (2010) — Actor-Critic ADP online
4. De La Cruz & Carelli (2008) — Mô hình dynamic WMR khi d ≠ 0
5. Polyakov (2012) — Lý thuyết fixed-time stabilization
