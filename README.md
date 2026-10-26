# ADP Fixed-Time Optimal Control for Wheeled Mobile Robot

Luận văn Thạc sĩ — Điều khiển & Tự động hóa
Đại học Bách khoa Hà Nội | GVHD: PGS.TS. Nguyễn Hoài Nam
Học viên: Nguyễn Thành Trung

---

## Tổng quan

Hoàn thiện phương pháp **ADP-based Fixed-time Optimal Control** (Wang et al., IEEE RA-L 2025)
thành **một bộ điều khiển backstepping hai bước** cho WMR vi sai nhận lệnh mô-men: bước 1 là
luật vận tốc ảo ADP-FT trên mô hình động học, bước 2 là luật mô-men trên mô hình động lực học
(quán tính, ma sát), có hàm Lyapunov chung cho toàn hệ.

### Bài báo gốc

> C. Wang, H. Zhan, Q. Guo, and T. Li, "Adaptive Dynamic Programming-Based Fixed-Time
> Optimal Control for Wheeled Mobile Robot," *IEEE Robotics and Automation Letters*,
> vol. 10, no. 1, pp. 176–183, Jan. 2025.
> → `references/Adaptive_Dynamic_Programming-Based_Fixed-Time_Optimal_Control_for_Wheeled_Mobile_Robot.pdf`

### Ranh giới: bài báo vs. luận văn

| | Wang et al. 2025 | Luận văn |
|---|---|---|
| Mô hình | Kinematic 3 trạng thái | **Full model 5 trạng thái** |
| Đầu ra | `u = [v, ω]` gán trực tiếp | **Mô-men `τ = [τ_R, τ_L]`** |
| Bộ điều khiển | Chỉ bước 1 (luật vận tốc) | **Backstepping hai bước**: bước 1 ADP-FT + bước 2 luật mô-men có số hạng chéo |
| Nhiễu | Không xét | **Sin tần số cao, 20% τ_max, cộng vào mô-men** |
| So sánh | — | **5 phương pháp × 3 quỹ đạo + robustness** |

Luật cập nhật eq.(17) và cấu trúc luật điều khiển eq.(16), (18) **giữ nguyên của Wang**; luận văn
chỉnh lại gain cho lệnh vận tốc bị chặn, bổ sung số hạng ghép nonholonomic (lấp khoảng hở kênh
ngang trong chứng minh gốc) + toán tử chiếu, và thêm bước 2 của backstepping.
Chi tiết ranh giới trích dẫn: [CLAUDE.md §2](CLAUDE.md).

### Đóng góp chính

| # | Đóng góp | Ghi chú |
|---|---|---|
| 1 | Bộ backstepping hai bước cho robot điều khiển bằng mô-men, **hàm Lyapunov chung** | Định lý 4.1, `dynamic_backstepping.m` |
| 2 | Phân tích lại chứng minh của Wang (**khoảng hở kênh ngang**, Bổ đề 4.1) + **ghép nonholonomic** (quyết định) + toán tử chiếu + tham số | Mục 4.2 luận văn |
| 3 | So sánh công bằng 5 luật bước 1 với cùng bước 2 (chỉnh gain đều nhau) | `sweep_gains.m` |
| 4 | Robustness + công thức dự đoán sai số do nhiễu + phân tích thành phần + đóng góp bước 2 | Mục 5.3–5.8 |

SM1/SM2 (kinematic): Actor-Critic cần PE → Critic-only + Concurrent Learning bỏ PE.

---

## Bắt đầu

**Yêu cầu:** MATLAB R2023a+, MiKTeX (để build LaTeX).

```powershell
$MATLAB = "C:\Program Files\MATLAB\R2023a\bin\matlab.exe"
$REPO   = "c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot"

# 1. Chạy mô phỏng chính (~5 phút) — sinh results/thesis_results.mat
& $MATLAB -batch "cd('$REPO\simulations'); sim_thesis"
#    Thí nghiệm bổ sung D–M (lâu hơn nhiều) và chỉnh gain đều nhau (tùy chọn)
& $MATLAB -batch "cd('$REPO\simulations'); sim_thesis_extra"
& $MATLAB -batch "cd('$REPO\simulations'); sweep_gains"

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
controllers/   5 luật bước 1 (BS, ADP-AC, SMC, CL, ADP-FT) + bước 2 (dynamic_backstepping.m)
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
B = (1/r)·[1, 1; L, −L]          lực F = (τR+τL)/r, mô-men N = L(τR−τL)/r
F(η) = [fv·v + fc·tanh(v/ε); fω·ω + fcω·tanh(ω/ε)]
```

> Trước 2026-10-26 code dùng nhầm `B = [r/2, r/2; r/(2L), −r/(2L)]` — đó là Jacobian
> động học (vận tốc bánh → v, ω), không phải ma trận mô-men. Xem
> `memory/feedback_b_matrix_error.md`.

**Controller ADP fixed-time** (eq.16–18 bài gốc):

```
u  = uf + uo
uf = [vr·cos(zθ), ωr]                                      feedforward
uo = ûo − g†(λ·tanh(z/ρ) + μz + αz^{p/q} + βz³)            feedback
ûo = −½ R⁻¹ gᵀ ∇φᵀ(z) Ŵ                                    critic-only ADP

Ŵ̇ = ½Γ(∇φ·g·R⁻¹·gᵀ·z − κ₁Ŵ − κ₂(ŴᵀŴ)Ŵ)                  eq.(17), mặc định
```

(Luận văn bổ sung `+ c_zy·v_r·tanh(z_y/ς)` vào thành phần θ của vector robust.)

**Backstepping hai bước** (đóng góp luận văn, Mục 4.3–4.4):

```
Bước 1: η_d = u_f + u_o(z, Ŵ)                              (ADP-FT hoặc luật so sánh)
Bước 2: B·τ = M(η̇_d + K_d·e + K_α·sig(e)^{p/q} + K_β·e³) + F(η) + K_c·[z_x; z_θ],  e = η_d − η
V = ½zᵀz + ½W̃ᵀΓ⁻¹W̃ + ½eᵀK_c⁻¹M·e   →   V̇ = V̇₁|η=η_d − (tiêu tán theo e) − eᵀK_c⁻¹B·d
```

Số hạng chéo `K_c·[z_x; z_θ] = −K_c·gᵀz` triệt số hạng tương tác trong `V̇₁`; đặt
`K_c = K_α = K_β = 0` thì trở về luật computed-torque của Fierro & Lewis.

---

## Tham số

**Robot** (`models/wmr_params.m`):

| Tham số | Ký hiệu | Giá trị | Đơn vị |
|---|---|---|---|
| Khối lượng | m | 10 | kg |
| Mô-men quán tính | I | 0.5 | kg·m² |
| Bán kính bánh | r | 0.05 | m |
| Nửa khoảng cách trục | L | 0.15 | m |
| Ma sát nhớt | f_v / f_ω | 0.5 / 0.1 | N·s/m, N·m·s/rad |
| Ma sát Coulomb | f_c / f_cω | 0.05 / 0.02 | N, N·m |
| Mô-men max/bánh | τ_max | 1 (tạm, chờ thông số motor thật) | N·m |

**Bước 1 ADP-FT** (`simulations/ctrl_params.m` — **đã chỉnh cho robot điều khiển bằng mô-men**,
khác giá trị gốc của Wang dành cho kinematic-only: λ = μ = α = diag(1, 1, 0.4),
β = diag(1e5, 1e5, 1), ρ = 10, Γ = 2I, κ₁ = κ₂ = 0.1, Ŵ(0) = 0):

```
p = 17, q = 19                      Q = diag(10,10,10),  R = diag(2,2)
φ(z) = [zx², zy², zθ², zx·zy, zy·zθ, zx·zθ]ᵀ    (l = 6)
Γ = 1·I₆                            κ₁ = 0.04,  κ₂ = 0.01
λ = μ = α = [0.04, 0.04, 0.025]     β = [0.075, 0.075, 0.04]
ρ = 0.1                             uo_max = 1.5   (clamp)
Ŵ(0) = [1.5, 1.5, 1.5, 0, 0, 0]ᵀ    (warm start)
c_zy = 1.0, ς = 0.2                 (ghép nonholonomic)   luật cập nhật: 'wang' (eq.17)
```

**Bước 2** (dùng chung cho mọi phương pháp): `K_d = 20·I`, `K_c = M = diag(10, 0.5)`
(→ `V = V₁ + ½eᵀe`), `K_α = K_β = 10·I` (T₂ ≤ 1 s khi chưa bão hòa), `p/q = 17/19`.

> Gain của mọi phương pháp chọn bằng `sweep_gains.m`: nhân toàn bộ gain danh nghĩa với
> cùng hệ số k ∈ {1/16 … 4}, lấy k* cho tổng Jc 3 quỹ đạo nhỏ nhất (với bước 2 đầy đủ:
> ADP-FT k* = 1/2, SMC 1/8, BS 1/4, CL 1/4, ADP-AC 1/4).

**Nhiễu:** `dist_amp = 0.2` (20% τ_max), `dist_freq = 30 rad/s` (~4.8 Hz), cộng cùng pha
vào hai bánh — chỉ tác động kênh v (`B·[1;1] = [2/r; 0]`). Kịch bản nhiễu độc lập hai
kênh: Phần F của `sim_thesis_extra.m`.

---

## Kết quả

> ⚠ Bảng dưới là số liệu với **bước 2 cũ** (luật Fierro & Lewis). Đang chạy lại với bước 2
> đầy đủ; sẽ cập nhật cùng Chương 5–6 (PR #1).

Mô hình đã sửa ma trận B, τ_max = 1 N·m (tạm), T = 60 s, nhiễu đồng pha 20%, gain chỉnh
đều nhau (chạy bằng Octave 2026-10-26; chi tiết trong [PROGRESS.md](PROGRESS.md) và Chương 5
luận văn):

| Phương pháp | Jc tròn | Jc thẳng | Jc hình 8 | Tổng | t(‖z‖ < 0.01) tròn |
|---|---|---|---|---|---|
| SMC (k* = 1/8) | **1.213** | **0.663** | **1.960** | **3.836** | 10.0 s |
| ADP-FT | 1.334 | 0.803 | 2.056 | 4.192 | **8.4 s** |
| NTSMC (cùng gain SMC) | 1.257 | 0.857 | 2.079 | 4.193 | 9.9 s |
| BS | 1.307 | 0.898 | 2.104 | 4.310 | 10.7 s |
| CL | 1.598 | 1.347 | 2.832 | 5.778 | 25.6 s |
| ADP-AC | 4.708 | 4.219 | 5.407 | 14.334 | 60.0 s |

- Sai số xác lập ≈ 5.3–5.5e-4 m ở BS/SMC/NTSMC/ADP-FT = sàn do nhiễu lọt qua vòng trong;
  công thức giải tích z_rms ≈ 2d₀/(√2·r·m·ω_n·√(ω_n²+K_d²)) khớp mô phỏng < 4%.
- Khối lượng tới 16 kg và nhiễu tới 60% (ba kiểu): Jc đổi < 4% ở mọi phương pháp.
- Ablation ADP-FT: ghép nonholonomic là quyết định (thiếu nó Jc ×3–5, kẹt ở điểm cân bằng
  giả); toán tử chiếu không kích hoạt; bỏ số hạng fixed-time giảm Jc 1.5–2%; critic đóng
  băng tốt hơn critic học 3% (danh nghĩa) và hội tụ nhanh ~3 lần khi ‖z0‖ = 3.5.
- Fixed-time không được bảo toàn: với |v| ≤ 0.5 m/s, thời gian quá độ buộc tăng theo ‖z0‖.

---

## Tài liệu tham khảo

1. Wang et al. (2025) — ADP fixed-time WMR, IEEE RA-L *(`references/`)*
2. Fierro & Lewis (1997) — Mô hình dynamic WMR, backstepping 2 lớp
3. Krstić, Kanellakopoulos & Kokotović (1995) — Thủ tục backstepping
4. Vamvoudakis & Lewis (2010) — Actor-Critic ADP online
5. De La Cruz & Carelli (2008) — Mô hình dynamic WMR khi d ≠ 0
6. Polyakov (2012) — Lý thuyết fixed-time stabilization
