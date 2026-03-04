# ADP Fixed-Time Optimal Control for Wheeled Mobile Robot

Luận văn Thạc sĩ — Điều khiển & Tự động hóa  
Đại học Bách khoa Hà Nội | GVHD: PGS.TS. Nguyễn Hoài Nam

## Tổng quan

Mở rộng phương pháp **ADP-based Fixed-time Optimal Control** (Wang et al., IEEE RA-L 2025) từ mô hình kinematic sang **dual-loop kinematic + dynamic**, tích hợp quán tính và ma sát thực tế cho WMR kiểu vi sai.

### Bài báo gốc

> C. Wang, H. Zhan, Q. Guo, and T. Li, "Adaptive Dynamic Programming-Based Fixed-Time Optimal Control for Wheeled Mobile Robot," *IEEE Robotics and Automation Letters*, vol. 10, no. 1, pp. 176–183, Jan. 2025.

**Bài gốc chỉ dùng kinematic** (u = [v, ω] gán trực tiếp). Luận văn bổ sung vòng dynamic để output là mô-men τ = [τ_R, τ_L], sát thực tế hơn.

### Đóng góp chính

| # | Vấn đề (từ SM1) | Giải pháp | Tài liệu |
|---|-----------------|-----------|-----------|
| 1 | Actor-Critic cần PE → rung hệ thống | Critic-only + Adaptive update law (không cần PE) | SM2, Luận văn |
| 2 | UUB → hội tụ chậm, không đảm bảo thời gian | Fixed-time convergence (κ₁Ŵ + κ₂Ŵ³, αz^{p/q} + βz³) | Luận văn |
| 3 | Mô hình kinematic bỏ qua quán tính, ma sát | Dual-loop: ADP (ngoài) + PI robust (trong) trên dynamic model | Luận văn |

## Cấu trúc thư mục

```
thesis_project/
├── models/                     # Mô hình WMR
│   ├── wmr_params.m            # Tham số robot (Pioneer 3-DX)
│   ├── wmr_kinematics.m        # ODE kinematic: [ẋ, ẏ, θ̇]
│   ├── wmr_dynamics.m          # ODE dynamic:   [v̇, ω̇]
│   └── wmr_full_model.m        # ODE ghép:      [ẋ, ẏ, θ̇, v̇, ω̇]
├── controllers/                # Bộ điều khiển
│   ├── actor_critic_adp.m      # SM1: Actor-Critic (baseline)
│   ├── critic_only_cl.m        # SM2: Critic-only + Concurrent Learning
│   ├── adp_fixed_time.m        # Luận văn: vòng ngoài (eq. 16-18)
│   └── inner_loop_pi.m         # Luận văn: vòng trong (PI robust)
├── simulations/                # Script chạy mô phỏng
│   ├── test_openloop.m         # Test mô hình vòng hở
│   ├── sim_sm1.m               # Mô phỏng SM1
│   ├── sim_sm2.m               # Mô phỏng SM2
│   └── sim_thesis.m            # Mô phỏng luận văn (2 vòng)
├── plotting/                   # Script vẽ hình
├── results/                    # Dữ liệu .mat, hình .fig/.png
├── docs/                       # Tài liệu
│   └── Wang2025_ADP_FixedTime_WMR.pdf
└── README.md
```

## Mô hình toán học

### Kinematic (đã có — eq. 1 bài gốc)

```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
θ̇ = ω
```

### Dynamic (bổ sung — theo Fierro & Lewis 1997)

```
M·η̇ = B·τ − F(η)

M = diag(m, I)
B = [r/2, r/2; r/(2L), -r/(2L)]
F(η) = [fv·v + fc·tanh(v/ε); fω·ω + fcω·tanh(ω/ε)]
```

### Controller (eq. 16-18 bài gốc)

```
u = uf + uo
uf = [vr·cos(zθ), ωr]                          % feedforward
uo = ûo − B†(λ·tanh(z/ρ) + μz + αz^{p/q} + βz³)  % feedback
ûo = −½ R⁻¹ Bᵀ ∇φᵀ(z) Ŵ                        % critic-only ADP

Ŵ̇ = ½Γ(∇φ·B·R⁻¹·Bᵀ·z − κ₁Ŵ − κ₂(ŴᵀŴ)Ŵ)    % adaptive update
```

### Dual-loop (đóng góp luận văn)

```
Vòng ngoài: z → ADP Fixed-time → (v_d, ω_d)
Vòng trong: (v_d−v, ω_d−ω) → PI robust → (τ_R, τ_L)
```

## Tham số robot

| Tham số | Ký hiệu | Giá trị | Đơn vị |
|---------|---------|---------|--------|
| Khối lượng | m | 10 | kg |
| Mô-men quán tính | I | 0.5 | kg·m² |
| Bán kính bánh | r | 0.05 | m |
| Nửa khoảng cách trục | L | 0.15 | m |
| Ma sát nhớt (v) | f_v | 0.5 | N·s/m |
| Ma sát nhớt (ω) | f_ω | 0.1 | N·m·s/rad |
| Ma sát Coulomb | f_c | 0.3 | N |
| Mô-men max/bánh | τ_max | 5 | N·m |

## Tham số controller (từ bài gốc, Section IV)

```
p = 17, q = 19
Q = 10·I₃, R = 2·I₃
φ(z) = [zx², zy², zθ², zx·zy, zy·zθ, zx·zθ]ᵀ   (l = 6)
Γ = 2·I₆, κ₁ = κ₂ = 0.1
λ = μ = α = diag(1, 1, 0.4)
β = diag(1e5, 1e5, 1), ρ = 10
Ŵ(0) = 0₆ₓ₁
```

## Tiến độ

- [x] Tuần 1: Mô hình dynamic WMR
- [ ] Tuần 2–3: Mô phỏng Actor-Critic (SM1)
- [ ] Tuần 4: Hoàn thiện SM1
- [ ] Tuần 5–7: Mô phỏng Critic-only + CL (SM2)
- [ ] Tuần 8: Hoàn thiện SM1 + SM2
- [ ] Tuần 9–11: Thiết kế + tích hợp 2 vòng
- [ ] Tuần 12–13: Mô phỏng + viết luận văn
- [ ] Tuần 14–16: Hoàn thiện + bảo vệ

## Tài liệu tham khảo chính

1. Wang et al. (2025) — Bài gốc, ADP fixed-time WMR *(trong `docs/`)*
2. Fierro & Lewis (1997) — Mô hình dynamic WMR, backstepping 2 lớp
3. Vamvoudakis & Lewis (2010) — Actor-Critic ADP online
4. Polyakov (2012) — Fixed-time stabilization theory

## Yêu cầu

- MATLAB R2022b+ (hoặc tương đương)
- Simulink (cho luận văn, tùy chọn)

## Tác giả

Nguyễn Thành Trung — Cao học Điều khiển & Tự động hóa, ĐHBK Hà Nội
