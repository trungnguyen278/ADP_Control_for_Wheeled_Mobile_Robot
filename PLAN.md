# PLAN — Implementation Sprint 2026-05-06

## Muc tieu
Implement 2 controller con thieu (ADP Fixed-time, Critic-only CL) va tich hop vao
simulation so sanh 5 phuong phap tren full model 5 state.

## Kich ban so sanh
{BS, ADP Actor-Critic, SMC, Critic-only CL, ADP Fixed-time} x {no dist, dist} = 10 runs
- Quy dao: circle, R=2m, vr=0.3 m/s, T=60s
- Nhieu: d(t) = 0.2*tau_max*sin(30t), cong vao tau (vong trong)
- Kien truc: 2 vong (kinematic outer + dynamic_backstepping inner)

## Tasks va trang thai

### [DONE] 1. Setup memory + plan
- Luu advisor feedback, project progress, MEMORY.md, PLAN.md
- Files: memory/feedback_advisor_nam.md, memory/project_progress.md

### [DONE] 2. Implement adp_fixed_time.m
- Wang et al. eq.16-18: Critic-only, fixed-time robust terms
- Interface: [u, W, info] = adp_fixed_time(q, qr, vr, omegar, W, s)
- W update: model-based (khong dung Bellman error)
- Robust terms: lambda*tanh + mu*z + alpha*z^{p/q} + beta*z^3
- W0 = 0 (robust terms dam bao on dinh ban dau)
- File: controllers/adp_fixed_time.m (97 dong)

### [DONE] 3. Implement critic_only_cl.m
- SM2: Critic-only + Concurrent Learning
- Interface: [u, cl, info] = critic_only_cl(t, q, qr, vr, omegar, cl, s)
- CL: luu sigma_stack + cost_stack, vectorized update
- W0 = [3;3;3;0;0;0] warm start (can feedback ban dau)
- File: controllers/critic_only_cl.m (108 dong)

### [DONE] 4. Update sm1_params.m
- Them section ADP Fixed-time: ft_p, ft_q, ft_Gamma, ft_kappa1/2, ft_lambda/mu/alpha/beta, ft_rho, ft_W0
- Them section Critic-only CL: cl_alpha, cl_alpha_hist, cl_kappa, cl_stack_max, cl_record_dt, cl_W0, cl_uo_max
- File: simulations/sm1_params.m (+30 dong)

### [DONE] 5. Update sim_sm1_full.m
- methods = {'bs', 'adp', 'smc', 'cl', 'adp_ft'} (5 methods x 2 conditions = 10 runs)
- Them switch cases cho 'cl' va 'adp_ft'
- Pre-allocate data.W cho CL va ADP-FT
- Khoi tao cl struct (W, sigma_stack, cost_stack, stack_count)
- uo tinh nhat quan: eta_d - [vr*cos(zth); omegar] cho tat ca methods
- File: simulations/sim_sm1_full.m (rewrite, 194 dong)

### [DONE] 6. Update plot_sm1_full.m
- 5 mau: BS=red, ADP=blue, SMC=green, CL=magenta, ADP-FT=orange
- 6 figures: XY(2), Error(2), Torque+Dist, Weight evolution
- Weight plot: 3 subplot (ADP-AC Wc, CL W, ADP-FT W)
- File: plotting/plot_sm1_full.m (rewrite, 162 dong)

### [DONE] 7. Review code consistency
- Matrix dimensions: nabla_phi(6x3), g_z(3x2), g_pinv(2x3), R_inv(2x2) — OK
- z luon defined truoc khi dung sau switch block — OK
- Param names khop giua sm1_params va controllers — OK
- cl_uo_max (1.5) rieng voi uo_max (1.0) la thiet ke co chu dich

## Ghi chu ky thuat
- g(z) trong Wang eq.5c KHAC B trong dynamic model (chu y ky hieu)
- g_pinv = (g'g)^{-1}g', det(g'g) = zx^2+1 > 0 luon ton tai
- beta = [1e5; 1e5; 1]: aggressive, se saturate khi z > 0.01, nhung dam bao fixed-time
- CL vectorized: S_bar * deltas' thay loop, nhanh hon ~100x trong MATLAB
- ADP-FT khong clamp uo vi robust terms can lon, chi clamp u cuoi cung

## Viec tiep theo (chua lam)
- [ ] Chay sim_sm1_full.m trong MATLAB, kiem tra ket qua
- [ ] Tune tham so neu can (dac biet beta, cl_alpha)
- [ ] Implement sim_sm2.m (SM2 kinematic-only rieng)
- [ ] Implement sim_thesis.m (robustness: m=15kg, nhieu lon hon)
- [ ] Viet bao cao SM2
- [ ] Viet luan van
