# TIEN DO LUAN VAN — ADP Fixed-Time Optimal Control for WMR

> **QUY TAC: Cap nhat file nay MOI KHI hoan thanh cong viec. Ghi ngay, noi dung, ket qua.**

De tai: Mo rong ADP-based Fixed-time Optimal Control (Wang et al., IEEE RA-L 2025)
tu kinematic-only sang dual-loop kinematic + dynamic cho WMR vi sai.

GVHD: PGS.TS. Nguyen Hoai Nam
Hoc vien: Nguyen Thanh Trung
Thoi gian: 16 tuan (~02/2026 - 06/2026)

---

## KE HOACH GOC (16 TUAN)

| Tuan | Noi dung | Trang thai |
|------|----------|------------|
| 1 | Mo hinh Dynamic WMR (models/*.m, test_openloop.m) | DA XONG |
| 2-3 | SM1: Actor-Critic ADP kinematic (2 mang, can PE) | DA XONG |
| 4 | Hoan thien SM1, viet bao cao | DA XONG |
| 5-7 | SM2: Critic-only + Concurrent Learning (1 mang, bo PE) | DA XONG (code) |
| 8 | Hoan thien SM1 + SM2 | DA XONG (code) |
| 9-11 | Luan van: ADP Fixed-time + Dual-loop (Wang et al.) | DA XONG (code) |
| 12-13 | Mo phong tong hop + Viet luan van | DA XONG (code) |
| 14-16 | Hoan thien + Bao ve | CHUA LAM |

> **Ghi chu**: Sau bao cao SM1 (2026-03-28), thay Nam yeu cau thay doi kien truc:
> phai co 2 vong (kinematic + dynamic), them disturbance, them SMC baseline.
> Ke hoach duoc dieu chinh: gop SM2 code + full model comparison vao sprint 2026-05-06,
> doi viec rieng sim_sm2.m sang sau.

---

## KIEN TRUC HE THONG

```
Vong ngoai (kinematic):     Vong trong (dynamic):     Plant:
  1. Backstepping       -+
  2. ADP Actor-Critic   -|   Dynamic Backstepping     wmr_full_model
  3. SMC                -+-> (Fierro & Lewis)     --> (5 state)
  4. Critic-only CL     -|   tau = B_inv*(M*(...)+F)    + d(t)
  5. ADP Fixed-time     -+
```

---

## LICH SU TIEN DO

### Tuan 1-3 (02-03/2026) — Nen tang
- [x] Mo hinh dynamic WMR 5 state (models/*.m) [ad88436]
- [x] SM1 Actor-Critic ADP kinematic (sim_sm1.m) [4bf357c]
- [x] Bao cao SM1 (LaTeX + figures) [e63ec30]

### Sprint 2026-05-06 — Mo rong full model
- [x] dynamic_backstepping.m: vong trong Fierro & Lewis
- [x] smc_kinematic.m: SMC vong ngoai (them c_zy coupling zy)
- [x] adp_fixed_time.m: controller chinh luan van (Wang et al. eq.16-18)
- [x] critic_only_cl.m: SM2 Critic-only + Concurrent Learning
- [x] sim_sm1_full.m: 5 methods x 2 conditions = 10 kich ban
- [x] plot_sm1_full.m: 6 figures

### 2026-05-11 — Tune tham so + chay MATLAB
- [x] ADP-FT: sua weight update (Bellman error gradient), warm start, giam robust gains
  - ft_beta: 1e5 -> 1, ft_rho: 10 -> 0.05, kappa: 0.1 -> 0.02
  - Jc: 4385 -> 123 (no dist), 4446 -> 152 (dist)
- [x] SMC: them c_zy=1.0 vao mat truot sigma_2
  - z_rms(no dist): 0.82 -> 0.28
- [x] Commit [d962d87]

### 2026-05-25 — SM2 + Thesis simulations
- [x] sim_sm2.m: kinematic-only so sanh BS vs AC+PE vs AC-PE vs CL (2 quy dao)
- [x] plot_sm2.m: 7 figures (XY, error, weights, Bellman error, running cost)
- [x] sim_thesis.m: full model robustness analysis
  - Phan A: 3 quy dao (circle, line, figure8) x 5 phuong phap
  - Phan B: 3 khoi luong (10, 15, 20 kg) x 3 pp (BS, CL, ADP-FT)
  - Phan C: 4 muc nhieu (0, 20, 40, 60%) x 3 pp
- [x] plot_thesis.m: 12 figures tong hop
- [x] ref_trajectory.m: them quy dao figure-8 (lemniscate)
- [x] sm1_params.m: them tham so fig8_a, fig8_b, fig8_omega

### 2026-05-25 — Bao cao + Luan van
- [x] export_all_figures.m: xuat 19 figures PDF tu .mat data
  - 6 SM2: sm2_xy_circle/line, sm2_error_circle, sm2_weights, sm2_bellman, sm2_cost
  - 12 thesis: xy/error x3 traj, bar_jc_traj, weights_adp_ft, robust_mass/dist, bar_jc_mass/dist
  - 1 torque: torque_dist
- [x] docs/sm2_report/sm2_report.tex: bao cao SM2 (CL thay PE, 7 sections, tieng Viet co dau)
- [x] docs/summary/summary_report.tex: bao cao tom tat cho thay (tieng Viet co dau)
- [x] docs/thesis/thesis.tex: luan van 6 chuong (tieng Viet co dau)
- [x] Compile LaTeX → PDF: thesis 36pp, summary 13pp, sm2_report 15pp

### 2026-05-26 — Cai thien luan van v2
- [x] Them logo truong (logoBK.png) + vien tikz trang bia
- [x] Them 8 hinh minh hoa TikZ: WMR schematic, body frame error, convergence comparison,
  ADP structure, CL concept, dual-loop architecture, ADP-FT signal flow, reference trajectories
- [x] Them hinh AGV (agv_example.png) Chuong 1
- [x] Mo rong noi dung: bang tham so robot, so sanh AC vs Critic-only, bang tong hop 5 PP,
  khuyen nghi su dung, phan tich mo-men chi tiet, ket luan mo rong
- [x] Compile → thesis 43pp (tang 7 trang, 20 hinh)

### 2026-08-12 — Tai cau truc repo
- [x] Sua loi git nghiem trong: results/*.mat (565MB, file lon nhat 300MB) bi commit
  -> GitHub chan cung file >100MB, commit 2b6cfef khong the push duoc
  -> reset --soft commit chua push, them .gitignore, untrack .mat + LaTeX artifacts
  -> .git: 578MB -> ~280MB, push len GitHub chay lai duoc
- [x] Sap xep lai thu muc:
  - references/ (moi): tach bai bao goc ra khoi docs/
  - docs/reports/{sm1,sm2,summary}/: gom bao cao seminar
  - docs/guides/ (moi): structure.md, commit-convention.md, workflow.md
  - Sua fig_dir trong plotting/export_sm1_figures.m va export_all_figures.m
- [x] Ho tro Claude Code:
  - CLAUDE.md: ranh gioi bai bao vs luan van, quy tac ngon ngu, quy tac git/LaTeX
  - .claude/settings.json: permission dung chung (settings.local.json -> gitignore)
  - .claude/skills/: run-sim, build-doc, checkpoint
  - memory/: junction tu ~/.claude/projects/<slug>/memory -> repo memory/
    (truoc do bi tach doi, MEMORY.md global co link tuong doi bi hong)
  - Chuan hoa frontmatter 4 memory cu (type: -> metadata.type:)
- [x] Sua .vscode/settings.json: files.associations "*.m" -> matlab
  (truoc do VSCode parse .m nhu Objective-C, bao hang tram loi gia)
- [x] Cap nhat README.md: cau truc thu muc dung thuc te, tham so ADP-FT da tune
  (README cu con ghi gain goc cua Wang: beta=1e5, rho=10, tau_max=5, fc=0.3)
- [x] results/README.md: bang tai tao .mat tu script nao

**Ket qua so sanh hien tai (circle, T=60s):**

| Method | Jc (no dist) | Jc (dist) | z_rms(5s) no dist |
|--------|-------------|-----------|-------------------|
| ADP-FT | **70.7**    | **85.2**  | 0.060             |
| BS     | 70.7        | 90.1      | 0.000             |
| CL     | 87.5        | 101.1     | 0.086             |
| ADP-AC | 107         | 121       | 0.136             |
| SMC    | 475         | 556       | 0.276             |

**Robustness nhieu (circle, ADP-FT vs BS):**

| Dist | BS   | CL    | ADP-FT   |
|------|------|-------|----------|
| 0%   | 70.7 | 87.5  | **70.7** |
| 20%  | 90.1 | 101.1 | **85.2** |
| 40%  | 116  | 124   | **112**  |
| 60%  | 157  | 162   | **158**  |

---

## VIEC CAN LAM (uu tien giam dan)

### Uu tien cao
- [x] sim_sm2.m: mo phong rieng SM2 (kinematic-only, AC vs CL) -- XONG
- [x] plot_sm2.m: 7 figures (XY, error, weights, Bellman, cost)
- [x] Viet bao cao SM2 (docs/sm2_report/sm2_report.tex)
- [x] export_all_figures.m: 19 figures PDF (6 SM2 + 12 thesis + 1 torque)

### Uu tien trung binh
- [x] sim_thesis.m: robustness analysis (3 quy dao, 3 mass, 4 dist_amp)
- [x] plot_thesis.m: 12 figures (A: 3 traj, B: mass, C: dist)
- [x] ref_trajectory.m: them quy dao figure-8
- [x] sm1_params.m: them tham so figure-8
- [x] Viet luan van ban nhap (docs/thesis/thesis.tex, 5 chuong)
- [x] Viet bao cao tom tat cho thay (docs/summary/summary_report.tex)
- [ ] Cai thien SMC (z_rms con cao, xem xet NTSMC)

### Uu tien thap (cuoi ky)
- [x] Compile LaTeX sang PDF (MiKTeX, pdflatex x2 cho cross-ref) -- XONG 2026-05-25
- [ ] Chuan bi slide bao ve

---

## YEU CAU TU THAY NAM (2026-03-28)

1. Disturbance dang sin, tan so cao, bien do 20% tau_max --> DA LAM
2. Disturbance cong vao mo-men (vong trong), ca 2 kenh --> DA LAM
3. So sanh voi SMC co ban --> DA LAM
4. Mo phong 2 vong: kinematic + dynamic --> DA LAM
5. Backstepping bac 2 (dynamic BS) --> DA LAM
6. Output vong trong la mo-men tau --> DA LAM

---

## GHI CHU KY THUAT

- V_m = 0 CHI KHI d=0 (trong tam trung truc banh). Neu ro gia thiet khi viet luan van.
- B(z) trong Wang eq.5c KHAC B trong dynamic model — can than ky hieu
- g_pinv = (g'g)^{-1}g', det(g'g) = zx^2+1 > 0 luon ton tai
- Tham so goc (claude_code_prompt.md): fc=0.3, tau_max=5 → da sua: fc=0.05, tau_max=20
- CL vectorized: S_bar * deltas' thay loop, nhanh hon ~100x trong MATLAB

---

## TAI LIEU THAM KHAO

1. Wang et al. (2025) — ADP fixed-time, critic-only, eq.(1)-(18). IEEE RA-L, vol.10, no.1
2. Fierro & Lewis (1997) — Dynamic WMR model, backstepping 2 lop
3. Vamvoudakis & Lewis (2010) — Online Actor-Critic ADP, policy iteration (SM1)
4. De La Cruz & Carelli (2008) — Dynamic model WMR, co V_m khi d!=0
5. Polyakov (2012) — Fixed-time stabilization theory

---

## CACH CHAY MATLAB TU TERMINAL

```powershell
& "C:\Program Files\MATLAB\R2023a\bin\matlab.exe" -batch "cd('c:\Users\LEGION\Desktop\luan van\ADP_Control_for_Wheeled_Mobile_Robot\simulations'); sim_sm1_full"
```
- `-batch`: chay roi thoat, khong GUI, in output ra terminal
- Timeout nen de ~5 phut cho simulation dai

> Quy trinh day du (sim -> export figure -> build LaTeX): `docs/guides/workflow.md`
> Cau truc thu muc: `docs/guides/structure.md`
> Quy uoc commit: `docs/guides/commit-convention.md`
