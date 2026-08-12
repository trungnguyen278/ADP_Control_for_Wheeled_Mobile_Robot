# TIEN DO LUAN VAN — ADP Fixed-Time Optimal Control for WMR

> **QUY TAC: Cap nhat file nay MOI KHI hoan thanh cong viec. Ghi ngay, noi dung, ket qua.**

De tai: Mo rong ADP-based Fixed-time Optimal Control (Wang et al., IEEE RA-L 2025)
tu kinematic-only sang dual-loop kinematic + dynamic cho WMR vi sai.

GVHD: PGS.TS. Nguyen Hoai Nam
Hoc vien: Nguyen Thanh Trung

---

## TRANG THAI (cap nhat 2026-08-12)

**Dang cho phan hoi cua GVHD.** Da gui ban v2 qua Teams ngay 2026-05-26,
den nay (2026-08-12) chua nhan duoc phan hoi — da 2,5 thang.

**Chua co lich bao ve.** Ke hoach goc 16 tuan (02/2026 - 06/2026) da qua han;
khong con dung lam moc thoi gian nua, giu lai duoi day chi de tham chieu lich su.

San pham da hoan thanh va nop:

| Tai lieu | Trang | Trang thai |
|----------|-------|------------|
| docs/thesis/thesis.pdf | 43 | Da nop 2026-05-26 (ban v2) |
| docs/reports/summary/summary_report.pdf | 13 | Da nop 2026-05-26 |
| docs/reports/sm2/sm2_report.pdf | 15 | Da nop 2026-05-26 |
| docs/reports/sm1/sm1_report.pdf | 10 | Da bao cao 2026-03-28 |

Toan bo code mo phong + 19 figure da xong. Xem muc VIEC CAN LAM cho phan con lai.

---

## KE HOACH GOC 16 TUAN (da qua han — luu de tham chieu)

| Tuan | Noi dung | Trang thai |
|------|----------|------------|
| 1 | Mo hinh Dynamic WMR (models/*.m, test_openloop.m) | DA XONG |
| 2-3 | SM1: Actor-Critic ADP kinematic (2 mang, can PE) | DA XONG |
| 4 | Hoan thien SM1, viet bao cao | DA XONG |
| 5-7 | SM2: Critic-only + Concurrent Learning (1 mang, bo PE) | DA XONG |
| 8 | Hoan thien SM1 + SM2 | DA XONG |
| 9-11 | Luan van: ADP Fixed-time + Dual-loop (Wang et al.) | DA XONG |
| 12-13 | Mo phong tong hop + Viet luan van | DA XONG |
| 14-16 | Hoan thien + Bao ve | CHUA LAM — cho GVHD |

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
  - Phan B: 3 khoi luong (10, 12, 14 kg) x 3 pp (BS, CL, ADP-FT)
  - Phan C: 4 muc nhieu (0, 20, 40, 60%) x 3 pp
- [x] plot_thesis.m: 12 figures tong hop
- [x] ref_trajectory.m: them quy dao figure-8 (lemniscate)
- [x] sm1_params.m: them tham so fig8_a, fig8_b, fig8_omega

### 2026-05-25 — Bao cao + Luan van
- [x] export_all_figures.m: xuat 19 figures PDF tu .mat data
  - 6 SM2: sm2_xy_circle/line, sm2_error_circle, sm2_weights, sm2_bellman, sm2_cost
  - 12 thesis: xy/error x3 traj, bar_jc_traj, weights_adp_ft, robust_mass/dist, bar_jc_mass/dist
  - 1 torque: torque_dist
- [x] docs/reports/sm2/sm2_report.tex: bao cao SM2 (CL thay PE, 7 sections, tieng Viet co dau)
- [x] docs/reports/summary/summary_report.tex: bao cao tom tat cho thay (tieng Viet co dau)
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

### 2026-05-26 → 2026-08-12 — Giai doan cho phan hoi (khong co hoat dong)
- Gui ban v2 cho thay Nam qua Teams ngay 2026-05-26, sau do khong co phan hoi.
- Repo khong co commit nao trong 2,5 thang. File noi dung sua cuoi cung:
  docs/thesis/thesis.tex (2026-05-26 10:01).
- Ghi lai khoang trong nay de sau nay khong hieu nham la du lieu bi mat.

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

### 2026-08-12 (chieu) — Them NTSMC + PHAT HIEN VAN DE VOI SO SANH SMC
- [x] Doi ten sm1_params.m -> ctrl_params.m (giu tham so cho ca 6 pp, ten cu sai pham vi)
- [x] controllers/ntsmc_kinematic.m: Non-singular Fast Terminal SMC
  - Chong ky di bang NOI TUYEN TINH duoi nguong eps_ns (kiem chung: huu han tai zy=1e-12 va zy=0)
  - Ban THICH NGHI cho he bac nhat, KHONG phai chep nguyen Feng et al. (he bac hai)
- [x] Wire 'ntsmc' vao sim_sm1_full.m + sim_thesis.m (thanh 6 phuong phap)
- [x] plotting: bo hardcode 5 pp, dung cmap theo thu tu => them pp moi khong phai sua ve

> ### !!! CANH BAO: SO SANH SMC TRONG LUAN VAN KHONG CONG BANG !!!
>
> Sweep gain (140+ lan chay full model) cho ket qua sau:
>
> | SMC | Jc (0% nhieu) | Jc (20%) | z_rms 5s cuoi |
> |-----|---------------|----------|----------------|
> | gain DANG DUNG trong luan van (lambda=3, eta=1.0) | 475.1 | 555.5 | 0.276 |
> | gain sau sweep (lambda=0.7, eta=0.05) | **61.8** | **79.9** | **0.00011** |
>
> **SMC tot hon 7.0 lan CHI nho doi gain.** Ket luan "ADP-FT tot nhat" cua luan van
> dua tren baseline SMC bi tune te.
>
> So sanh cong bang (circle, T=60s), Jc theo muc nhieu:
>
> | Method       | 0%    | 20%   | 40%   | 60%   |
> |--------------|-------|-------|-------|-------|
> | SMC (tuned)  | 61.8  | 79.9  | 107.3 | 150.9 |
> | NTSMC(tuned) | 62.0  | 80.2  | 107.8 | 151.5 |
> | ADP-FT       | 70.7  | 85.2  | 111.8 | 157.7 |
> | BS           | 70.7  | 90.1  | 116.6 | 156.7 |
> | CL           | 87.5  | 101.1 | 124.2 | 162.3 |
>
> SMC tuned thang ADP-FT o MOI muc nhieu. z_rms 5s cuoi: SMC tuned 0.00011
> vs ADP-FT 0.038-0.060 (kem hon ~400 lan).
>
> Sai lech khoi luong (controller tuong m=10, nhieu 20%), Jc:
>
> | Method       | 10kg | 12kg  | 14kg   | 16kg   | 20kg    |
> |--------------|------|-------|--------|--------|---------|
> | NTSMC(tuned) | 80.2 | 149.0 | 264.8  | 494.8  | 1537.3  |
> | SMC (tuned)  | 79.9 | 148.6 | 267.0  | 530.0  | 1916.8  |
> | BS           | 90.1 | 156.4 | 264.2  | 619.8  | 5722.7  |
> | CL           | 101.1| 159.8 | 334.6  | 3000.4 | 2929.9  |
> | ADP-FT       | 85.2 | 157.2 | **4283.0** | 8241.1 | 10518.8 |
>
> ADP-FT la bo TE NHAT duoi sai lech khoi luong, sup o 14 kg (z_rms=3.03, mat bam).
> 14 kg NAM TRONG Phan B cua luan van (masses = [10,12,14]) => du lieu nay DA CO
> trong luan van, chi la chua doi chieu voi baseline duoc tune tuong duong.
>
> **CHUA SUA gain SMC trong ctrl_params.m** — doi Trung quyet dinh, vi viec nay
> thay doi toan bo bang so lieu + hinh + ket luan cua luan van da nop.
>
> Lap luan con bao ve duoc cho ADP-FT (trung thuc):
> 1. ADP-FT dat ket qua do KHONG can sweep offline 140 lan chay va KHONG can biet
>    mo hinh; gain SMC tim duoc bang brute-force co mo hinh.
> 2. Diem ban cua ADP-FT la BAO DAM hoi tu co dinh thoi gian, khong phai Jc nho nhat.
> => Nen dat lai tuyen bo theo 2 huong nay, thay vi tuyen bo "Jc tot nhat".

### 2026-08-12 — Ket qua NTSMC sau khi tune (sim_sm1_full, circle T=60s)

| Method | Jc (no dist) | Jc (dist) | z_rms 5s cuoi (no dist) |
|--------|--------------|-----------|--------------------------|
| NTSMC  | **62.01**    | **80.20** | **0.000091**             |
| BS     | 70.72        | 90.08     | 0.000244                 |
| ADP-FT | 70.66        | 85.22     | 0.059807                 |
| CL     | 87.50        | 101.06    | 0.086265                 |
| ADP-AC | 107.26       | 121.20    | 0.135562                 |
| SMC    | 475.05       | 555.51    | 0.276272                 |

> Ket qua sweep NTSMC (phai biet khi viet luan van):
> - Optimum nam o beta -> vo cung, tuc TAT so hang terminal tren mat truot
> - alpha_s KHONG anh huong (4 gia tri cho ket qua giong het chu so)
> - alpha_r anh huong ~0.08%
> => Cai thien den TU VIEC GIAM GAIN, khong phai tu cau truc terminal.
>    KHONG duoc viet "NTSMC tot hon SMC nho hoi tu huu han thoi gian".

**Ket qua so sanh CU (gain SMC chua tune) — giu de doi chieu:**

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

## VIEC CAN LAM

### Dang bi chan (cho GVHD)
- [ ] Hoan thien luan van theo phan hoi cua thay Nam -- CHUA CO PHAN HOI tu 2026-05-26
- [ ] Chuan bi slide bao ve -- chua co lich bao ve

### Lam duoc ngay, khong phu thuoc GVHD
- [ ] Cai thien SMC (z_rms = 0.276 con cao so voi ADP-FT 0.060, xem xet NTSMC)
- [ ] Gom trung lap giua plot_sm1_results.m va plot_sm1_full.m
- [ ] Can nhac doi ten sm1_params.m -> ctrl_params.m (dang giu tham so cho CA 5 phuong phap,
      ten khong con phan anh pham vi)

### Da xong
- [x] sim_sm2.m + plot_sm2.m: mo phong rieng SM2 (kinematic-only, AC vs CL)
- [x] sim_thesis.m + plot_thesis.m: robustness (3 quy dao, 3 mass, 4 dist_amp)
- [x] ref_trajectory.m + sm1_params.m: them quy dao figure-8
- [x] export_all_figures.m: 19 figures PDF (6 SM2 + 12 thesis + 1 torque)
- [x] docs/reports/sm2/sm2_report.tex -- bao cao SM2
- [x] docs/reports/summary/summary_report.tex -- bao cao tom tat cho thay
- [x] docs/thesis/thesis.tex -- luan van 6 chuong
- [x] Compile LaTeX sang PDF (MiKTeX, pdflatex x2 cho cross-ref) -- 2026-05-25
- [x] Tai cau truc repo + tai lieu quy uoc -- 2026-08-12

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
- Tham so ban dau fc=0.3, tau_max=5 → da sua thanh fc=0.05, tau_max=20 (models/wmr_params.m).
  (Nguon goc la file claude_code_prompt.md, da bo tu commit cfae86b)
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
