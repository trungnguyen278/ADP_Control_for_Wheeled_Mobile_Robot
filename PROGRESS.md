# TIEN DO LUAN VAN — ADP Fixed-Time Optimal Control for WMR

> **QUY TAC: Cap nhat file nay MOI KHI hoan thanh cong viec. Ghi ngay, noi dung, ket qua.**

De tai: Mo rong ADP-based Fixed-time Optimal Control (Wang et al., IEEE RA-L 2025)
tu kinematic-only sang dual-loop kinematic + dynamic cho WMR vi sai.

GVHD: PGS.TS. Nguyen Hoai Nam
Hoc vien: Nguyen Thanh Trung

---

## TRANG THAI (cap nhat 2026-10-26)

**2026-10-26: phat hien va sua loi ma tran B trong mo hinh dong luc hoc**, chay
lai toan bo mo phong. Phan lon so lieu va nhieu ket luan Ch.5-6 cua ban
78 trang (2026-08-13) KHONG con dung. Chi tiet o muc "2026-10-26" ben duoi.
**Luan van CHUA viet lai** — danh sach viec o muc VIEC CAN LAM, muc 2.

**GVHD:** thay Nam da nhan ban v2 (43 trang, commit 7fedea9) + bao cao tom tat
(13 trang, commit 5e1a452) qua Teams 2026-05-26 nhung CHUA DOC. Hai ban nay lam
sau chat 28/03 (yeu cau cua thay: memory/feedback_advisor_nam.md). Ban 78 trang
chua gui. Ca ba ban deu mang loi B. Trung muon hoan thien roi in ra nop.

**2026-08-13: da hoan thanh ban v3** sau dot tu ra soat toan bo luan van
(doi chieu voi bai bao goc + chay them 4 thi nghiem kiem chung).
Chi tiet o muc "2026-08-13" ben duoi. thesis.pdf: 43 -> 66 -> 78 trang.

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

### 2026-08-13 — RA SOAT TOAN BO LUAN VAN + BAN v3

Doi chieu tung cong thuc voi bai bao goc, chay 4 thi nghiem kiem chung moi.
Ket qua: 3 loi trich dan + 1 loi tinh toan + 3 phat hien ky thuat.

**A. Loi trich dan da sua (4 file .tex):**
- Ten tac gia Wang: "Y. Wang, Z. Li, et al." -> **C. Wang, H. Zhan, Q. Guo, T. Li**
  (loi co tu commit e63ec30 ngay 2026-03-28, lan sang ca 4 tai lieu).
  summary_report con bia them "C. Yang" va sai so trang (460-467 -> 176-183).
- **eq.(17) trong luan van KHONG phai cua Wang.** Wang dung
  `nabla_phi*B*R_inv*B'*z`, luan van dung gradient sai so Bellman. Da tach ra
  thanh dong gop rieng (§4.7.2) + ghi ro chung minh goc khong con phu duoc.
- **Bang 4.1 cot "Wang" sai toan bo.** Gia tri that: lambda=mu=alpha=diag(1,1,0.4),
  beta=1e5, rho=10, kappa1=kappa2=0.1, W(0)=0. Cot cu thuc ra la gia tri trung
  gian cua chinh minh. "Giam 3-5 lan" -> that ra beta giam 666.000 lan.

**B. Loi tinh toan da sua:**
- §5.5: `Ju = Jc - Je*Q/R` SAI, phai la `Jc - 10*Je`.
  ADP-FT: 47.3 -> **9.42**; SMC: 409.4 -> **263.25**; ti so 8.6x -> **27.9x**.
  Phat hien them: 53% chi phi cua SMC la do BAM KEM, khong phai nang luong.

**C. Ba thi nghiem moi (simulations/sim_thesis_extra.m):**

1. **Ablation ADP-UUB** (bo kappa2, beta, p/q=1):

   | Traj | ADP-FT | ADP-UUB |
   |---|---|---|
   | circle | **85.2** | 109.8 |
   | line | 8.5 | **7.6** |
   | figure8 | **93.2** | **2357.6** |

   => So hang fixed-time cai thien **25 lan** tren figure8. Co gia tri that.

2. **Quet dieu kien dau** — ADP-FT **MAT BAM 3/5**, CL mat 4/5, BS on dinh 5/5.
   Nguyen nhan: W(T) doi dau => V_hat mat tinh xac dinh duong.
   => Tinh hoi tu co dinh thoi gian KHONG ton tai tren kien truc 2 vong.

3. **Nhieu 2 kenh**: `B*[1;1] = [r;0]` — nhieu dong pha bi triet tieu HOAN TOAN
   o kenh omega. Yeu cau #2 cua thay Nam chua dat theo nghia kich thich 2 kenh.
   TIN TOT: voi nhieu doc lap 2 kenh, ADP-FT van thang BS o moi muc va khoang
   cach RONG HON (60%: 124.5 vs 136.7 = 8.9%, so voi kich ban cu la thua 0.6%).

4. **Ablation luat cap nhat** (them `s.ft_update_law` vao adp_fixed_time.m):

   | Luat | circle | line | fig8 | z0 on dinh | 14kg |
   |---|---|---|---|---|---|
   | bellman (mac dinh) | **85.2** | **8.5** | 93.2 | 2/5 | **4283** |
   | wang nguyen ban | 89.4 | 9.2 | **62.8** | **3/5** | 6495 |

   Danh doi 2 chieu, khong co phuong an thang tuyet doi. GIU bellman lam mac dinh.
   Luu y: cai thien `Jc 4385 -> 123` (2026-05-11) KHONG phai do doi luat cap nhat
   ma chu yeu do tune gain — truoc day quy nham.

**D. Bo sung vao luan van:**
- Front matter day du: loi cam doan, loi cam on, tom tat VI, abstract EN,
  danh muc hinh/bang/tu viet tat/ky hieu, danh so La Ma.
- §3.4: them dinh nghia **practical fixed-time** (hoi tu vao lan can).
- §4.8 MOI: phan tich on dinh he ghep hai vong (cascade/ISS), dieu kien tach
  thang thoi gian dinh luong `Kd >> ||mu||` (250 >> 1), va 3 gioi han cua dam bao.
- §5.6 MOI (ablation), §5.7 MOI (quet z0), §5.8 MOI (nhieu 2 kenh).
- §5.9 + Chuong 6: dat lai toan bo tuyen bo cho dung pham vi.
- SMC: them caveat trung thuc ve viec chua tune cong bang (CHUA doi gain).

**E. Build:** thesis 66 trang (tu 43), summary 13, sm1 10, sm2 15.
Tat ca: 0 loi, 0 undefined ref, 0 Overfull hbox.

**F. Khac:** khoi phuc ten goc file bai bao
(`Adaptive_Dynamic_Programming-Based_Fixed-Time_Optimal_Control_for_Wheeled_Mobile_Robot.pdf`)
— session truoc doi thanh `Wang2025_...` ma khong ghi lai.

### 2026-08-13 (chieu) — NANG CAP: TOAN TU CHIEU + SO SANH CONG BANG SMC

Trung chon phuong an "nang cap vua". Hai viec chinh.

**1. Toan tu chieu (dong gop ky thuat moi)**

V_hat = W'*phi = z'*P(W)*z. Rang buoc P(W) >= eps_p*I la tap loi nen chieu duoc.
Kiem tra bang Sylvester (re), chi eig khi vi pham. Bat mac dinh trong ctrl_params.

| | khong chieu | co chieu |
|---|---|---|
| Jc 3 quy dao (z0 chuan) | 85.2 / 8.5 / 93.2 | KHONG DOI |
| Jc 14kg | 4283 | **624.0** |
| z_rms 14kg | 3.03 | **0.172** |
| z0=3.500 | 3.75 | **1.1e-3** |
| z0=0.612 | 3.79 | 1.84 (van hong) |
| z0=2.375 | 2.31 | 2.85 (xau di) |

=> Cuu duoc van de khoi luong, KHONG cuu duoc mien hut (van 3/5).
   Gia thuyet "W mat xac dinh duong" chi dung MOT PHAN. Phan con lai quy cho
   nang luc bieu dien cua co so bac hai co dinh — phai doi cau truc xap xi.

**2. So sanh cong bang voi SMC (§5.10 moi)**

| | circle | line | figure8 |
|---|---|---|---|
| SMC thong dung (lambda=3, eta=1) | 555.5 | 64.7 | 2457.6 |
| SMC da tune (lambda=0.7, eta=0.05) | **79.9** | **5.5** | **8.5** |
| ADP-FT | 85.2 | 8.5 | 93.2 |

Theo muc nhieu (circle): SMC tune 61.8 / 79.9 / 107.3 / 150.9 — thang ADP-FT
(70.7 / 85.2 / 111.8 / 157.7) o MOI muc. z_rms 1.07e-4 vs 4.63e-2 (433 lan).

=> DA DAT LAI tuyen bo trung tam cua luan van. ADP-FT KHONG phai phuong phap
   co Jc thap nhat. Gia tri con bao ve duoc: dat hieu nang CUNG BAC ma khong
   can biet mo hinh va khong can quet 140 lan chay ngoai tuyen.
=> Bai hoc phuong phap luan duoc nang thanh mot dong gop: chat luong tune tham
   so co the lan at khac biet ban chat giua cac phuong phap.

**Ket qua:** thesis.pdf 66 -> 73 trang, 0 loi / 0 undefined ref / 0 Overfull.
Xuat lai 19 figure. Commit 8551f3c, e12ff7d, 4ec31f3, 28acb09.

### 2026-08-13 (toi) — DO BEN CUA SMC DA TUNE: THANG TOAN DIEN

Chay SMC-tune qua dung 3 bo thi nghiem da ap cho ADP-FT (Phan I).

| Khoi luong (Jc) | 10kg | 12kg | 14kg | 16kg |
|---|---|---|---|---|
| SMC tune | **79.9** | **148.6** | 267.0 | **530.0** |
| BS | 90.1 | 156.4 | **264.2** | 619.8 |
| ADP-FT+chieu | 85.2 | 157.2 | 624.0 | 8109.5 |

| z_rms theo z0 | 0.245 | 0.612 | 1.225 | 2.375 | 3.500 |
|---|---|---|---|---|---|
| SMC tune | 1.07e-4 | 1.07e-4 | 1.07e-4 | 1.07e-4 | 1.07e-4 |
| ADP-FT+chieu | 4.6e-2 | 1.84 | 1.3e-1 | 2.85 | 1.1e-3 |

Nhieu doc lap 2 kenh (Jc): SMC tune 61.8 / 77.2 / 96.4 / 123.3 — thap nhat moi muc.

**KET LUAN: uu the cua SMC tune la BEN VUNG.** ADP-FT khong thang o BAT KY tieu
chi nao. Nghich ly: SMC khong tuyen bo hoi tu co dinh thoi gian nhung z_rms
khong doi qua ca 5 dieu kien dau — doc lap z0 tot hon han ADP-FT.

Doi chung NTSMC (cung gain 0.7/0.05): 80.2 / 5.4 / 8.4 vs SMC 79.9 / 5.5 / 8.5.
Chenh duoi 2% => cai thien den tu GIA TRI THAM SO, khong tu cau truc terminal.

**Lap luan bao chua PHAI BO:** "ADP-FT khong can tune". Sai — 10 tham so vs 2.
**Lap luan CON lai (chi mot):** thich nghi khi Q, R doi. Chua co so lieu.
Da dat thanh huong so 1 o §6.3.

Luan van da phat bieu thang ket qua am tinh nay. thesis.pdf 73 -> 75 trang.
Commit 15466cc, 8da2b10.

### 2026-08-13 (khuya) — TIM RA NGUYEN NHAN THAT VA KHAC PHUC DUOC

Trung yeu cau: "it nhat cung phai on dinh va hoi tu". Da dat duoc.

**Chan doan (theo buoc thoi gian, khong quet mu):**
Bon so hang robust tac dong len TUNG thanh phan z doc lap. Khi sai so don vao
kenh ngang (zy lon, zx va zth ~ 0) thi ca uo_adp lan robust deu ~ 0 => diem can
bang gia. Do duoc: ||z||=2.85 nhung ||uo||=0.085 — khong phan ung.

Day la cascade nonholonomic (§2.5 luan van da mo ta). BS xu ly bang k2*vr*zy,
SMC bang mat truot zth+c*zy. Wang KHONG co co che tuong duong.

**Giai phap:** robust(3) += c_zy * vr * tanh(zy/sat), voi c_zy=2.0, sat=0.2.

| | truoc | sau |
|---|---|---|
| max z_rms qua 5 z0 | 2.85 (hong 2/5) | **8.2e-5 (on dinh 5/5)** |
| z_rms circle | 4.63e-2 | **7.8e-5** (thap nhat trong CA 6 PP) |
| t_settle max/min | -- | **1.68** (BS 2.87) |
| Jc figure8 | 93.2 | **75.3** |
| Jc 14kg / 16kg | 624 / 8110 | **527 / 4238** |
| Jc circle | 85.2 | 89.6 (gia phai tra) |
| Jc 60% nhieu | 157.7 | 196.5 (chattering) |

**Truoc khi tim ra, da thu va THAT BAI:** tang robust gain (k=1..20) — lam xau
di moi thu. Ghi lai de khong thu lai.

**TUYEN BO TRUNG TAM MOI:** ADP-FT dat SAI SO BAM NHO NHAT trong ca 5 phuong
phap, on dinh moi dieu kien dau, t_settle it phu thuoc z0 hon BS. KHONG dat chi
phi thap nhat (SMC tune van thap hon) va KHONG dat hoi tu co dinh thoi gian
theo nghia chat (1.68 khac 1).

thesis.pdf 75 -> 78 trang, 0 loi. Commit 47d384e, 57ddb91.

> **CON LAI:** (1) do ben khoi luong van la diem yeu (527 vs BS 264 tai 14kg);
> (2) Jc o nhieu 60% cao hon BS 25%; (3) chua kiem chung kha nang thich nghi khi
> doi Q, R — huong so 1 o §6.3.
>
> **[2026-10-26] TOAN BO so lieu tu 2026-05-11 den day do tren mo hinh co loi
> ma tran B — xem muc 2026-10-26 ben duoi.**

### 2026-10-26 — PHAT HIEN VA SUA LOI MA TRAN B, CHAY LAI TOAN BO

**Loi.** `wmr_params.m` dung `B = [r/2 r/2; r/(2L) -r/(2L)]` — Jacobian dong hoc
(van toc banh -> v, w), KHONG phai ma tran mo-men. Dung theo Fierro & Lewis (cong ao,
B = J^-T): `B = (1/r)[1 1; L -L]`, tuc F = (tauR+tauL)/r. Kenh v yeu 800 lan, kenh w
yeu 18 lan. Loi co tu mo hinh goc (02/2026) nen ban v2 da gui thay cung mang loi nay.

**Kiem chung truoc khi sua** (Octave 8.4; tai tao 12/12 so chinh cua luan van, sai
khac < 0.1% => so lieu cu khong chep sai, sai o mo hinh):
- 94-98% Jc nam trong 10 s dau; mo-men bao hoa ~68% thoi gian 10 s dau; moi PP
  mat 3.5 s moi dat 95% vr. Bang Jc cu do "ai chiu bao hoa luc khoi dong tot hon".
- Nang tau_max 5 -> 20 va ha fc 0.3 -> 0.05 truoc day la va trieu chung cua loi nay.

**Da lam.** Sua B, tau_max = 1 N*m TAM (gia toc max 4 m/s^2), test_openloop 5/5 PASS
[9b3fcf0]. Quet lai SMC 72 lan: mat chi phi phang, giu 0.7/0.05 [d9a9532]. Them
Phan J: ablation co che + dong bang critic + tham so goc Wang [5babecd, 17d1f82].
Chay lai sim_thesis + sim_thesis_extra bang Octave (231 lan chay). Gain giu nguyen.

**Ket qua moi** (T = 60 s, nhieu 20% dong pha, z0 chuan):

| Phuong phap | Jc circle | Jc line | Jc figure8 | z_rms circle |
|---|---|---|---|---|
| SMC da tune (Phan H) | **1.195** | **0.679** | **1.963** | 5.37e-4 |
| NTSMC (cung gain) | 1.247 | 0.798 | 2.056 | 5.35e-4 |
| BS | 1.393 | 0.704 | 2.765 | 5.57e-4 |
| ADP-FT | 1.402 | 0.783 | 2.109 | 5.32e-4 |
| SMC thong dung | 1.631 | 0.957 | 2.396 | 4.35e-4 |
| ADP-AC | 7.450 | 5.143 | 7.912 | 4.85e-2 |
| CL | 7.712 | 5.110 | 8.272 | 6.14e-2 |

Do ben (Phan B, C, F, I): khoi luong 10-16 kg va nhieu 0-60% (dong pha, vi sai, doc
lap) KHONG con phan biet duoc cac PP: Jc doi < 2% (BS 1.391-1.413, ADP-FT
1.397-1.409, SMC tune 1.192-1.213). z_rms tang tuyen tinh theo bien do nhieu va nhu
nhau o BS / ADP-FT (20%: dong pha 5.3-5.6e-4, vi sai 1.6e-3, doc lap 9.9e-4).
=> Vong trong (dynamic BS, Kd = 20) hap thu gan het bat dinh khoi luong va nhieu mo-men.

Phan E — t_settle (tol 0.05), ||z0|| = 0.245 / 0.612 / 1.225 / 2.375 / 3.5:
- ADP-FT: 3.95 / 5.09 / 6.06 / 11.70 / 19.08 s
- BS:     8.53 / 13.16 / 14.44 / 15.98 / 27.66 s
- CL: khong dat o z0 nao (z_rms 6e-2 > tol)
=> ADP-FT nhanh hon BS o ca 5 z0. Ca hai tang theo ||z0|| (bat buoc vi v bi chan).

Phan D, G, J — ablation ADP-FT (Jc):

| Cau hinh | circle | line | figure8 | quet 5 z0 (z_rms) |
|---|---|---|---|---|
| Tham so goc Wang | 4.856 | 12.976 | 93.977 | 0.02 / 0.43 / 0.37 / 0.35 / 0.71 |
| Chua co che (gain luan van) | 8.092 | 5.657 | 10.041 | mat bam 2/5 (2.68; 2.12), con lai 0.06-0.14 |
| + toan tu chieu | 8.092 | 5.657 | 10.041 | mat bam 1/5 (2.85), con lai 0.02-0.14 |
| + ghep nonholonomic | 1.402 | 0.783 | 2.109 | 5.3e-4 ca 5 |
| Day du (mac dinh) | 1.402 | 0.783 | 2.109 | 5.3e-4 ca 5 |
| Dong bang critic (Gamma = 0) | **1.313** | **0.734** | **1.976** | 5.3e-4 ca 5 |
| ADP-UUB (bo so hang fixed-time) | 1.327 | 0.743 | 1.990 | -- |
| Luat cap nhat goc Wang | 1.401 | 0.783 | 2.111 | -- |

=> **Chi co ghep nonholonomic tao khac biet** (5.8 lan tren circle, het mat bam).
Toan tu chieu, luat cap nhat, so hang fixed-time va viec critic hoc deu KHONG dong
gop, hoac lam xau di 5-6%. ||W(T)|| = 0.586 voi ca hai luat: khi z nho, so hang
sigma-mod chiem uu the nen W chi suy giam ve 0.
=> Tham so goc Wang KHONG phat tan nhung bam kem (sai so 0.35-0.71 m o 4/5 z0, hinh 8
gap 45 lan) — van can chinh tham so, nhung ly do khong phai "phat tan".

**Do nhay gain** (thong tin cho quyet dinh tune lai, Jc circle / figure8):
- ADP-FT nhan gain robust x0.5 / x1 / x2 / x5 / x10: 1.49/2.27, 1.40/2.11,
  **1.29/1.94**, 1.28/2.38, 1.72/4.31.
- BS nhan ca 3 gain x0.5 / x1 / x2: **1.26/2.35**, 1.39/2.77, 1.51/3.15; k1 = 3 hoac 5
  deu xau hon.
=> Tune nhe moi PP doi Jc ~10%, khong doi buc tranh: SMC tune tot nhat tren circle,
ADP-FT ~ SMC tren hinh 8, BS kem hon tren hinh 8.

**Phan K — ADP-FT co thich nghi khi doi Q, R khong** [cabcfa7] (circle, nhieu 20%;
Je = tich phan ||z||^2, Ju = tich phan ||uo||^2):

| Q x / R x | 1 / 1 | 0.1 / 1 | 10 / 1 | 1 / 0.1 | 1 / 10 |
|---|---|---|---|---|---|
| Je | 0.1263 | 0.1267 | 0.1228 | 0.2475 | 0.2888 |
| Ju | 0.0696 | 0.0698 | 0.0677 | 0.1629 | 0.2239 |

=> AM TINH. Doi Q 100 lan: Je, Ju doi < 3%. Doi R (ca hai chieu) lam xau CA Je lan Ju
— R chi tac dong qua R^-1 trong uo_adp nhu mot he so khuech dai, khong phai thich nghi.
||W(T)|| = 0.586 o moi cau hinh. Day la "phep thu quyet dinh" ma Muc 6.3-6.4 dat ra.

**So sanh voi ket luan ban 78 trang:**

| Ket luan cu | Mo hinh da sua |
|---|---|
| ADP-FT sai so bam nho nhat | Ngang nhau 5.3-5.6e-4; SMC thong dung con thap nhat (4.35e-4) |
| ADP-FT yeu voi khoi luong | Khong PP nao suy giam den 16 kg |
| ADP-FT kem BS 25% o nhieu 60% | 1.407 vs 1.397 (0.7%) |
| ADP-FT yeu tren hinh 8 | ADP-FT 2.109 < BS 2.765 |
| Tham so goc Wang lam phat tan | Khong phat tan, chi bam kem |
| So hang fixed-time cai thien 25 lan tren hinh 8 | Bo di con tot hon 6% |
| Luat Bellman vs Wang: danh doi hai chieu | Nhu nhau |
| Toan tu chieu cuu 14 kg (6.9 lan) | Khong tac dung |
| **Ghep nonholonomic het mat bam** | **VAN DUNG — co che duy nhat tao khac biet** |
| SMC thong dung vs tune: 7 lan / 289 lan | 1.37 / 1.41 / 1.22 lan |
| Ti so t_settle 1.68 vs BS 2.87 | Tieu chi sai; ADP-FT nhanh hon BS o moi z0 |
| Lap luan cuoi: ADP-FT thich nghi khi doi Q, R (chua co so) | Am tinh (Phan K) |

---

## VIEC CAN LAM

### 1. Sua mo hinh va chay lai (2026-10-26)
- [x] Sua ma tran B theo Fierro & Lewis, tau_max tam 1 N*m [9b3fcf0]
- [x] Quet lai SMC tren mo hinh moi: giu 0.7/0.05 [d9a9532]
- [x] Them Phan J: ablation co che + dong bang critic + tham so goc Wang [5babecd, 17d1f82]
- [x] Chay lai sim_thesis + sim_thesis_extra bang Octave 8.4 (so lieu: muc 2026-10-26)
- [ ] **Trung quyet:** tau_max that cua motor (dang tam 1 N*m). Doi gia tri => chay lai het,
      vi nhieu tinh theo % tau_max.
- [ ] **Trung quyet:** co tune lai gain ADP-FT / BS tren mo hinh moi khong. Gain hien tai chon
      tren mo hinh sai (k1 = 1.5 cua BS, toan bo gain robust ADP-FT). Do nhay da do (muc
      2026-10-26): tune nhe doi Jc ~10%, khong doi buc tranh. Neu tune thi tune DEU moi PP.
- [ ] **Trung quyet (quan trong nhat):** dong gop chinh cua luan van. Ket qua moi: chi ghep
      nonholonomic tao khac biet; critic hoc, so hang fixed-time, luat cap nhat, toan tu
      chieu deu khong dong gop; doi Q, R cung khong thich nghi (Phan K). Ten de tai va
      Ch.1/Ch.6 phu thuoc quyet dinh nay -- nen hoi y thay Nam.
- [ ] Chay lai tren MATLAB: test_openloop -> sim_thesis -> sim_thesis_extra -> export_all_figures.
      19 hinh trong docs/thesis/figures VAN LA MO HINH CU (Octave khong dung de xuat hinh).
- [ ] (khong gap) sim_sm1_full.m chua chay lai -- khong dung trong luan van.

### 2. Viet lai luan van (LAM SAU, khi muc 1 xong)
Nguyen tac: KHONG sua chu truoc khi chot tau_max va chay lai tren MATLAB -- so se doi.
Moi con so trong ban 78 trang deu tu mo hinh sai, ke ca cac doan dang nhat quan voi nhau.

Ch.2-4
- [ ] 2.2 (thesis.tex:465-467): B = (1/r)[1 1; L -L], F = (tauR+tauL)/r, N = L(tauR-tauL)/r.
      Cong thuc cu F = (r/2)(tauR+tauL) sai ca thu nguyen (N*m^2).
- [ ] Bang tham so robot (thesis.tex:475, 489): tau_max = 20 -> gia tri moi. Luu y bo tham
      so hien tai khong khop Pioneer 3-DX that (r = 0.05 m; banh P3-DX theo datasheet ~19.5 cm
      duong kinh): hoac dung so that cua P3-DX (ca tau_max), hoac bo ten "Pioneer 3-DX".
- [ ] 4.7.3: bo lap luan "tham so goc cua Wang gay phat tan tren dual-loop". Voi B dung, gain
      goc KHONG phat tan, chi kem hon (hinh 8: Jc 94.0 so voi 2.11). Neu giu, doi ly do.
- [ ] 4.8: phan tich cascade van dung (Kd = 20, mu = 0.08); sua cau ve bao hoa mo-men.
- [ ] 4.7.2: hai luat cap nhat cho ket qua nhu nhau (Phan G) => co the quay ve eq.(17) goc
      cua Wang (s.ft_update_law = 'wang'). Het luon van de trich dan "eq.(17) khong phai cua
      Wang" va "chung minh goc khong con phu" (memory/feedback_wang_citation_mismatch.md).

Ch.5 (toan bo bang, hinh, nhan xet)
- [ ] 5.1: thiet lap mo phong theo tau_max moi.
- [ ] 5.2-5.4 (Phan A/B/C): bo 4 ket luan da sai: "ADP-FT sai so bam nho nhat", "ADP-FT yeu
      voi bat dinh khoi luong", "ADP-FT kem BS 25% o nhieu 60%", "ADP-FT yeu tren hinh 8".
- [ ] 5.5 phan tich mo-men: viet lai -- het hien tuong bao hoa 68% thoi gian luc khoi dong.
- [ ] 5.6: so moi tu Phan D, G -- bo so hang fixed-time con tot hon 5-6%, hai luat cap nhat
      nhu nhau. THEM muc "dong bang critic" (Phan J): critic khong hoc cho ket qua tot hon
      6% => phai noi ro hieu nang den tu so hang robust + ghep, khong tu ADP.
- [ ] 5.7: BO tieu chi "ti so t_settle max/min ~ 1" (sai khai niem: fixed-time chi doi
      t_settle <= T_max). Trinh bay t_settle theo ||z0||. Sua luon thesis.tex:1945, 2034.
- [ ] 5.8: bang co che lay tu Phan J. Toan tu chieu khong con tac dung (bang het voi/khong
      co); ghep nonholonomic la co che quyet dinh. Muc 4.7.4 can xem lai co giu khong.
- [ ] 5.9: nhieu hai kenh lay tu Phan F. Thay chi yeu cau nhieu o MO-MEN vong trong (chat 28/03,
      memory/feedback_advisor_nam.md) -- kich ban dong pha hien tai da dung yeu cau; neu ro dong pha
      chi tac dong kenh v, kich ban doc lap tung banh la kiem tra them.
- [ ] 5.10: SMC thong dung vs da tune chi con chenh ~1.4 lan (truoc 7 lan va 289 lan) --
      bai hoc "chat luong tune lan at phuong phap" yeu di nhieu, can viet lai cho dung muc.
- [ ] 5.11 + bang tong hop 5 phuong phap: viet lai.

Ch.6 va phan dau
- [ ] 6.1-6.4 viet lai theo ket qua moi. 6.3 y 1(iii) de xuat toan tu chieu -- da lam o 4.7.4.
      6.3 y 2 va 6.4 goi "doi Q, R" la phep thu quyet dinh -- da chay (Phan K): am tinh.
- [ ] Tom tat VI + Abstract EN, Muc 1.4 (dong gop), Loi cam doan neu dong gop thay doi.
- [ ] Hinh so sanh 3 loai on dinh (thesis.tex:901): "hoi tu tai cung T_f bat ke x0" -- sai.
- [ ] Bia: "Ha Noi, 06/2026" -> thang nop that.

Khac
- [ ] Chi dung so tu script (sim_thesis Phan A-C, sim_thesis_extra Phan D-K). Vai so cu chay
      le, chua co script (quet be rong bao hoa varsigma, warm start W0 = [6,6,6]): bo, hoac
      them script truoc khi dung.
- [ ] (tuy chon) Them ghep nonholonomic cho CL: CL con sai so ~6 cm cung nguyen nhan thieu
      kenh z_y. Neu chon ghep lam dong gop chinh, dieu nay cho thay co che dung chung.
- SM1, SM2 chi dung mo hinh dong hoc (sim_sm1.m, sim_sm2.m) nen KHONG bi loi B -- giu nguyen.
- [ ] CLAUDE.md muc 2 va README dong 32 ghi "luat ADP eq.(16)-(18) giu nguyen cua Wang" --
      sai tu thang 8 (eq.17 da doi sang Bellman). Sua theo quyet dinh o muc 4.7.2 ben tren.
- [ ] Bao cao tom tat (summary_report.tex) con ghi "ADP-FT tot nhat trong 5 PP" -- dong bo.
- [ ] Gui thay Nam ban moi + danh sach thay doi so voi v2, NOI RO loi B (v2 cung mang loi nay).
- [ ] Gom trung lap plot_sm1_results.m va plot_sm1_full.m (khong gap).

### 3. Dang bi chan (cho GVHD)
- [ ] Hoan thien luan van theo phan hoi cua thay Nam -- CHUA CO PHAN HOI tu 2026-05-26
- [ ] Chuan bi slide bao ve -- chua co lich bao ve

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
2. Disturbance cong vao mo-men (vong trong) --> DA LAM (thay khong yeu cau cu the 2 kenh v, w)
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
