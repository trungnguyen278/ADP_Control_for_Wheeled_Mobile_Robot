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
| 8 | Hoan thien SM1 + SM2 | DANG LAM |
| 9-11 | Luan van: ADP Fixed-time + Dual-loop (Wang et al.) | DA XONG (code) |
| 12-13 | Mo phong tong hop + Viet luan van | CHUA LAM |
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

**Ket qua so sanh hien tai (circle, T=60s):**

| Method | Jc (no dist) | Jc (dist) | z_rms(5s) no dist |
|--------|-------------|-----------|-------------------|
| BS     | 71          | 90        | 0.0002            |
| CL     | 88          | 101       | 0.086             |
| ADP-AC | 107         | 121       | 0.136             |
| ADP-FT | 123         | 152       | 0.122             |
| SMC    | 475         | 556       | 0.276             |

---

## VIEC CAN LAM (uu tien giam dan)

### Uu tien cao
- [ ] sim_sm2.m: mo phong rieng SM2 (kinematic-only, so sanh AC vs CL)
- [ ] Viet bao cao SM2

### Uu tien trung binh
- [ ] sim_thesis.m: robustness analysis (tang m=15kg, nhieu lon hon, quy dao khac)
- [ ] Cai thien SMC (z_rms con cao, xem xet NTSMC)

### Uu tien thap (cuoi ky)
- [ ] Viet luan van
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
