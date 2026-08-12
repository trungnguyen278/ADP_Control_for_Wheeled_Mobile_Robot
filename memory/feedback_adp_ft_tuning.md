---
name: feedback-adp-ft-tuning
description: ADP-FT robust gains must be reduced for dual-loop architecture; uo clamp prevents divergence
metadata:
  type: feedback
---

ADP-FT robust gains from Wang et al. are for kinematic-only. In dual-loop (kinematic + dynamic), gains must be 3-5x smaller because inner loop already rejects disturbance.

**Why:** With original gains (lambda=0.3, beta=1), ADP-FT diverges at dist_amp >= 0.4 and mass uncertainty >= 40%. The robust terms create aggressive commands that the inner loop can't follow, creating a positive feedback loop.

**How to apply:**
- ft_lambda, ft_mu, ft_alpha ~ 0.08 (was 0.3)
- ft_beta ~ 0.15 (was 1.0)
- ft_rho = 0.1 (was 0.05)
- ft_Gamma = 1*I (was 2*I) — slower learning prevents weight oscillation
- ft_kappa1 = 0.04 (was 0.02) — stronger regularization
- ft_uo_max = 1.5 — ESSENTIAL clamp on feedback (like CL has cl_uo_max)
- With these params: ADP-FT beats BS at 0-60% disturbance on circle

Related: [[feedback-advisor-nam]]
