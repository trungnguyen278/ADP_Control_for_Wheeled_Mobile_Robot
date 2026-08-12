---
name: feedback-figure-placement
description: Figures must not be placed between equations and their explanations — place at natural section breaks
metadata:
  type: feedback
---

Không đặt hình giữa công thức và phần giải thích ký hiệu. Đặt hình tại điểm chuyển đoạn tự nhiên (cuối mục, trước subsection mới).

**Why:** User reported "nhiều hình bị chèn giữa phần giải thích ký hiệu và nội dung" — figures splitting equations from their explanations breaks the reading flow.

**How to apply:** When inserting TikZ figures or includegraphics in LaTeX, check that the figure does not land between an equation and the paragraph that explains its symbols. Also: legends must not cover plot content — place outside the plot area (e.g. right side or below). For TikZ diagrams, verify labels don't overlap arrows/axes by checking coordinate geometry.

Related: [[feedback-vietnamese]], [[project-progress]]
