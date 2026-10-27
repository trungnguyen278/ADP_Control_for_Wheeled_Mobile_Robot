"""Tach cac hinh TikZ cua luan van thanh PDF rieng de dung trong slide bao ve.

Chay trong thu muc docs/slides:  python extract_figures.py
Can pdflatex (MiKTeX) trong PATH va docs/thesis/thesis.aux (build luan van truoc) de lay
so hieu cong thuc ghi tren hinh. Moi hinh duoc bien dich rieng voi cung co chu 13pt, font
T5 nhu luan van, nen hinh tren slide giong het hinh trong luan van.
Ghi them figures/controller_full_bbox.tex: khung bao cua so do day du (pt), dung de ve ban
do thu nho co to sang khoi tren slide.
"""
import os
import re
import shutil
import subprocess
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
THESIS_DIR = os.path.normpath(os.path.join(HERE, '..', 'thesis'))
OUT_DIR = os.path.join(HERE, 'figures')

FIGS = [  # (nhan trong thesis.tex, ten file PDF)
    ('fig:wmr_schematic', 'wmr_schematic'),
    ('fig:error_body_frame', 'error_body_frame'),
    ('fig:architecture', 'architecture'),
    ('fig:controller_full', 'controller_full'),
]

PREAMBLE = r"""\documentclass[12pt,border=2pt]{standalone}
\usepackage[utf8]{inputenc}
\usepackage[T5]{fontenc}
\usepackage{scrextend}
\changefontsizes{13pt}
\usepackage{amsmath, amssymb, amsfonts, bm, graphicx}
\usepackage{tikz}
\usetikzlibrary{arrows.meta, positioning, shapes.geometric, calc, decorations.pathreplacing, patterns, fit}
\usepackage{pgfplots}
\pgfplotsset{compat=1.18, legend cell align=left}
"""

BBOX_HOOK = (r'\path (current bounding box.south west); \pgfgetlastxy{\bbxa}{\bbya}%' '\n'
             r'\path (current bounding box.north east); \pgfgetlastxy{\bbxb}{\bbyb}%' '\n'
             r'\typeout{BBOX \bbxa\space\bbya\space\bbxb\space\bbyb}%' '\n')


def tikz_of(tex, label):
    """Doan \\begin{tikzpicture} ... \\end{tikzpicture} trong moi truong figure co \\label{label}."""
    k = tex.index('\\label{%s}' % label)
    a = tex.rindex('\\begin{figure}', 0, k)
    b = tex.index('\\begin{tikzpicture}', a)
    e = tex.index('\\end{tikzpicture}', b)
    assert e < k, label
    return tex[b:e], '\\end{tikzpicture}'


def macros(tex):
    """Cac macro toan hoc mot dong cua luan van (\\R, \\T, \\diag, ...)."""
    head = tex[:tex.index('\\begin{document}')]
    keep = []
    for line in head.splitlines():
        if re.match(r'\\(newcommand|DeclareMathOperator)\{\\[A-Za-z]+\}', line) and \
                'chapter' not in line and 'seclabel' not in line:
            keep.append(line)
    return '\n'.join(dict.fromkeys(keep))


def main():
    tex = open(os.path.join(THESIS_DIR, 'thesis.tex'), encoding='utf-8').read()
    aux = open(os.path.join(THESIS_DIR, 'thesis.aux'), encoding='utf-8').read()
    labels = '\n'.join(l for l in aux.splitlines() if l.startswith('\\newlabel{'))
    os.makedirs(OUT_DIR, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix='slidefig_')
    try:
        for label, name in FIGS:
            body, end = tikz_of(tex, label)
            hook = BBOX_HOOK if name == 'controller_full' else ''
            doc = (PREAMBLE + macros(tex) + '\n\\makeatletter\n' + labels + '\n\\makeatother\n'
                   '\\begin{document}\n' + body + hook + end + '\n\\end{document}\n')
            src = os.path.join(tmp, name + '.tex')
            open(src, 'w', encoding='utf-8').write(doc)
            r = subprocess.run(['pdflatex', '-interaction=nonstopmode', '-halt-on-error', name + '.tex'],
                               cwd=tmp, capture_output=True, text=True, encoding='utf-8', errors='replace')
            log = open(os.path.join(tmp, name + '.log'), encoding='utf-8', errors='replace').read()
            if r.returncode != 0 or not os.path.exists(os.path.join(tmp, name + '.pdf')):
                raise SystemExit('Loi khi bien dich %s:\n%s' % (name, log[-2000:]))
            shutil.copy(os.path.join(tmp, name + '.pdf'), os.path.join(OUT_DIR, name + '.pdf'))
            print('ok', name + '.pdf')
            if hook:
                m = re.search(r'BBOX ([-\d.]+)pt ([-\d.]+)pt ([-\d.]+)pt ([-\d.]+)pt', log)
                assert m, 'khong doc duoc khung bao'
                with open(os.path.join(OUT_DIR, 'controller_full_bbox.tex'), 'w', encoding='utf-8') as f:
                    f.write('%% Khung bao (pt) cua so do day du, do extract_figures.py ghi ra\n')
                    for key, val in zip(('mapSWx', 'mapSWy', 'mapNEx', 'mapNEy'), m.groups()):
                        f.write('\\def\\%s{%s}\n' % (key, val))
                    f.write('\\def\\mapBorder{2}\n')
                print('ok controller_full_bbox.tex', m.groups())
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    main()
