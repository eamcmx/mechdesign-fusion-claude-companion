# -*- coding: utf-8 -*-
"""Pictures for the book: build a design with a capture script, colour it, and save views of it.

    python tools/capture.py captures/first_assembly.py ../book/figures/ch02_first_assembly [iso-top-right top ...] [--dark 235]
        -> ch02_first_assembly_iso-top-right.png, ... (1600 x 1000)

A capture script is a Fusion script (run(_context)) that builds the design to show; __COMPANION__ is replaced as in
run_check.py. Views: current, front, back, top, bottom, left, right, iso-top-right, iso-top-left, ...
After the pictures the design is closed unsaved (python tools/run_check.py --cleanup).
"""
import io
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import run_check  # noqa: E402


def crop(png, margin=80, dark=170):
    """Cut the empty grid away: keep the box round the pixels darker than `dark` (the model, not the light grid)."""
    from PIL import Image, ImageFilter
    im = Image.open(png).convert('RGB')
    g = im.convert('L').point(lambda v: 255 if v < dark else 0)
    box = g.filter(ImageFilter.MinFilter(7)).getbbox()     # thin axis and grid lines drop out, the model stays
    if box:
        x0, y0, x1, y1 = box
        im.crop((max(0, x0 - margin), max(0, y0 - margin), min(im.width, x1 + margin), min(im.height, y1 + margin))).save(png)


def shot(png, view, w=3200, h=2000, dark=170):
    r = subprocess.run([sys.executable, run_check.MCP, 'shot', png, view, str(w), str(h)], capture_output=True,
                       encoding='utf-8', errors='replace', timeout=600)
    crop(png, dark=dark)
    return (r.stdout + r.stderr).strip()[-160:]


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    args = sys.argv[1:]
    dark = 170
    if '--dark' in args:
        i = args.index('--dark'); dark = int(args[i + 1]); del args[i:i + 2]
    script, stem = args[0], args[1]
    views = args[2:] or ['iso-top-right']
    src = io.open(os.path.join(ROOT, script), encoding='utf-8').read().replace('__COMPANION__', ROOT.replace('\\', '/'))
    print(run_check.send(src))
    for v in views:
        print(shot(os.path.abspath(stem + '_' + v + '.png'), v, dark=dark))
    print(run_check.send(run_check.CLEANUP))
