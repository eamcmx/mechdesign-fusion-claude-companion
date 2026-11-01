# -*- coding: utf-8 -*-
"""Pictures for the book: build a design with a capture script, colour it, and save views of it.

    python tools/capture.py captures/first_assembly.py ../book/figures/ch02_first_assembly [iso-top-right top ...] [--dark 235] [--size 3200x2000]
        -> ch02_first_assembly_iso-top-right.png, ... (3200 x 2000 before the empty grid is cut away)

A capture script is a Fusion script (run(_context)) that builds the design to show; __COMPANION__ is replaced as in
run_check.py. Views: current, front, back, top, bottom, left, right, iso-top-right, iso-top-left, ...
After the pictures the design is closed unsaved (python tools/run_check.py --cleanup).
If the script prints a line 'ANCHORS {json}' (the view's camera, the model's extent on it and points of the parts in mm
across and up, as Chapter 27's exploded view does), the picture is cut to the model's extent and the points, now in
pixels of the picture, are kept in <picture>.json for the balloons of the figure (tools/balloons.py).
"""
import io
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import run_check  # noqa: E402


def crop(png, margin=80, dark=170, box=None):
    """Cut the empty grid away: keep the box round the pixels darker than `dark` (the model, not the light grid), or
    the box given. Returns the box cut."""
    from PIL import Image, ImageFilter
    im = Image.open(png).convert('RGB')
    if box is None:
        g = im.convert('L').point(lambda v: 255 if v < dark else 0)
        box = g.filter(ImageFilter.MinFilter(7)).getbbox()     # thin axis and grid lines drop out, the model stays
    if box:
        x0, y0, x1, y1 = [int(round(c)) for c in box]
        cut = (max(0, x0 - margin), max(0, y0 - margin), min(im.width, x1 + margin), min(im.height, y1 + margin))
        im.crop(cut).save(png)
        return cut


def shot(png, view, w=3200, h=2000, dark=170, box=None):
    r = subprocess.run([sys.executable, run_check.MCP, 'shot', png, view, str(w), str(h)], capture_output=True,
                       encoding='utf-8', errors='replace', timeout=600)
    cut = crop(png, dark=dark, box=box)
    return (r.stdout + r.stderr).strip()[-160:], cut


def projected(A, w, h):
    """Points given on the view in mm (right, up) to pixels of a w x h picture: an orthographic view is centred on
    the camera's target and shows a height equal to the camera's viewExtents (found by drawing the points)."""
    s = h / A['radius']
    ut, vt = A['target']
    return lambda u, v: (w / 2 + s * (u - ut), h / 2 - s * (v - vt))


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    args = sys.argv[1:]
    dark, w, h = 170, 3200, 2000
    if '--dark' in args:
        i = args.index('--dark'); dark = int(args[i + 1]); del args[i:i + 2]
    if '--size' in args:
        i = args.index('--size'); w, h = [int(c) for c in args[i + 1].split('x')]; del args[i:i + 2]
    script, stem = args[0], args[1]
    views = args[2:] or ['iso-top-right']
    src = io.open(os.path.join(ROOT, script), encoding='utf-8').read().replace('__COMPANION__', ROOT.replace('\\', '/'))
    out = run_check.send(src)
    print(out)
    try:
        anchors = [json.loads(ln[8:]) for ln in out.splitlines() if ln.startswith('ANCHORS ')]
        for v in views:
            png = os.path.abspath(stem + '_' + v + '.png')
            box, px = None, None
            if anchors:                                  # the model's extent on the view, not its dark pixels
                A = anchors[0]
                px = projected(A, w, h)
                umin, umax, vmin, vmax = A['extent']
                box = px(umin, vmax) + px(umax, vmin)
            msg, cut = shot(png, v, w, h, dark=dark, box=box)
            print(msg)
            if anchors and cut:
                items = {k: [name] + [c - o for c, o in zip(px(*uv), cut[:2])] for k, (name, uv) in A['items'].items()}
                io.open(png[:-4] + '.json', 'w', encoding='utf-8').write(json.dumps(dict(size=[cut[2] - cut[0], cut[3] - cut[1]], items=items), indent=1))
    finally:                                         # the design is closed even if a picture fails
        print(run_check.send(run_check.CLEANUP))
