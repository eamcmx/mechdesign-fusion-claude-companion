# -*- coding: utf-8 -*-
"""Item balloons on an exploded view: the numbers of a parts list, each in a circle on the empty paper near its
part, with a leader to a point on the part.

    python tools/balloons.py ../book/vol2/figures/ch27_reducer_exploded_current.png [--width 133] [--preview out.png]

The picture's .json (written by tools/capture.py) gives each item's point on its part in pixels.
A balloon goes where its circle covers only paper (the light grid and the white), as near its point as it can, its
leader crossing no other leader or balloon and as little of the other parts as it can. The items with the fewest
places are placed first. The TikZ overlay is written beside the picture as <picture>_balloons.tex, in coordinates
of the picture's width and height, for a figure set at --width mm.
"""
import json
import math
import os
import sys

from PIL import Image, ImageChops, ImageDraw, ImageFilter

RADIUS_MM = 2.3                                   # the balloon's radius on the page
DISTANCES = (1.6, 2.2, 3.0, 4.0, 5.2, 6.6, 8.2, 10.0, 12.0)   # leader lengths tried, in balloon radii
DIRECTIONS = 36


def paper(im):
    """Not zero where the picture shows a part: dark or coloured pixels, thin lines left out, widened a little."""
    rgb = im.convert('RGB')
    dark = rgb.convert('L').point(lambda v: 255 if v < 200 else 0)
    coloured = rgb.convert('HSV').split()[1].point(lambda v: 255 if v > 45 else 0)
    busy = ImageChops.lighter(dark, coloured).filter(ImageFilter.MinFilter(5))   # thin lines (axes, trails) are paper
    return busy.filter(ImageFilter.MaxFilter(13)).load()


def crosses(p, q, r, s):
    """Do the segments pq and rs cross?"""
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return side(p, q, r) * side(p, q, s) < 0 and side(r, s, p) * side(r, s, q) < 0


def place(items, busy, W, H, rho):
    centre = (W / 2, H / 2)

    def disc_clear(b):
        if not (rho + 4 < b[0] < W - rho - 4 and rho + 4 < b[1] < H - rho - 4):
            return False
        for k in range(24):
            for f in (0.35, 0.7, 1.0):
                a = 2 * math.pi * k / 24
                if busy[int(b[0] + f * (rho + 6) * math.cos(a)), int(b[1] + f * (rho + 6) * math.sin(a))]:
                    return False
        return not busy[int(b[0]), int(b[1])]

    def over_parts(a, b):
        n = int(math.dist(a, b) / 4)
        hit = 0
        for i in range(n):
            t = i / max(1, n)
            p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if math.dist(p, a) > 2.5 * rho and busy[int(p[0]), int(p[1])]:
                hit += 1
        return hit

    cands = {}
    for k, (name, x, y) in items.items():
        a = (x, y)
        out = math.atan2(y - centre[1], x - centre[0])
        cs = []
        for f in DISTANCES:
            for j in range(DIRECTIONS):
                th = 2 * math.pi * j / DIRECTIONS
                b = (x + f * rho * math.cos(th), y + f * rho * math.sin(th))
                if disc_clear(b):
                    away = math.cos(th - out)
                    cs.append((f * rho + 6.0 * over_parts(a, b) - 0.6 * rho * away, b))
        cands[k] = sorted(cs)
    placed = {}
    for k in sorted(items, key=lambda k: len(cands[k])):
        a = tuple(items[k][1:])
        for cost, b in cands[k]:
            if any(math.dist(b, q) < 2 * rho + 10 for _, q in placed.values()):
                continue
            if any(crosses(a, b, p, q) for p, q in placed.values()):
                continue
            if any(seg_near_point(a, b, q, rho + 4) or seg_near_point(p, q, b, rho + 4) for p, q in placed.values()):
                continue
            placed[k] = (a, b)
            break
        else:
            raise SystemExit('no place for the balloon of item %s (%s)' % (k, items[k][0]))
    return placed


def seg_near_point(a, b, c, d):
    """Is the point c within d of the segment ab?"""
    ab = (b[0] - a[0], b[1] - a[1])
    t = max(0.0, min(1.0, ((c[0] - a[0]) * ab[0] + (c[1] - a[1]) * ab[1]) / (ab[0] ** 2 + ab[1] ** 2 or 1)))
    return math.dist(c, (a[0] + t * ab[0], a[1] + t * ab[1])) < d


def main(argv):
    png = argv[0]
    width = float(argv[argv.index('--width') + 1]) if '--width' in argv else 133.0
    d = json.load(open(png[:-4] + '.json', encoding='utf-8'))
    im = Image.open(png)
    W, H = im.size
    rho = RADIUS_MM * W / width
    placed = place({int(k): v for k, v in d['items'].items()}, paper(im), W, H, rho)
    lines = ['%% Balloons for %s, written by tools/balloons.py; the items are those of the parts list.' % os.path.basename(png),
             '%% Set at %.0f mm wide; coordinates are fractions of the picture\'s width and height.' % width]
    for k in sorted(placed):
        (ax, ay), (bx, by) = placed[k]
        lines.append('\\draw[leader] (%.4f,%.4f) -- (%.4f,%.4f);' % (ax / W, 1 - ay / H, bx / W, 1 - by / H))
    for k in sorted(placed):
        (ax, ay), (bx, by) = placed[k]
        lines.append('\\fill (%.4f,%.4f) circle (0.6pt); \\node[balloon] at (%.4f,%.4f) {%d};'
                     % (ax / W, 1 - ay / H, bx / W, 1 - by / H, k))
    tex = png[:-4] + '_balloons.tex'
    open(tex, 'w', encoding='utf-8', newline='\n').write('\n'.join(lines) + '\n')
    print('%d balloons -> %s' % (len(placed), tex))
    if '--preview' in argv:
        pv = im.convert('RGB')
        dr = ImageDraw.Draw(pv)
        for k, ((ax, ay), (bx, by)) in placed.items():
            dr.line([ax, ay, bx, by], fill=(0, 0, 0), width=3)
            dr.ellipse([bx - rho, by - rho, bx + rho, by + rho], fill=(255, 255, 255), outline=(0, 0, 0), width=3)
            dr.text((bx - 8, by - 8), str(k), fill=(0, 0, 0))
        pv.resize((W // 2, H // 2)).save(argv[argv.index('--preview') + 1])


if __name__ == '__main__':
    main(sys.argv[1:])
