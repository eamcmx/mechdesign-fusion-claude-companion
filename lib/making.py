"""Chapter 66: making the parts. The tine cut from plate (its outline, area and corner by hand, the plate Fusion nests
it on, the cut's length and time), why it is not bent from bar, and the census of the 168 made kinds by the process
that makes them, sorted from what Fusion measured of each (scripts/Making census(), results/making_census.json).

The tine P13-012 (Chapter 47's bag gripper, lib/bag_gripper.py's LAY): a leg 25 wide running 250 down from the jaw's
tube and a foot 25 deep running 255 in under the bag, 10 thick, sixteen of them (eight a jaw). Its L lies in the plane
of its 25 mm depth, so a bend from 25 x 10 bar would be a bend about the 10 mm side, in the bar's own plane: Fusion's
sheet metal bends about a line in the sheet, never in it, so the tine has no bend to make. Turned flat, the bar would
bend the easy way but give a tine 10 deep under the bag instead of 25.

Lengths mm, areas mm2, forces N, times s unless named."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import bag_gripper as B          # noqa: E402

RHO_1020 = 7870.0                # kg/m3, Fusion's 'Steel AISI 1020 107 HR' (lib/materials_fits.py FUSION_DENSITY)
# the tine's inside corner, the plate it is nested on and the gaps: r is chosen (below); the spacing between parts and
# at the plate's edge is the plate's thickness, assumed until the shop names its machine; the plate a 1000 x 2000 sheet
TINE = dict(r=10.0, n=16, t=10.0, spacing=10.0, frame=10.0, plate=(2000.0, 1000.0))


def tine(r=None, lay=B.LAY, bag=B.BAG):
    """The tine's flat L from the layout: its width w (the leg's), depth h (the foot's), the outline W x H, the area,
    volume and outline's length with the inside corner rounded to r, and how far the round stays from the bag."""
    r = TINE['r'] if r is None else r
    w = lay['leg_y'][1] - lay['leg_y'][0]                 # 25, the leg's width
    h = lay['foot_z'][1] - lay['foot_z'][0]               # 25, the foot's depth
    W = lay['leg_y'][1] - lay['tip_y']                    # 255, the foot's length to the tip
    H = lay['leg_top'] - lay['foot_z'][0]                 # 275, the tine's height
    t = lay['tine_t']
    A0 = W * h + w * (H - h)
    A = A0 + (1 - math.pi / 4) * r * r                    # the round adds the corner's spandrel
    P0 = 2 * (W + H)                                      # an L's outline is its box's
    P = P0 - (2 - math.pi / 2) * r                        # two straight r replaced by a quarter circle
    inner = lay['leg_y'][0]                               # the leg's inner face, 270
    side = bag['w'] / 2                                   # the bag's side, 250
    seat = lay['foot_z'][1]                               # the foot's top, -480
    return dict(w=w, h=h, W=W, H=H, t=t, r=r, A0=A0, A=A, V=A * t, m=A * t * RHO_1020 * 1e-9, P0=P0, P=P,
                round_from=inner - r, clear=inner - r - side, seat=seat, bag_bottom=lay['bag_bottom'],
                rounded_rise=r)


def tine_bent():
    """The tine bent from 25 x 10 flat bar the easy way (about a line across its 25 mm face): its L then lies in the
    plane of the 10 mm, so the foot carries the bag on a depth of 10 instead of 25. The foot's section modulus and
    second moment either way, with Chapter 47's load (lib/bag_gripper.py design()['tines'])."""
    d = B.design()['tines']
    b_, h_ = 10.0, 25.0                                  # drawn: 10 thick, 25 deep under the load
    W_drawn, I_drawn = b_ * h_ ** 2 / 6, b_ * h_ ** 3 / 12
    W_bent, I_bent = h_ * b_ ** 2 / 6, h_ * b_ ** 3 / 12  # bent flat: 25 wide, 10 deep
    return dict(W_drawn=W_drawn, W_bent=W_bent, I_drawn=I_drawn, I_bent=I_bent, stress_ratio=W_drawn / W_bent,
                stiffness_ratio=I_drawn / I_bent, M_foot=d['M_foot'], s_drawn=d['M_foot'] * 1e3 / W_drawn,
                s_bent=d['M_foot'] * 1e3 / W_bent, s_leg=d['s_leg'], Sy=d['Sy'])


def nest(used, n=None, r=None):
    """Fusion's nest: the rectangle the n tines take (used, mm, without the frame), the plate's piece with its frame,
    and the share of each that is tine."""
    n = TINE['n'] if n is None else n
    tt = tine(r)
    a_used = used[0] * used[1]
    piece = (used[0] + 2 * TINE['frame'], used[1] + 2 * TINE['frame'])
    a_piece = piece[0] * piece[1]
    plate = TINE['plate'][0] * TINE['plate'][1]
    return dict(n=n, area=n * tt['A'], used=used, a_used=a_used, eff_used=n * tt['A'] / a_used, piece=piece,
                a_piece=a_piece, eff_piece=n * tt['A'] / a_piece, eff_plate=n * tt['A'] / plate,
                per_plate=int(plate // a_piece), mass_piece=a_piece * TINE['t'] * RHO_1020 * 1e-9,
                mass_parts=n * tt['m'])


def cut_time(feed, pierce, n=None, r=None):
    """The time to cut n outlines at a feed (mm/min) with one pierce each (s), lead-ins and moves left out: the
    lower bound Fusion's own time must sit above."""
    n = TINE['n'] if n is None else n
    L = n * tine(r)['P']
    return dict(L=L, cutting=L / feed * 60.0, piercing=n * pierce, total=L / feed * 60.0 + n * pierce)


# ---- the census by process ------------------------------------------------------------------------------------------
# Each made kind gets a route (how it comes to be) and a shape (what the machining is), sorted from Fusion's census
# (results/making_census.json) and the book's own words:
#   the aircraft's   the flap's panel and the wing's skins and spar web, drawn as the brief's section so the actuator
#                    has something to move and stand on (Volume 2, Chapter 34): the airframe maker's, not ours
#   moulded          polyurethane and rubber
#   cast             cast iron (Chapter 51's material for the axle's carrier, caps and differential case)
#   welded           a weldment's pieces, and the 1020 parts: Volume 2 named 1020 hot rolled for the robot's welded
#                    arm boxes and their plates (Chapter 43) and the tines welded to the jaw's tube (Chapters 47, 55);
#                    and the axle's banjo, named "pressed and welded" (Chapter 30)
#   gear             a toothed part, by its name (z and a tooth count, bevel gear, crown wheel, side gear, the
#                    differential's spiders): blank turned, teeth cut, then hardened as its material says
#   machined         the rest, from bar, plate or block
# Shapes, from the faces Fusion measured (tolerances 1e-4 on directions, 0.01 mm on lengths):
#   turned           every face a cylinder, cone, torus or sphere on one axis, or a plane square to it, and the
#                    smallest box no wider across that axis than the largest such cylinder
#   turned, milled   the box no wider than that cylinder and at least half the area on the axis, but some faces off
#                    it (flats, keyseats, cross holes)
#   plate profile    every plane square to or along the smallest box's thinnest side, every cylinder along it, and the
#                    planes along it only at its two faces: an outline through the plate's thickness
#   milled 2.5D      every plane and cylinder square to or along one of the box's three sides, at any depth
#   3D               anything else (cones, tori or curved faces off those sides, faces at an angle)
AIRFRAME = ('F01-005', 'F01-016', 'F01-019', 'F01-024')
# the lower arm's pieces Volume 2 welds (Chapter 43: the fork plates to the shoulder's hub bosses and to the box, the box
# to the elbow's hub) that Chapter 51 gave 1045 by default; this chapter makes them 1020 by Chapter 55's rule
ARM_WELDED = ('P01-020', 'P01-024', 'P01-025', 'P01-017')
# the robot's rod bodies are 1020 too, but nothing in Volumes 2 and 3 names how their eyes are held to them: not taken
# as welded (an open question, Chapter 66)
ROD_BODIES = ('P01-033', 'P01-036', 'P01-040')
TURNED_SHARE = 0.5          # at least half the area on the axis for 'turned, milled'
GEAR = r'\bz\d+\b|bevel gear|crown wheel|side gear$|^spider$'
TOL_DIR, TOL_LEN = 1e-4, 0.01


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _par(a, b):
    return abs(abs(_dot(a, b)) - 1.0) < TOL_DIR


def _perp(a, b):
    return abs(_dot(a, b)) < TOL_DIR


def _off_line(p, o, ax):
    d = [p[i] - o[i] for i in range(3)]
    k = _dot(d, ax)
    return math.sqrt(max(0.0, _dot(d, d) - k * k))


def turned(body):
    """The best axis of revolution: its share of the body's area (faces on it, planes square to it), the largest
    cylinder on it, and whether the smallest box fits inside that cylinder across the axis."""
    fs = body['faces']
    A = sum(f['a'] for f in fs)
    best = dict(share=0.0, R=0.0, fits=False, axis=None)
    seen = []
    for c in fs:
        if c['t'] not in ('cylinder', 'cone', 'torus'):
            continue
        o, ax = c['o'], c['ax']
        if any(_par(ax, a2) and _off_line(o, o2, a2) < TOL_LEN for o2, a2 in seen):
            continue
        seen.append((o, ax))
        on, R = 0.0, 0.0
        for f in fs:
            if f['t'] in ('cylinder', 'cone', 'torus') and _par(f['ax'], ax) and _off_line(f['o'], o, ax) < TOL_LEN:
                on += f['a']
                if f['t'] == 'cylinder':
                    R = max(R, f['r'])
            elif f['t'] == 'sphere' and _off_line(f['o'], o, ax) < TOL_LEN:
                on += f['a']
            elif f['t'] == 'plane' and _par(f['n'], ax):
                on += f['a']
        across = [d for d, v in zip(body['box'], body.get('box_dirs', [None] * 3)) if v is None or not _par(v, ax)]
        along = [v for v in body.get('box_dirs', []) if _par(v, ax)]
        fits = bool(along) and len(across) == 2 and max(across) <= 2 * R + TOL_LEN
        share = on / A
        if (fits, share) > (best['fits'], best['share']):
            best = dict(share=share, R=R, fits=fits, axis=ax)
    return best


def prismatic(body):
    """Whether every face is square to or along one of the smallest box's sides; and, along its thinnest side, whether
    the body is an outline through its thickness (planes along it only at its two faces)."""
    dirs = body.get('box_dirs')
    if not dirs:
        return dict(prism=False, plate=False, t=body['box'][0])
    fs = body['faces']
    prism = True
    for f in fs:
        if f['t'] == 'plane':
            ok = any(_par(f['n'], d) for d in dirs)
        elif f['t'] == 'cylinder':
            ok = any(_par(f['ax'], d) for d in dirs)
        else:
            ok = False
        prism = prism and ok
    d, t, c = dirs[0], body['box'][0], body['box_centre']
    plate = prism
    for f in fs:
        if f['t'] == 'plane' and _par(f['n'], d):
            h = abs(_dot([f['o'][i] - c[i] for i in range(3)], d))
            plate = plate and abs(h - t / 2) < TOL_LEN
        elif f['t'] == 'plane':
            plate = plate and _perp(f['n'], d)
        elif f['t'] == 'cylinder':
            plate = plate and _par(f['ax'], d)
    return dict(prism=prism, plate=plate, t=t)


def shape(body):
    tu = turned(body)
    if tu['fits'] and tu['share'] > 1 - 1e-6:
        return 'turned', dict(D=2 * tu['R'])
    if tu['fits'] and tu['share'] >= TURNED_SHARE:
        return 'turned, milled', dict(D=2 * tu['R'], share=tu['share'])
    pr = prismatic(body)
    if pr['plate']:
        return 'plate profile', dict(t=pr['t'])
    if pr['prism']:
        return 'milled 2.5D', dict(t=pr['t'])
    return '3D', {}


def route(kind, named=None):
    import re
    d, mat = kind['description'], (kind['material'] or '')
    if kind['number'] in AIRFRAME:
        return "the aircraft's"
    if re.search(r'polyurethane|rubber', mat, re.I):
        return 'moulded'
    if re.search(r'cast iron', mat, re.I):
        return 'cast'
    welded_1020 = re.search(r'1020', mat) and kind['number'] not in ROD_BODIES
    if kind['weldment'] or welded_1020 or kind['number'] in ARM_WELDED or (named and re.search(r'weld', named, re.I)):
        return 'welded'
    if re.search(GEAR, d, re.I):
        return 'gear'
    return 'machined'


def named_materials():
    import io
    import json
    d = json.load(io.open(os.path.join(HERE, 'named_materials.json'), encoding='utf-8'))
    return {(mk, k.lower()): v['material'] for mk, parts in d.items() for k, v in parts.items()}


def census(path=None):
    """The 168 made kinds, each with its route and its shape (a weldment by its pieces), and the totals by route and by
    shape: kinds, pieces and kg (the pieces' and masses from results/bom.json)."""
    import io
    import json
    import re
    res = os.path.join(os.path.dirname(HERE), 'results')
    data = json.load(io.open(path or os.path.join(res, 'making_census.json'), encoding='utf-8'))
    bom = json.load(io.open(os.path.join(res, 'bom.json'), encoding='utf-8'))
    mass = {(m['key'], l['number']): l['book'] for m in bom['machines'] for l in m['lines'] if l['kind'] == 'made'}
    named = named_materials()
    rows = []
    for k in data['kinds']:
        nm = named.get((k['machine'], re.sub(r',.*$', '', k['description']).strip().lower()))
        r = route(k, nm)
        shapes = [(b.get('piece'), ) + shape(b) for b in k['bodies']]
        rows.append(dict(machine=k['machine'], number=k['number'], description=k['description'], qty=k['qty'],
                         material=k['material'], named=nm, route=r, shapes=shapes, shape=shapes[0][1] if len(shapes) == 1 else 'weldment',
                         kg=mass.get((k['machine'], k['number']), 0.0), bodies=len(k['bodies'])))
    tot = {}
    for key in ('route', 'shape'):
        t = {}
        for r in rows:
            g = t.setdefault(r[key], dict(kinds=0, pieces=0, kg=0.0))
            g['kinds'] += 1
            g['pieces'] += r['qty']
            g['kg'] += r['kg']
        tot[key] = t
    return dict(rows=rows, by=tot, kinds=len(rows), pieces=sum(r['qty'] for r in rows), kg=sum(r['kg'] for r in rows))


# ---- the casting's allowances ---------------------------------------------------------------------------------------
# ISO 8062:1994's required machining allowance (RMA), mm on each machined face, by the raw casting's largest dimension,
# grades F, G, H (technologicalprocess.com after PN-ISO 8062:1997, Table 2; Mechanical Design References/Web pages/
# TechnologicalProcess_ISO8062_machining_allowances_2026-11-07.md). Grey iron in sand: E to G in series, F to H for a
# few castings moulded by hand; G is in both, and the carrier takes it.
RMA = {(40, 63): (0.5, 0.7, 1.0), (63, 100): (1.0, 1.4, 2.0), (100, 160): (1.5, 2.2, 3.0), (160, 250): (2.0, 2.8, 4.0),
       (250, 400): (2.5, 3.5, 5.0), (400, 630): (3.0, 4.0, 6.0)}
RMA_GRADE = 'G'


def rma(largest, grade=RMA_GRADE):
    for (lo, hi), row in RMA.items():
        if lo < largest <= hi:
            return row['FGH'.index(grade)]
    raise ValueError('no RMA row for %g mm' % largest)


def rma_raw(finished):
    """The RMA by the raw casting's largest dimension, which is the finished part's plus the allowance at each end:
    taken until it no longer changes the row."""
    a = rma(finished)
    while rma(finished + 2 * a) != a:
        a = rma(finished + 2 * a)
    return a


def casting(largest, face=82.0, grade=10):
    """A machined face of a casting: the raw casting's dimension carries the RMA on each machined face and the CT
    tolerance about it (Chapter 61's CT10 for machine-moulded sand at the legs' 82 mm, lib/stackups.py); the metal the
    cutter meets runs from RMA - CT/2 to RMA + CT/2."""
    import stackups as S
    a = rma_raw(largest)
    ct = S.ct(face, grade)
    return dict(largest=largest, raw=largest + 2 * a, grade=RMA_GRADE, rma=a, rma_finished=rma(largest), ct=ct, face=face,
                ct_grade=grade, least=a - ct / 2, most=a + ct / 2)


def castings(path=None):
    """The three castings' largest finished dimensions from the census, and each one's allowance."""
    c = census(path)
    out = {}
    for r in c['rows']:
        if r['route'] == 'cast':
            import io
            import json
            res = os.path.join(os.path.dirname(HERE), 'results')
            k = [k for k in json.load(io.open(path or os.path.join(res, 'making_census.json'), encoding='utf-8'))['kinds']
                 if k['number'] == r['number']][0]
            out[r['number']] = dict(description=r['description'], **casting(max(k['bodies'][0]['box'])))
    return out


# ---- what earlier chapters handed to this one -----------------------------------------------------------------------
def handovers():
    """The numbers the shop floor needs, each from the library that worked it out."""
    import axle_detail as AD
    import dfma as DF
    import drawings_detail as DD
    import fatigue_detail as FD
    import wrist_design as W
    sp, nut, bolts, sleeve, tw = DD.spacer(), AD.pinion_stack(), FD.tightening(), DF.sleeve_thread(), DF.tine_weld()
    pad = W.stops40()
    return dict(spacer=sp, nut=dict(F=nut['F'], T=nut['T'], sigma=nut['sigma']), bolts=bolts,
                sleeve=dict(T=sleeve['T'], d=sleeve['d'], pitch=sleeve['pitch'], n=sleeve['n']),
                tine_weld=dict(h=tw['h'], length=tw['weld_length'], welds=tw['welds'], n_weld=tw['n_weld'], leg=tw['leg_stop']),
                pad=dict(E=pad['E'], F=pad['F'], travel=W.STOP40['travel'], t=W.STOP40['pad'], A=pad['pad'], p=pad['p'],
                         k=pad['F'] / W.STOP40['travel']),
                minor=30.0 - 1.0825 * 1.5)


if __name__ == '__main__':
    for k, v in tine().items():
        print('tine', k, round(v, 4) if isinstance(v, float) else v)
    print('sharp', {k: round(v, 4) for k, v in tine(0.0).items() if isinstance(v, float)})
    print('bent', {k: round(v, 4) for k, v in tine_bent().items()})
    print('nest', nest((472.6, 953.3)))
    print('cut at 10 m/min, 1 s', cut_time(10000.0, 1.0))
