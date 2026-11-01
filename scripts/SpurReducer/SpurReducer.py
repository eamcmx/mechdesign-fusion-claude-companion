# =============================================================================
# SpurReducer.py  -  Chapter 27 of "Mechanical Design in Autodesk Fusion with Claude": a single-stage spur reducer
# designed by subsystems. The sizing (lib/reducer_design.py) decides every part along the load path; this script
# refuses a duty the chosen parts cannot carry, then builds the machine as three subassemblies of real parts from the
# parts library (lib/mdfc_parts.py): the housing with its caps, seals, bearing outer rings and fasteners, and the two
# shaft assemblies, each with its shaft, gear, keys, spacer and bearing inner rings. Written from PROMPT.txt beside it;
# checked by checks/spur_reducer_check.py.
#
#   INPUTS        the dialog: power, input speed, application factor, bearing life, design factor
#   DERIVED       reducer_design.design(): gears, shafts, bearings, keys; the layout below
#   HARD RULES    build() refuses a duty for which a shaft's factor of safety, a key's stresses, a bearing's life,
#                 a slope or the gear's deflection falls short at the chosen sizes, saying which
#   CONVENTIONS   shafts along y, the split plane z = 0, the input shaft at x = 0 and the output at x = a; every
#                 part a component of its subassembly; the shaft assemblies turn on As-built revolute joints at
#                 their bearings, linked at the gear ratio
#   ACCEPTANCE    the message and its parts list; the numbers recomputed; no interference at rest or through one
#                 pinion pitch; the ratio measured; the parts list from Fusion's components
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

LIB = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import reducer_design as rd     # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(rd)
importlib.reload(mp)

VERSION = 'SpurReducer 1.0 (2026-11-01)'
CM = 0.1
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
Y_IN, Y_OUT, CAP_T = 42.0, 60.0, 8.0          # walls from y = 42 to 60 mm each side; caps 8 mm outside them
WALL, FLANGE_T, FLANGE_W = 10.0, 15.0, 20.0     # housing walls; the split flange's half-thickness and width
BACKLASH = 0.2
SEALS = {20: 35, 30: 47}                        # lip seals d x D x 7 (DIN 3760 range), shaft d: outside D
SH_MIN = 1.1                                    # the least contact factor accepted at concept level


def layout(d):
    """Positions along y and the housing's cavity, from the design."""
    g, B = d['gears'], d['bearings']
    L = dict(a=g['a'], ra1=g['d1'] / 2 + g['m'], ra2=g['d2'] / 2 + g['m'], rf1=g['d1'] / 2 - 1.25 * g['m'],
             rf2=g['d2'] / 2 - 1.25 * g['m'], b1=B['in']['B'], b2=B['out']['B'])
    L['cav'] = (-L['ra1'] - 13.5, L['a'] + L['ra2'] + 12.0, -(L['ra2'] + 30.0), L['ra2'] + 12.0)   # x0, x1, z0, z1
    L['span1'] = 2 * Y_IN + L['b1']                 # bearing centres
    L['span2'] = 2 * Y_IN + L['b2']
    return L


def check_duty(d):
    """Every sizing check at the chosen sizes; a list of what falls short."""
    b, g, s, B, K = d['brief'], d['gears'], d['shafts'], d['bearings'], d['keys']
    short = []
    for k, name in (('1', 'pinion'), ('2', 'gear')):
        if g['SF' + k] < b['nd']:
            short.append('the %s\'s teeth have a bending factor of %.2f '
                         '(%.1f needed)' % (name, g['SF' + k], b['nd']))
        if g['SH' + k] < SH_MIN:
            short.append('the %s\'s flanks have a contact factor of %.3f '
                         '(%.1f needed)' % (name, g['SH' + k], SH_MIN))
    for k, name in (('n_in', 'the input end'), ('n_gear', 'the gear seat'), ('n_out', 'the output end')):
        if s[k] < b['nd']:
            short.append('%s has a factor of safety of %.2f (%.1f needed)' % (name, s[k], b['nd']))
    for k, v in K.items():
        if not v['ok']:
            short.append('the %s key bears %.1f MPa and shears %.1f MPa (100 and 60 allowed)' % (k, v['sigma'], v['tau']))
    for k, v in B.items():
        if v['L10h'] < b['life']:
            short.append('bearing %s lasts %.0f h (%.0f h needed)' % (v['name'], v['L10h'], b['life']))
    if max(s['theta1'], s['theta2']) > 0.001:
        short.append('a bearing slope of %.5f rad (0.001 allowed)' % max(s['theta1'], s['theta2']))
    if max(s['y1'], s['y2']) > s['y_allow']:
        short.append('a gear deflection of %.4f mm (%.3f allowed)' % (max(s['y1'], s['y2']), s['y_allow']))
    return short


# ---------- the housing subassembly ----------
def housing(root, d, L):
    asm = mp.new_occ(root, 'Housing')
    x0, x1, z0, z1 = L['cav']
    X0, X1, Z0, Z1 = x0 - WALL, x1 + WALL, z0 - WALL, z1 + WALL
    bores = [(0.0, d['bearings']['in']['D'] / 2), (L['a'], d['bearings']['out']['D'] / 2)]
    caps = [(bx, r + 15.0) for bx, r in bores]                       # the caps' flanges sit on the walls here
    bolts = [(x, y) for x in (X0 - 7.0, L['a'] * 0.48, L['a'] + 100.0, X1 + 7.0) for y in (-70.0, 70.0)]
    dowels = [(X0 - 10.0, 0.0), (X1 + 10.0, 0.0)]

    def half(lo, hi, name):
        h = mp.box(X0, X1, -Y_OUT, Y_OUT, lo, hi)
        fz0, fz1 = (max(lo, -FLANGE_T), min(hi, FLANGE_T))
        mp.union(h, mp.box(X0 - FLANGE_W, X1 + FLANGE_W, -Y_OUT - FLANGE_W, Y_OUT + FLANGE_W, fz0, fz1))
        for bx, rc in caps:                                          # the flange stops short of each cap
            for s in (-1, 1):
                mp.cut(h, mp.cyl((bx, s * Y_OUT, 0), (bx, s * (Y_OUT + FLANGE_W + 1), 0), rc + 2.0))
        if lo < 0:                                                   # the base's feet
            mp.union(h, mp.box(X0, X1, -110.0, 110.0, Z0 - 14.5, Z0 + 0.001))
            for fx in (X0 + 25.0, X1 - 25.0):
                for fy in (-95.0, 95.0):
                    mp.cut(h, mp.cyl((fx, fy, Z0 - 16), (fx, fy, Z0 + 1), 7.0))
        mp.cut(h, mp.box(x0, x1, -Y_IN, Y_IN, max(lo, z0), min(hi, z1)))
        for bx, r in bores:
            mp.cut(h, mp.cyl((bx, -Y_OUT - 1, 0), (bx, Y_OUT + 1, 0), r))
            for s in (-1, 1):                                        # tapped holes for the caps' screws, M6
                for k in range(4):
                    a = math.radians(45 + 90 * k)
                    px, pz = bx + (r + 7.5) * math.cos(a), (r + 7.5) * math.sin(a)
                    if lo <= pz <= hi:
                        mp.cut(h, mp.cyl((px, s * (Y_OUT + 1), pz), (px, s * (Y_OUT - 14), pz), 3.0))
        for bx, by in bolts:
            mp.cut(h, mp.cyl((bx, by, -FLANGE_T - 1), (bx, by, FLANGE_T + 1), 5.5))
        for dx, dy in dowels:
            mp.cut(h, mp.cyl((dx, dy, -FLANGE_T - 1), (dx, dy, FLANGE_T + 1), 4.0))
        return h

    base = half(Z0 - 14.5, 0.0, 'Base')
    mp.cut(base, mp.cyl((X1 + 1, 0.0, z0 + 10.0), (x1 - 1, 0.0, z0 + 10.0), 8.0))      # the drain
    cover = half(0.0, Z1, 'Cover')
    mp.cut(cover, mp.cyl((L['a'] * 0.5, 0.0, z1 - 1), (L['a'] * 0.5, 0.0, Z1 + 1), 8.0))   # the filler
    parts = {}
    for name, tb in (('Housing base', base), ('Housing cover', cover)):
        o = mp.new_occ(asm.component, name)
        mp.add_body(o, tb, name, 'Iron, Cast')
        parts[name] = o
    plug = mp.cyl((x1 + 1.0, 0.0, z0 + 10.0), (X1 + 0.001, 0.0, z0 + 10.0), 8.0)
    mp.union(plug, mp.cyl((X1, 0.0, z0 + 10.0), (X1 + 8.0, 0.0, z0 + 10.0), 12.0))
    o = mp.new_occ(asm.component, 'Drain plug M16')
    mp.add_body(o, plug, 'Drain plug M16', 'Steel')
    plug = mp.cyl((L['a'] * 0.5, 0.0, z1 + 1.0), (L['a'] * 0.5, 0.0, Z1 + 0.001), 8.0)
    mp.union(plug, mp.cyl((L['a'] * 0.5, 0.0, Z1), (L['a'] * 0.5, 0.0, Z1 + 8.0), 12.0))
    o = mp.new_occ(asm.component, 'Filler plug M16')
    mp.add_body(o, plug, 'Filler plug M16', 'Steel')

    # the bearings' outer rings and balls, the caps that hold them, the seals and the screws
    shafts = (('in', 0.0, d['shafts']['d']['d_seat1'], -1), ('out', L['a'], d['shafts']['d']['d_seat2'], 1))
    for key, bx, ds, drive in shafts:
        br = d['bearings'][key]
        for tag, y0 in (('A', -Y_IN - br['B']), ('B', Y_IN)):
            inner, outer, balls, geo = mp.bearing_bodies(ds, br['D'], br['B'], y0, bx, 0.0)
            for part, tb in (('outer ring', outer), ('balls', balls)):
                nm = 'Bearing %s %s %s' % (br['name'], tag, part)
                o = mp.new_occ(asm.component, nm)
                mp.add_body(o, tb, nm, 'Steel')
        for s in (-1, 1):
            through = (s == drive)
            r = br['D'] / 2
            face = s * (Y_IN + br['B'])                               # the bearing's outer face
            cap = mp.cyl((bx, s * Y_OUT, 0), (bx, s * (Y_OUT + CAP_T), 0), r + 15.0)
            mp.union(cap, mp.tube((bx, face, 0), (bx, s * Y_OUT + s * 0.001, 0), r, geo['out_id'] / 2 + 0.3))
            if through:
                D_seal = SEALS[int(ds)]
                mp.cut(cap, mp.cyl((bx, s * (Y_OUT + CAP_T + 1), 0), (bx, s * (Y_OUT + 1), 0), D_seal / 2))
                mp.cut(cap, mp.cyl((bx, s * (Y_OUT + 2), 0), (bx, face - s, 0), ds / 2 + 1.0))
            else:
                mp.cut(cap, mp.cyl((bx, face - s, 0), (bx, s * Y_OUT, 0), geo['out_id'] / 2 + 0.3))
            for k in range(4):
                a = math.radians(45 + 90 * k)
                px, pz = bx + (r + 7.5) * math.cos(a), (r + 7.5) * math.sin(a)
                mp.cut(cap, mp.cyl((px, s * (Y_OUT - 1), pz), (px, s * (Y_OUT + CAP_T + 1), pz), 3.3))
            nm = 'Cap %s %s' % ('input' if key == 'in' else 'output', 'through' if through else 'blind')
            o = mp.new_occ(asm.component, nm)
            mp.add_body(o, cap, nm, 'Iron, Cast')
            if through:
                D_seal = SEALS[int(ds)]
                seal = mp.tube((bx, s * (Y_OUT + 1), 0), (bx, s * (Y_OUT + CAP_T), 0), D_seal / 2, ds / 2)
                nm = 'Lip seal %dx%dx7' % (ds, D_seal)
                o = mp.new_occ(asm.component, nm)
                mp.add_body(o, seal, nm)
            for k in range(4):
                a = math.radians(45 + 90 * k)
                px, pz = bx + (r + 7.5) * math.cos(a), (r + 7.5) * math.sin(a)
                sc = mp.cap_screw(6.0, 16.0, (px, s * (Y_OUT + CAP_T), pz), (0, -s, 0))
                nm = 'Cap screw M6x16 %s%s%d' % (key, 'L' if s < 0 else 'R', k + 1)
                o = mp.new_occ(asm.component, nm)
                mp.add_body(o, sc, nm, 'Steel')
    for k, (bx, by) in enumerate(bolts):
        o = mp.new_occ(asm.component, 'Screw M10x45 %d' % (k + 1))
        mp.add_body(o, mp.cap_screw(10.0, 45.0, (bx, by, FLANGE_T), (0, 0, -1)), 'Screw M10x45', 'Steel')
        o = mp.new_occ(asm.component, 'Nut M10 %d' % (k + 1))
        mp.add_body(o, mp.nut(10.0, (bx, by, -FLANGE_T), (0, 0, -1)), 'Nut M10', 'Steel')
    for k, (dx, dy) in enumerate(dowels):
        o = mp.new_occ(asm.component, 'Dowel 8x24 %d' % (k + 1))
        mp.add_body(o, mp.dowel(8.0, (dx, dy, -12.0), (dx, dy, 12.0)), 'Dowel', 'Steel')
    asm.isGrounded = True
    return asm


# ---------- the shaft assemblies ----------
def input_assembly(root, d, L):
    asm = mp.new_occ(root, 'Input shaft assembly')
    s, B, K = d['shafts']['d'], d['bearings']['in'], d['keys']['in']
    end = -Y_OUT - CAP_T - 48.0
    sections = [(end, -Y_OUT - CAP_T, s['d_in']), (-Y_OUT - CAP_T, -Y_IN, s['d_seat1']),
                (-Y_IN, -15.0, B['dS']), (-15.0, 15.0, 2 * L['rf1']), (15.0, Y_IN, B['dS']),
                (Y_IN, Y_IN + B['B'], s['d_seat1'])]
    sh = mp.stepped_shaft(sections)
    ky0 = end + 10.0
    mp.keyseat(sh, s['d_in'], ky0, ky0 + K['L'], K['b'], K['t1'])
    o = mp.new_occ(asm.component, 'Pinion shaft')
    body = mp.add_body(o, sh, 'Pinion shaft', 'Steel')
    mp.spur_teeth(o, body, d['gears']['m'], d['gears']['z1'], d['gears']['alpha'], BACKLASH, 0.0, -15.0, 15.0)
    o = mp.new_occ(asm.component, 'Key %dx%dx%d input' % (K['b'], K['h'], K['L']))
    mp.add_body(o, mp.parallel_key(s['d_in'], ky0, ky0 + K['L'], K['b'], K['h'], K['t1']), 'Key', 'Steel')
    for tag, y0 in (('A', -Y_IN - B['B']), ('B', Y_IN)):
        inner, _, _, _ = mp.bearing_bodies(s['d_seat1'], B['D'], B['B'], y0)
        nm = 'Bearing %s %s inner ring' % (B['name'], tag)
        o = mp.new_occ(asm.component, nm)
        mp.add_body(o, inner, nm, 'Steel')
    return asm


def output_assembly(root, d, L):
    asm = mp.new_occ(root, 'Output shaft assembly')
    a = L['a']
    s, B, g = d['shafts']['d'], d['bearings']['out'], d['gears']
    Kg, Ko = d['keys']['gear'], d['keys']['out']
    hub = 25.0                                                       # the gear's hub, 50 mm long
    end = Y_OUT + CAP_T + 60.0
    sections = [(-Y_IN - B['B'], -Y_IN, s['d_seat2']), (-Y_IN, -hub, B['dS'] + 5.0), (-hub, Y_IN, s['d_gear']),
                (Y_IN, Y_OUT + CAP_T, s['d_seat2']), (Y_OUT + CAP_T, end, s['d_out'])]
    sh = mp.stepped_shaft(sections, a)
    mp.keyseat(sh, s['d_gear'], -Kg['L'] / 2, Kg['L'] / 2, Kg['b'], Kg['t1'], a)
    ky0 = Y_OUT + CAP_T + 4.0
    mp.keyseat(sh, s['d_out'], ky0, ky0 + Ko['L'], Ko['b'], Ko['t1'], a)
    o = mp.new_occ(asm.component, 'Output shaft', (a, 0, 0))
    mp.add_body(o, sh, 'Output shaft', 'Steel')
    # the gear: hub, web with six holes, rim; the keyway; then the teeth
    gb = mp.tube((a, -hub, 0), (a, hub, 0), 30.0, s['d_gear'] / 2)
    mp.union(gb, mp.tube((a, -5.0, 0), (a, 5.0, 0), L['rf2'] - 11.25, 29.0))
    mp.union(gb, mp.tube((a, -15.0, 0), (a, 15.0, 0), L['rf2'], L['rf2'] - 11.25))
    for k in range(6):
        t = 2 * math.pi * k / 6
        mp.cut(gb, mp.cyl((a + 82.0 * math.cos(t), -6.0, 82.0 * math.sin(t)), (a + 82.0 * math.cos(t), 6.0, 82.0 * math.sin(t)), 20.0))
    mp.cut(gb, mp.box(a - Kg['b'] / 2, a + Kg['b'] / 2, -hub - 1, hub + 1, s['d_gear'] / 2 - 1.0, s['d_gear'] / 2 + Kg['t2']))
    o = mp.new_occ(asm.component, 'Gear z%d' % g['z2'], (a, 0, 0))
    body = mp.add_body(o, gb, 'Gear', 'Steel')
    mp.spur_teeth(o, body, g['m'], g['z2'], g['alpha'], BACKLASH, math.pi - math.pi / g['z2'], -15.0, 15.0)
    o = mp.new_occ(asm.component, 'Key %dx%dx%d gear' % (Kg['b'], Kg['h'], Kg['L']), (a, 0, 0))
    mp.add_body(o, mp.parallel_key(s['d_gear'], -Kg['L'] / 2, Kg['L'] / 2, Kg['b'], Kg['h'], Kg['t1'], a), 'Key', 'Steel')
    o = mp.new_occ(asm.component, 'Key %dx%dx%d output' % (Ko['b'], Ko['h'], Ko['L']), (a, 0, 0))
    mp.add_body(o, mp.parallel_key(s['d_out'], ky0, ky0 + Ko['L'], Ko['b'], Ko['h'], Ko['t1'], a), 'Key', 'Steel')
    o = mp.new_occ(asm.component, 'Spacer', (a, 0, 0))
    mp.add_body(o, mp.tube((a, hub, 0), (a, Y_IN, 0), 22.5, s['d_gear'] / 2), 'Spacer', 'Steel')
    for tag, y0 in (('A', -Y_IN - B['B']), ('B', Y_IN)):
        inner, _, _, _ = mp.bearing_bodies(s['d_seat2'], B['D'], B['B'], y0, a)
        nm = 'Bearing %s %s inner ring' % (B['name'], tag)
        o = mp.new_occ(asm.component, nm, (a, 0, 0))
        mp.add_body(o, inner, nm, 'Steel')
    return asm


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_y(root, moving, fixed, x, name):
    """An As-built revolute joint turning the subassembly `moving` about the world y axis through (x, 0, 0); the
    joint directions are tried (Volume 1, Chapter 17). Returns (joint, sign of its turn about +y)."""
    comp = moving.component
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(sk.modelToSketchSpace(mp.P3(x * CM, 0, 0))).createForAssemblyContext(moving)
    J = adsk.fusion.JointDirections
    for direction in (J.YAxisJointDirection, J.ZAxisJointDirection, J.XAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.05
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][k] * r0[c][k] for k in range(3)) for c in range(3)] for a in range(3)]
        w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        if abs(w[1]) > 0.02 and abs(w[0]) < 0.02 and abs(w[2]) < 0.02:
            j.name = name
            return j, (1 if w[1] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about y' % moving.name)


def parts_list(root):
    """The parts, counted from the components Fusion holds: name without its position tag, and how many."""
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        w = [t for t in o.component.name.split() if t not in ('A', 'B')]
        if w[-1].isdigit() or w[0:2] == ['Cap', 'screw']:
            w = w[:-1] if w[-1].isdigit() else w[:3]
        n = ' '.join(w)
        count[n] = count.get(n, 0) + 1
    return count


def build(P=4.0, n1=1440.0, KA=1.25, life=20000.0, nd=2.0):
    d = rd.design(P=P, n1=n1, KA=KA, life=life, nd=nd)
    short = check_duty(d)
    if short:
        raise ValueError('At %.2f kW and %.0f rpm the chosen parts fall short: %s.' % (P, n1, '; '.join(short)))
    L = layout(d)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    hs = housing(root, d, L)
    ia = input_assembly(root, d, L)
    oa = output_assembly(root, d, L)
    j_in, s_in = revolute_y(root, ia, hs, 0.0, 'Input shaft')
    j_out, s_out = revolute_y(root, oa, hs, L['a'], 'Output shaft')
    g = d['gears']
    k = math.gcd(g['z1'], g['z2'])
    li = root.motionLinks.createInput(j_in, j_out)
    li.valueOne = VS('%d deg' % (360 * g['z2'] // k))             # z2 / k turns of the input ...
    li.valueTwo = VS('%d deg' % (360 * g['z1'] // k))             # ... turn the output z1 / k times, the other way
    li.isReversed = (s_in * s_out) > 0
    root.motionLinks.add(li).name = 'Mesh %d:%d' % (g['z1'], g['z2'])

    for occ, c in ((hs, 'Paint - Enamel Glossy (Grey)'), (ia, 'Paint - Enamel Glossy (Blue)'),
                   (oa, 'Paint - Enamel Glossy (Yellow)')):
        mp.colour(design, occ, c)
    app.activeViewport.fit()
    pl = parts_list(root)
    s, B, K = d['shafts'], d['bearings'], d['keys']
    return ('Spur reducer built: %.1f kW at %.0f rpm, %d:%d = %.2f:1, %.1f N m out at %.0f rpm.\n'
            'Gears m %.0f, face %.0f mm, a = %.1f mm; Lewis %.1f and %.1f MPa, contact %.0f MPa (Kv %.3f); %.0f HB '
            'steel: bending factors %.2f and %.2f, contact factors %.3f and %.3f.\n'
            'Shafts: %.0f / %.0f mm in, %.0f / %.0f / %.0f mm out; DE-Goodman factors %.2f, %.2f, %.2f; slopes %.2e '
            'and %.2e rad; gear deflection %.4f mm.\n'
            'Bearings %s (L10 %.0f h) and %s (L10 %.0f h) for %.0f h.\n'
            'Keys %dx%dx%d, %dx%dx%d, %dx%dx%d: crushing %.1f, %.1f, %.1f MPa.\n\n'
            'Parts: %d components; %s.\n\n[%s]'
            % (P, n1, g['z1'], g['z2'], g['ratio'], g['T2'], g['n2'], g['m'], g['face'], g['a'], g['s1'], g['s2'],
               g['sc'], g['Kv'], g['HB'], g['SF1'], g['SF2'], g['SH1'], g['SH2'], s['d']['d_in'], s['d']['d_seat1'], s['d']['d_out'], s['d']['d_seat2'],
               s['d']['d_gear'], s['n_in'], s['n_gear'], s['n_out'], s['theta1'], s['theta2'], s['y1'],
               B['in']['name'], B['in']['L10h'], B['out']['name'], B['out']['L10h'], life,
               K['in']['b'], K['in']['h'], K['in']['L'], K['gear']['b'], K['gear']['h'], K['gear']['L'],
               K['out']['b'], K['out']['h'], K['out']['L'], K['in']['sigma'], K['gear']['sigma'], K['out']['sigma'],
               sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('P', 'Power (kW)', 4.0), ('n1', 'Input speed (rpm)', 1440.0), ('KA', 'Application factor', 1.25),
          ('life', 'Bearing life L10 (h)', 20000.0), ('nd', 'Design factor', 2.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            ui.messageBox(build(**{k: ins.itemById(k).value for k, _, _ in FIELDS}), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for key, label, dflt in FIELDS:
            ins.addValueInput(key, label, '', VI(dflt))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'SpurReducerCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Spur reducer', 'A single-stage spur reducer')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('SpurReducer could not start:\n' + traceback.format_exc())
