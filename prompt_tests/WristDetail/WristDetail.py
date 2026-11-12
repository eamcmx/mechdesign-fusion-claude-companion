# =============================================================================
# WristDetail.py  -  Chapter 52 of "Mechanical Design in Autodesk Fusion with Claude": the robotic wrist of Chapter 40
# in detail. WristIntegration.py builds the wrist, loaded, not copied, with the makers' figures in place of Volume 2's
# assumptions (lib/wrist_detail.py, products()). Before the build the functions that draw the bought products, the
# shoulders at the 6002s and the slip ring bracket's screw are replaced by ones drawn to the makers' figures and to
# SKF's bands, each passing every call it does not mean straight through; after the build they are restored.
# MaterialsFits.py's apply() then gives every made part its material and every fitting face its class and roughness,
# and the two housings that SKF's table 4 decides on the load classes are given their classes. Written from PROMPT.txt.
#
#   INPUTS        the dialog: the hard stop's contact angle (128 deg)
#   DERIVED       wrist_detail: products(), numbers(), abutments(), load_classes(), housing_classes(),
#                 detail_numbers(); wrist_design: layout(), stops40(), backlash40(); MaterialsFits.apply()
#   HARD RULES    WristIntegration's build() refuses, run with the products' figures, a contact angle the AM8111's
#                 brakes do not stop the carrier short of, or one that leaves the block under 10 deg a side
#   CONVENTIONS   WristIntegration's: its design document, its groups, WristDifferential's As-built joints and
#                 motion links; nothing moved; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the gear motors, the output shafts, the slip ring, the abutments, the key, the
#                 housings' classes; no interference at rest and over the range; 123 deg refused
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP     # noqa: E402  every lib/ folder on sys.path; CP.script(name) finds a script
import wrist_design as wd        # noqa: E402
import wrist_detail as WDET      # noqa: E402
importlib.reload(WDET)


def _load(name, script):
    spec = importlib.util.spec_from_file_location(name, CP.script(script))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


WI = _load('mdfc_wrist_integration', 'WristIntegration')   # Chapter 40's build(); it loads WristDifferential.py
MF = _load('mdfc_materials_fits', 'MaterialsFits')         # Chapter 51's apply()
WD = WI.WD                                                  # the WristDifferential that WristIntegration builds with

VERSION = 'WristDetail 1.0 (2026-11-11)'
ASSEMBLY = 'wrist'                  # the wrist's key among Chapter 51's nine assemblies

# ---- the detail's own sizes (mm), from the prompt -------------------------------------------------------------------
GH_FLANGE_T = 5.0                   # the PLE040's square flange at the gearhead's face
SPIGOT_H = 2.0                      # its 26 h7 spigot beyond the face
SLOT = 27.0                         # the motor bracket's middle slot, across
KEY_L = 10.0                        # the stop disc's key, DIN 6885 A 5 x 5 at the length that is made
SET_SCREW_D = 4.0                   # the two M4 set screws on the smooth shaft
# ---- Chapters 39 and 40's own figures, kept by the redrawn parts and used to recognise the calls meant --------------
HUB_R = 10.0                        # the small pulley's hub (Chapter 39)
TRAVEL = 3.0                        # the bracket's slots either way along z, for the belt's tension (Chapter 39)
CONNECTOR = (12.0, 12.0, 6.0)       # the motor's connector on its far end, as Chapter 39 drew it
ROTOR_R = 7.5                       # the slip ring's rotor flange on the roll shaft (Chapter 40)
TAB = (13.0, 19.0, 6.0, 17.0, -25.5, -22.5)    # the slip ring bracket's tab under the carrier (Chapter 40)
SCREW_AT = (17.5, 9.5, -25.5)       # the bracket's screw: the head's underside at the tab (Chapter 40)
SCREW_L = 8.0                       # its length in Chapter 40
HOLE_R = 1.7                        # the M3 screw's clearance hole (Chapter 40's tab)
OLD_KEY = 'Key DIN 6885 A 5x5x8'
GM_NAME = 'Servo motor with brake and 15:1 gearhead'
SIDE_GEAR = 'Side bevel gear with 36-groove pulley'
CARRIER = 'Carrier block'


def plan(L):
    """Every size the replacements draw, taken while products() holds the makers' figures."""
    K = wd.LAY40
    G, Mo = WDET.GEARHEAD, WDET.MOTOR
    p = dict(L=L, abut=WDET.abutments())
    by = {(a['what'], a['bearing']): a for a in p['abut']}

    def change(what):
        a = by[(what, '6002-2RSH')]
        return a['d'] / 2.0, a['new'] / 2.0              # radius as drawn, radius the detail gives it
    p['collar'], p['S4'], p['S5'] = change('roll shaft collar'), change('spacer S4'), change('spacer S5')
    p['r_in'] = wd.BEARINGS['6002-2RSH']['d'] / 2.0
    p['length'] = G['body'] + G['adapter'] + Mo['k']    # the gearhead's face to the motor's end
    if abs(L['gh_face'] - p['length'] - L['m_end']) > 1e-6:
        raise RuntimeError('the layout puts the motor\'s end at %.1f, the makers\' figures at %.1f'
                           % (L['m_end'], L['gh_face'] - p['length']))
    if SLOT <= G['spigot']:
        raise RuntimeError('a slot of %.1f does not pass the %.1f spigot' % (SLOT, G['spigot']))
    over, t = wd.SLIPRING['flange']                      # the flange's overhang and thickness, products()'
    p['fl_t'], p['fl_D'] = t, wd.SLIPRING['D'] + 2.0 * over
    p['screw_L'] = SCREW_L + t                           # the screw longer by the flange under the tab
    try:
        p['nums'], p['nums_err'] = WDET.detail_numbers(), None
    except Exception as e:                               # the names still come from the drawing; the numbers not
        p['nums'], p['nums_err'] = {}, str(e)
    p['key_name'] = next((n for n in p['nums'] if n.startswith('Key DIN 6885')), 'Key DIN 6885 A 5x5x%g' % KEY_L)
    if not p['key_name'].endswith('x%g' % KEY_L):
        raise RuntimeError('the detail names the key %s, drawn %g long' % (p['key_name'], KEY_L))
    p['screw_name'] = 'Cap screw ISO 4762 M3x%g' % p['screw_L']
    k0 = K['key'][0]
    p['key'] = (k0, k0 + KEY_L)
    p['disc'] = (K['disc'][0], k0 + KEY_L)
    p['tr_end'] = k0 + KEY_L
    return p


# ---------- the parts redrawn (mm, world coordinates as the functions they replace) ----------
def gearmotor_detail(L, p):
    """The right-hand gear motor: the PLE040's square flange at the gearhead's face with its 26 h7 spigot, its round
    body to the adapter, the adapter square; the AM8111 with its brake, square, to the motor's end; the connector."""
    G, Mo = WDET.GEARHEAD, WDET.MOTOR
    xm, zm = L['x_m'], L['z_m']
    y = L['gh_face']
    F, Fm = G['D'] / 2.0, Mo['flange'] / 2.0
    fl = WD.union(WD.box(xm - F, xm + F, y - GH_FLANGE_T, y, zm - F, zm + F),
                  WD.ycyl(y - 0.1, y + SPIGOT_H, G['spigot'] / 2.0, xm, zm))
    WD.cut(fl, WD.ycyl(y - GH_FLANGE_T - 1, y + SPIGOT_H + 1, G['shaft'] / 2.0 + 0.2, xm, zm))   # the shaft's way out
    for dx in (-16, 16):
        for dz in (-16, 16):
            WD.cut(fl, WD.ycyl(y - GH_FLANGE_T - 1, y + 1, 2.0, xm + dx, zm + dz))             # M4 tapped (the bracket)
    y_ad = y - G['body']
    gh = WD.ycyl(y_ad, y - GH_FLANGE_T, G['D'] / 2.0, xm, zm)
    y_mo = y_ad - G['adapter']
    ad = WD.box(xm - F, xm + F, y_mo, y_ad, zm - F, zm + F)
    y_end = y_mo - Mo['k']
    y_br = y_end + wd.MOTOR39['L_br']
    mo = WD.box(xm - Fm, xm + Fm, y_br, y_mo, zm - Fm, zm + Fm)
    br = WD.box(xm - Fm, xm + Fm, y_end, y_br, zm - Fm, zm + Fm)
    cw, cl, ch = CONNECTOR
    cn = WD.box(xm - cw / 2, xm + cw / 2, y_end + 6.0, y_end + 6.0 + cl, zm + Fm, zm + Fm + ch)
    return [(fl, 'gearhead flange and spigot', 'alu'), (gh, 'planetary gearhead PLE040 15:1', 'dark'),
            (ad, 'motor adapter', 'alu'), (mo, 'servo motor AM8111', 'stator'), (br, 'holding brake', 'dark'),
            (cn, 'connector', 'black')]


def motor_output_detail(L, p):
    """The gear unit's smooth output shaft and the 18-groove pulley on it, held by two M4 set screws in its hub."""
    G = WDET.GEARHEAD
    xm, zm = L['x_m'], L['z_m']
    rs = G['shaft'] / 2.0
    h0, h1 = L['m_hub']
    (fa0, fa1), (fb0, fb1) = L['m_flanges']
    teeth, ro = WD.htd_teeth(fa1 - 0.01, fb0 + 0.01, wd.PULLEYS39[0])
    m = adsk.core.Matrix3D.create()
    m.setToRotation(-math.pi / 2, adsk.core.Vector3D.create(1, 0, 0), adsk.core.Point3D.create(0, 0, 0))   # local z -> +y
    WD.tbm().transform(teeth, m)
    mv = adsk.core.Matrix3D.create()
    mv.translation = adsk.core.Vector3D.create(xm * WD.MM, 0, zm * WD.MM)
    WD.tbm().transform(teeth, mv)
    body = WD.union(WD.ycyl(h0, h1 + 0.01, HUB_R, xm, zm), WD.ycyl(fa0, fa1, ro + 2.5, xm, zm), teeth,
                    WD.ycyl(fb0, fb1, ro + 2.5, xm, zm))
    WD.cut(body, WD.ycyl(h0 - 1, fb1 + 1, rs, xm, zm))
    ys = (h0 + h1) / 2.0
    r = SET_SCREW_D / 2.0
    WD.cut(body, WD.cyl((xm, ys, zm + rs - 1), (xm, ys, zm + HUB_R + 1), r),
           WD.cyl((xm + rs - 1, ys, zm), (xm + HUB_R + 1, ys, zm), r))
    s0, s1 = L['m_shaft']
    shaft = WD.ycyl(s0, s1, rs, xm, zm)
    g1 = WD.cyl((xm, ys, zm + rs), (xm, ys, zm + HUB_R), r)
    g2 = WD.cyl((xm + rs, ys, zm), (xm + HUB_R, ys, zm), r)
    return [(body, 'pulley 18 grooves', 'alu'), (shaft, 'gearhead output shaft %g' % G['shaft'], 'steel'),
            (g1, 'set screw M4 (top)', 'screw'), (g2, 'set screw M4 (side)', 'screw')]


def motor_bracket_detail(L, p, zc):
    """The right-hand bracket, its middle slot wide enough for the 26 mm spigot over the slot's travel."""
    xm, zm = L['x_m'], L['z_m']
    y0, y1 = L['bracket']
    zb = L['base'][1]
    b = WD.union(WD.box(xm - 25, xm + 25, y0, y1, zb, zm + 25), WD.box(xm - 25, xm + 25, y1 - 0.01, 44.0, zb, zb + 6))

    def slot(x, z, r, dz):
        return WD.union(WD.ycyl(y0 - 1, y1 + 1, r, x, z - dz), WD.ycyl(y0 - 1, y1 + 1, r, x, z + dz),
                        WD.box(x - r, x + r, y0 - 1, y1 + 1, z - dz, z + dz))
    WD.cut(b, slot(xm, zm, SLOT / 2.0, TRAVEL))
    for dx in (-16, 16):
        for dz in (-16, 16):
            WD.cut(b, slot(xm + dx, zm + dz, 2.2, TRAVEL))
    for x in (14.0, 40.0):
        WD.cut(b, zc(zb - 1, zb + 7, 2.5, x, 38.0))
    return b


def slip_parts_detail(K, p, zc):
    """The slip ring's rotor flange as Chapter 40 drew it; its stator 22 across with the 44.5 flange at the top,
    the whole 19.5 long, the flange against the tab's underside and bored for the bracket's screw; the tab alone."""
    r0, r1 = K['rotor']
    rotor = WD.cut(zc(r0, r1, ROTOR_R), zc(r0 - 1, r1 + 1, wd.BORE40 / 2.0))
    S = wd.SLIPRING
    top, t = TAB[4], p['fl_t']
    stator = WD.union(zc(top - t, top, p['fl_D'] / 2.0), zc(top - S['L'], top - t + 0.1, S['D'] / 2.0))
    x, y = SCREW_AT[0], SCREW_AT[1]
    WD.cut(stator, zc(top - t - 1, top + 1, HOLE_R, x, y))
    tab = WD.box(*TAB)
    WD.cut(tab, zc(TAB[4] - 0.5, TAB[5] + 0.5, HOLE_R, x, y))
    return rotor, stator, None, tab


# ---------- replacing and restoring ----------
_REPLACED = (('WD', 'gearmotor'), ('WD', 'motor_output'), ('WD', 'motor_bracket'), ('WD', 'ring'), ('WD', 'zcyl'),
             ('WD', 'frame'), ('WI', 'slip_parts'), ('B', 'screw'), ('B', 'part'))


def _owner(w):
    return {'WD': WD, 'WI': WI, 'B': WD.Builder}[w]


def _originals():
    return {(w, n): getattr(_owner(w), n) for w, n in _REPLACED}


def _restore(O):
    for (w, n), f in O.items():
        setattr(_owner(w), n, f)


def _near(a, b):
    return abs(a - b) < 1e-6


def _install(p, O):
    """Each replacement recognises the call it means by its arguments and passes every other to the original."""
    L, K = p['L'], wd.LAY40
    zc, rg, fr = O[('WD', 'zcyl')], O[('WD', 'ring')], O[('WD', 'frame')]
    made = []                                        # the frames made for the bracket's screw

    def the_layout(a, k):
        return (len(a) == 1 and not k and isinstance(a[0], dict)
                and all(n in a[0] and _near(a[0][n], L[n]) for n in ('gh_face', 'x_m', 'z_m')))

    def gearmotor(*a, **k):
        if the_layout(a, k):
            return gearmotor_detail(a[0], p)
        return O[('WD', 'gearmotor')](*a, **k)

    def motor_output(*a, **k):
        if the_layout(a, k):
            return motor_output_detail(a[0], p)
        return O[('WD', 'motor_output')](*a, **k)

    def motor_bracket(*a, **k):
        if the_layout(a, k):
            return motor_bracket_detail(a[0], p, zc)
        return O[('WD', 'motor_bracket')](*a, **k)

    def zcyl(z0, z1, r, x=0.0, y=0.0):
        (c0, c1), (old, new) = L['collar'], p['collar']
        if _near(z0, c0) and _near(z1, c1) and _near(r, old) and _near(x, 0.0) and _near(y, 0.0):
            r = new                                  # the roll shaft's collar
        return zc(z0, z1, r, x, y)

    def ring(z0, z1, ri, ro, x=0.0, y=0.0):
        for (s0, s1), (old, new) in ((L['S4'], p['S4']), (K['S5'], p['S5'])):
            if (_near(z0, s0) and _near(z1, s1) and _near(ri, p['r_in']) and _near(ro, old)
                    and _near(x, 0.0) and _near(y, 0.0)):
                ro = new                             # spacer S4 or S5
                break
        return rg(z0, z1, ri, ro, x, y)

    def frame(X, Y, Z, T=(0, 0, 0)):
        try:
            down = tuple(tuple(v) for v in (X, Y, Z)) == tuple(tuple(v) for v in WD.DOWN)
        except TypeError:
            down = False
        if down and len(T) == 3 and all(_near(T[i], SCREW_AT[i]) for i in range(3)):
            m = fr(X, Y, Z, (T[0], T[1], T[2] - p['fl_t']))     # the head under the slip ring's flange
            made.append(m)
            return m
        return fr(X, Y, Z, T)

    def slip_parts(*a, **k):
        if len(a) == 1 and not k and isinstance(a[0], dict) and 'rotor' in a[0] and 'stator' in a[0]:
            return slip_parts_detail(a[0], p, zc)
        return O[('WI', 'slip_parts')](*a, **k)

    def screw(self, parent, dn, L_, M, kind='shcs'):
        if dn == 3 and _near(L_, SCREW_L) and kind == 'shcs' and any(M is m for m in made):
            return O[('B', 'screw')](self, parent, dn, p['screw_L'], M, kind)
        return O[('B', 'screw')](self, parent, dn, L_, M, kind)

    def part(self, parent, key, name, items, M=None):
        if key == 'stator' and name == 'Slip ring stator':
            items = [it for it in items if it[0] is not None and it[1] != 'pin']     # the stator's pin gone
        elif key == 'key8' and name == OLD_KEY:
            name = p['key_name']                                                     # the key at the length made
        return O[('B', 'part')](self, parent, key, name, items, M)

    WD.gearmotor, WD.motor_output, WD.motor_bracket = gearmotor, motor_output, motor_bracket
    WD.ring, WD.zcyl, WD.frame = ring, zcyl, frame
    WI.slip_parts = slip_parts
    WD.Builder.screw, WD.Builder.part = screw, part
    return made


# ---------- reading the built model ----------
def _name(o):
    return o.component.name.split(' (')[0]


def _cyl_faces(o, r_mm):
    out = []
    for b in o.bRepBodies:
        for f in b.faces:
            g = f.geometry
            if isinstance(g, adsk.core.Cylinder) and abs(g.radius * 10.0 - r_mm) < 1e-3:
                out.append(f)
    return out


def _mid(f, k):
    bb = f.boundingBox
    return (getattr(bb.minPoint, k) + getattr(bb.maxPoint, k)) / 2.0 * 10.0


def housings(root):
    """At rest: each front side gear's bore (the 61902's seat nearer the apex) and the carrier's bore for the roll
    shaft's lower, floating 6002."""
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    r_side = wd.BEARINGS['61902-2RS1']['D'] / 2.0
    r_roll = wd.BEARINGS['6002-2RSH']['D'] / 2.0
    front = [min(fs, key=lambda f: abs(_mid(f, 'y')))
             for fs in (_cyl_faces(o, r_side) for o in leaves if _name(o) == SIDE_GEAR) if fs]
    floating = [min(fs, key=lambda f: _mid(f, 'z'))
                for fs in (_cyl_faces(o, r_roll) for o in leaves if _name(o) == CARRIER) if fs]
    return front, floating


def set_class(f, cls):
    """The face's hole class replaced, its roughness kept; returns the value the rules had given, or None."""
    a = f.attributes.itemByName('mdfc', 'fit')
    if a is None:
        return None
    was = a.value
    parts = []
    for v in was.split(' + '):
        bits = v.split(' ', 1)
        parts.append(cls + ' ' + bits[1] if len(bits) == 2 and bits[0][:1].isupper() else v)
    a.value = ' + '.join(parts)
    return was


def motor_ends(frame_occ, face):
    out = []
    for o in frame_occ.childOccurrences:
        if _name(o) != GM_NAME:
            continue
        ys = []
        for b in o.bRepBodies:
            bb = b.boundingBox
            ys += [bb.minPoint.y * 10.0, bb.maxPoint.y * 10.0]
        lo, hi = min(ys), max(ys)
        end = lo if abs(lo) > abs(hi) else hi
        f = face if end < 0 else -face
        out.append(dict(end=end, face=f, length=abs(end - f), spigot=(hi - lo) - abs(end - f)))
    return out


# ---------- building ----------
def build(contact=128.0):
    contact = float(contact)
    with WDET.products():                         # the makers' figures for the build, its refusal and its numbers
        L = wd.layout()
        p = plan(L)
        O = _originals()
        lay = {k: wd.LAY40[k] for k in ('disc', 'key', 'tr_end')}
        try:
            _install(p, O)
            wd.LAY40.update(disc=p['disc'], key=p['key'], tr_end=p['tr_end'])
            WI.build(contact)
        finally:
            wd.LAY40.update(lay)
            _restore(O)
        stop = wd.stops40()                       # at this contact angle, with the AM8111's brake
        play = wd.backlash40()
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    # the materials and the fits, then the two housings SKF's table 4 decides on the load classes
    try:
        mf_text = MF.apply(ASSEMBLY).split('\n\n[')[0]
    except Exception:
        mf_text = 'not applied: ' + traceback.format_exc().strip().split('\n')[-1]
    hc = WDET.housing_classes()
    front, floating = housings(root)
    was_front = [set_class(f, hc['front']) for f in front]
    was_float = [set_class(f, hc['floating']) for f in floating]
    ends = motor_ends(WD._last['occ']['frame'], L['gh_face'])
    b, a = WDET.numbers()
    dc = WDET.drive_checks(a)
    lc = WDET.load_classes()
    _last.update(L=L, plan=p, front=front, floating=floating, mf=MF._last, stop=stop, play=play, numbers=(b, a),
                 housing=hc, loads=lc, ends=ends, contact=contact)
    app.activeViewport.fit()
    return message(p, contact, ends, stop, play, b, a, dc, lc, hc, was_front, was_float, mf_text,
                   sum(WD.parts_list(root).values()))


def message(p, contact, ends, stop, play, b, a, dc, lc, hc, was_front, was_float, mf_text, n_parts):
    G, Mo, S = WDET.GEARHEAD, WDET.MOTOR, WDET.SLIPRING
    nums = p['nums']
    pn = lambda n: nums[n][0] if n in nums else 'no number: ' + (p['nums_err'] or 'not in detail_numbers()')
    moved = [x for x in p['abut'] if not x['ok']]
    bands = sorted({'%s %.1f to %.1f' % (x['bearing'], x['lo'], x['hi']) for x in moved})
    mo = a['motors']
    lines = [
        'Robotic wrist in detail: Chapter 40\'s wrist with the makers\' figures.',
        'Gear motors (the %s, on the %s): %s mm from the gearhead\'s face to the motor\'s end, the %.0f h7 spigot %s mm beyond, '
        '%.0f mm square; the motors\' ends at y %s (with the assumed products %.1f).'
        % (G['name'], Mo['name'], ', '.join('%.1f' % e['length'] for e in ends), G['spigot'],
           ', '.join('%.1f' % e['spigot'] for e in ends), Mo['flange'], ', '.join('%+.1f' % e['end'] for e in ends),
           b['m_end']),
        'Output: the unit\'s smooth %.0f mm shaft, %.0f mm beyond its face, the pulley on two M4 set screws; the '
        'bracket\'s middle slot %.0f mm across.' % (G['shaft'], G['shaft_L'], SLOT),
        'Slip ring %s: stator %.0f across and %.1f long, its %.1f flange at the top against the bracket\'s tab; %d '
        'circuits of %.0f A, %.0f rev/min against the roll\'s %.0f. The bar and the pin gone; the bracket\'s screw now '
        'a %s (%s).' % (S['name'], S['D'], S['L'], S['flange'], S['circuits'], S['amps'], S['rpm'], dc['roll_rpm'],
                   p['screw_name'], pn(p['screw_name'])),
        'Abutments: %s mm, inside SKF\'s da (%s); the other %d shoulders inside their bands.'
        % (', '.join('%s %.0f to %.0f' % (x['what'], x['d'], x['new']) for x in moved), '; '.join(bands),
           len(p['abut']) - len(moved)),
        'Key: %s (%s) from %.0f to %.0f mm, in the stop disc from %.0f to %.0f; the trunnion\'s end at %.0f.'
        % (p['key_name'], pn(p['key_name']), p['key'][0], p['key'][1], p['disc'][0], p['disc'][1], p['tr_end']),
        'Motors\' duty: %.3f N m peak (%.2f of the AM8111\'s %.2f), %.3f N m RMS (%.2f of its %.2f), %.0f rev/min; '
        'the brakes %.1f times the holding torque needed.'
        % (mo['peak'], dc['peak_ratio'], Mo['peak'], mo['rms'], dc['rms_ratio'], Mo['rated'], mo['nmax'],
           dc['brake_ratio']),
        'Stop: the switch at %.0f deg; the brakes (%.0f ms) stop the carrier by %.1f deg, contact at %.0f; the block '
        '%.1f deg either side.' % (wd.STOP40['switch'], Mo['brake_apply'] * 1000, stop['stop_at'], contact,
                                   stop['block_half']),
        'Play at the tool: %.3f mm (assumed products %.3f). The wrist\'s mass: %.2f kg (%.2f).'
        % (play['at_tool'], b['play']['at_tool'], a['mass']['m'], b['mass']['m']),
        'Bearing loads against SKF\'s C: %s.' % '; '.join('%s %.3f C, %s' % (r['position'], r['ratio'], r['cls'])
                                                          for r in lc),
        'Housings: the front side gears\' bores %s on %d faces (the rules gave %s); the floating roll bearing\'s bore '
        '%s on %d face (the rules gave %s); every other face as Chapter 51\'s rules decide.'
        % (hc['front'], sum(1 for w in was_front if w), ', '.join(sorted({w for w in was_front if w})) or 'none',
           hc['floating'], sum(1 for w in was_float if w), ', '.join(sorted({w for w in was_float if w})) or 'none'),
        'Materials and fits, %s' % mf_text,
        '',
        'Parts: %d components.' % n_parts,
        '',
        '[%s]' % VERSION]
    return '\n'.join(lines)


def pose(pitch, roll):
    WI.pose(pitch, roll)


_last = WI._last
parts_list = WD.parts_list

# ---------- the command dialog ----------
_handlers = []
FIELDS = [('contact', 'Hard stop contact (deg)', 128.0)]


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
            ins.addValueInput(key, label, '', adsk.core.ValueInput.createByReal(dflt))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'WristDetailCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Robotic wrist in detail', 'The wrist of Chapter 52')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('WristDetail could not start:\n' + traceback.format_exc())
