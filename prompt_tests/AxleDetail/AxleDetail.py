# =============================================================================
# AxleDetail.py  -  Chapter 54 of "Mechanical Design in Autodesk Fusion with Claude": the rear axle in detail. The axle
# is built by RearAxle.py's own build(), with the functions that draw the parts replaced for the length of the build:
# the cups' bores closed to the cups, the nose's middle bore stepped inside SKF's abutment band of each pinion cup,
# the spacer from cone face to cone face, the companion flange's hub and the adjusters relieved at the bearings, the
# half shafts' and the side gears' splines to ISO 14. After the build the case's ends are relieved where the cones
# sit, the carrier's and the caps' bores are split where the adjusters begin, MaterialsFits.py's apply() gives the
# materials and the fits, and the threads and the splines get their classes by hand.
#
#   INPUTS        the dialog: the adjusters' thread pitch (mm), 2
#   DERIVED       lib/axle_detail.py: SKF_ABUT, ABUTMENTS, abutments(), carrier_layout(), spacer(), load_classes(),
#                 splines(), SHAFT, PINION, FITS14, adjuster(), ADJ, RELIEF; MaterialsFits.apply()
#   HARD RULES    a pitch at which one thread would not carry the case bearing's axial load at the peak with a factor
#                 of 2 in shear on the adjusters' 1045 is refused
#   CONVENTIONS   RearAxle.py's: the axle along Y, forward +X, up +Z, six subassemblies, As-built joints and motion
#                 links; nothing moved; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the bores, the spacer, the reliefs, the splines, the classes; no interference at rest,
#                 driving straight, nor in a turn; the refusal of a 1 mm pitch
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
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_' + name, CP.script(name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


RA = _load('RearAxle')                  # loads RearAxleCarrier.py and AxleHousing.py itself
MF = _load('MaterialsFits')
import axle_design as ad        # noqa: E402  (reloaded by RearAxle.py)
import mdfc_parts as mp         # noqa: E402  (reloaded by RearAxle.py)
import axle_detail as XD        # noqa: E402
importlib.reload(XD)
CAR, HSG, M = RA.CAR, RA.HSG, MF.M

VERSION = 'AxleDetail 1.0 (2026-11-11)'
CM = 0.1
VI = adsk.core.ValueInput.createByReal
TOL = 1e-5                              # cm, for matching faces


# ---------- the sizes, all from lib/axle_detail.py and lib/axle_design.py ----------
def _abut(brg, kind):
    """The row of axle_detail.ABUTMENTS for one bearing and one kind of abutment."""
    return [r for r in XD.ABUTMENTS if r[1] == brg and r[2] == kind][0]


CUP_D = {b: float(ad.SKF_TRB[b][1]) for b in ('32309', '32308', '32213')}
DIA = dict(cup_A=CUP_D['32309'], cup_B=CUP_D['32308'], cup_C=CUP_D['32213'],
           middle_A=_abut('32309', 'Da')[4], middle_B=_abut('32308', 'Da')[4],
           hub_relief=_abut('32308', 'da')[4], case_shoulder=_abut('32213', 'da')[4],
           adjuster_relief=_abut('32213', 'Da')[4], spacer=_abut('32309', 'db')[4])
# Volume 2's diameters, by which the replacements recognise the calls they mean
V2 = dict(bore_A=_abut('32309', 'Db')[3], bore_B=_abut('32308', 'Db')[3], middle=_abut('32309', 'Da')[3],
          hub=_abut('32308', 'da')[3], spacer=_abut('32309', 'db')[3], adjuster_bore=_abut('32213', 'Da')[3])
V2_OVER = V2['bore_A'] - DIA['cup_A']   # Volume 2 drew every cup's bore this much over the cup
RELIEF = dict(XD.RELIEF)                # mm deep: the flange's hub, the adjusters, the case's ends
CASE_RING_OD = 82.0                     # the ring cut at each end of the case, from the shoulder's 75 out to 82
Y_CUP_C = XD.HUB_Y + XD.C_T             # the case cups' back faces, where the adjusters begin (y = -/+ 106.75)
SY_1045 = 310.0                         # MPa, 1045 hot rolled (Shigley Table A-20): the adjusters are 120 across,
#                                         and materials_fits.DEFAULT takes 1045 as hot rolled over 76 mm
N_THREAD = 2.0                          # the factor one thread must keep in shear
PITCH = 2.0                             # mm, the dialog's default
SPLINE_JOBS = [('Half shaft', False, XD.SHAFT, 'sliding'), ('Side gear', True, XD.SHAFT, 'hub'),
               ('Pinion shaft', False, XD.PINION, 'fixed'), ('Companion flange', True, XD.PINION, 'hub')]
EXPECTED = {'cup A bore': 1, 'cup B bore': 1, 'cup C bores': 4, 'middle bore': 1, 'spacer': 1, 'flange hub': 1,
            'adjusters': 2}


# ---------- the replacements, each passing every call it does not mean ----------
_ORIG = {}
_HITS = {}
_NOW = {}                               # the layout and the spacer while the build runs


def _same(a, b, tol=1e-6):
    return abs(a - b) < tol


def _on_axis(p0, p1, i):
    """Whether the segment p0-p1 lies on the world axis i (0 x, 1 y, 2 z)."""
    return (all(abs(p[j]) < 1e-6 for p in (p0, p1) for j in range(3) if j != i)
            and abs(p1[i] - p0[i]) > 1e-6)


def _hit(what):
    _HITS[what] = _HITS.get(what, 0) + 1


def _dir(p0, p1):
    d = [p1[i] - p0[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in d))
    return [v / n for v in d], n


def _middle_bore(p0, p1):
    """The nose's middle bore, middle_A across from cup A's back face to the middle, middle_B on to cup B's."""
    cyl, L = _ORIG['cyl'], _NOW['L']
    a_end, b_end = (p0, p1) if abs(p0[0] - L['cupA'][1]) <= abs(p1[0] - L['cupA'][1]) else (p1, p0)
    sgn = 1.0 if b_end[0] > a_end[0] else -1.0
    xs = L['step']
    rA, rB = DIA['middle_A'] / 2.0, DIA['middle_B'] / 2.0
    if rA >= rB:
        tb = cyl((xs - sgn * 1.0, 0.0, 0.0), b_end, rB)
        mp.union(tb, cyl(a_end, (xs, 0.0, 0.0), rA))
    else:
        tb = cyl(a_end, (xs + sgn * 1.0, 0.0, 0.0), rA)
        mp.union(tb, cyl((xs, 0.0, 0.0), b_end, rB))
    return tb


def _cyl(p0, p1, r):
    cyl = _ORIG['cyl']
    if _on_axis(p0, p1, 0):
        if _same(r, (DIA['cup_A'] + V2_OVER) / 2.0):
            _hit('cup A bore')
            return cyl(p0, p1, DIA['cup_A'] / 2.0)
        if _same(r, (DIA['cup_B'] + V2_OVER) / 2.0):
            _hit('cup B bore')
            return cyl(p0, p1, DIA['cup_B'] / 2.0)
        if _same(r, V2['middle'] / 2.0):
            _hit('middle bore')
            return _middle_bore(p0, p1)
    if _on_axis(p0, p1, 1) and _same(r, (DIA['cup_C'] + V2_OVER) / 2.0):
        _hit('cup C bores')
        return cyl(p0, p1, DIA['cup_C'] / 2.0)
    return cyl(p0, p1, r)


def _hub(p0, p1, r_out, r_in):
    """The companion flange's hub, hub_relief across for RELIEF['flange'] mm at cone B (p0), r_out beyond."""
    cyl = _ORIG['cyl']
    e, n = _dir(p0, p1)
    at = lambda t: [p0[i] + e[i] * t for i in range(3)]
    d = RELIEF['flange']
    tb = cyl(at(d), p1, r_out)
    mp.union(tb, cyl(p0, at(d + 0.5), DIA['hub_relief'] / 2.0))
    return mp.cut(tb, cyl(at(-1.0), at(n + 1.0), r_in))


def _adjuster(p0, p1, r_out, r_in):
    """An adjuster, adjuster_relief across for RELIEF['adjuster'] mm at the cup (p0), its own bore beyond."""
    cyl = _ORIG['cyl']
    e, n = _dir(p0, p1)
    at = lambda t: [p0[i] + e[i] * t for i in range(3)]
    tb = cyl(p0, p1, r_out)
    mp.cut(tb, cyl(at(-1.0), at(RELIEF['adjuster']), DIA['adjuster_relief'] / 2.0))
    return mp.cut(tb, cyl(at(-1.0), at(n + 1.0), r_in))


def _tube(p0, p1, r_out, r_in):
    tube, L, sp = _ORIG['tube'], _NOW['L'], _NOW['sp']
    if _on_axis(p0, p1, 0):
        if _same(r_out, V2['spacer'] / 2.0) and _same(p0[0], sp['v2'][0], 1e-4):
            _hit('spacer')
            return tube((sp['new'][0], 0.0, 0.0), (sp['new'][1], 0.0, 0.0), r_out, r_in)
        if _same(r_out, V2['hub'] / 2.0) and _same(p0[0], L['xB'], 1e-4):
            _hit('flange hub')
            return _hub(p0, p1, r_out, r_in)
    if (_on_axis(p0, p1, 1) and _same(r_out, DIA['cup_C'] / 2.0) and _same(r_in, V2['adjuster_bore'] / 2.0)
            and _same(abs(p0[1]), Y_CUP_C, 1e-4)):
        _hit('adjusters')
        return _adjuster(p0, p1, r_out, r_in)
    return tube(p0, p1, r_out, r_in)


def _engagement_defaults():
    """axle_design.spline_engagement's defaults with ISO 14's minor and major diameters and number of splines."""
    f = ad.spline_engagement
    co = f.__code__
    names = co.co_varnames[:co.co_argcount]
    dflt = list(f.__defaults__)
    new = dict(d_minor=XD.SHAFT[0], d_major=XD.SHAFT[1], n=XD.SHAFT[2])
    return tuple(new.get(nm, v) for nm, v in zip(names[len(names) - len(dflt):], dflt))


def _patch(A):
    _NOW.update(L=A['L'], sp=A['sp'])
    _ORIG.update(cyl=mp.cyl, tube=mp.tube)
    old = dict(car=CAR.SHAFT_SPLINE, hsg=HSG.SPLINE, eng=ad.spline_engagement.__defaults__)
    mp.cyl, mp.tube = _cyl, _tube
    CAR.SHAFT_SPLINE = tuple(XD.SHAFT)
    HSG.SPLINE = tuple(XD.SHAFT)
    ad.spline_engagement.__defaults__ = _engagement_defaults()
    return old


def _restore(old):
    mp.cyl, mp.tube = _ORIG['cyl'], _ORIG['tube']
    CAR.SHAFT_SPLINE, HSG.SPLINE = old['car'], old['hsg']
    ad.spline_engagement.__defaults__ = old['eng']


# ---------- the design ----------
def design(pitch=2.0):
    if not pitch > 0:
        raise ValueError('A pitch of %g mm is no thread.' % pitch)
    keep = XD.ADJ['p']
    try:
        XD.ADJ['p'] = float(pitch)
        a = XD.adjuster()
    finally:
        XD.ADJ['p'] = keep
    Ssy = 0.577 * SY_1045
    a['n_shear'] = Ssy / a['tau_one']
    return dict(pitch=float(pitch), thread='M%g x %g' % (XD.ADJ['d'], pitch), adj=a, Ssy=Ssy,
                abut=XD.abutments(), sp=XD.spacer(), spl=XD.splines(), loads=XD.load_classes(),
                L=XD.carrier_layout())


def refuse(A):
    a = A['adj']
    if a['n_shear'] < N_THREAD:
        raise ValueError('The adjusters\' thread %s falls short: one thread would carry the case bearing\'s axial load '
                         'at the peak, %.0f N, at %.1f MPa in shear, a factor of %.2f against the 1045\'s %.1f MPa '
                         '(0.577 x %.0f, hot rolled); %.0f is needed.'
                         % (A['thread'], a['Fa'], a['tau_one'], a['n_shear'], A['Ssy'], SY_1045, N_THREAD))


# ---------- after the build ----------
def _child(comp, name):
    return [o for o in comp.occurrences if o.component.name == name][0]


def _unit(v):
    n = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    return (v.x / n, v.y / n, v.z / n)


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _sub(p, q):
    return (p.x - q.x, p.y - q.y, p.z - q.z)


def relieve_case(root):
    """A ring RELIEF['case'] deep, from case_shoulder to CASE_RING_OD across, cut at each end of the differential case
    where the cones' back faces sit."""
    case = _child(_child(root, 'Differential assembly').component, 'Differential case')
    comp = case.component
    body = comp.bRepBodies.item(0)
    tools = adsk.core.ObjectCollection.create()
    for s in (-1, 1):
        ring = mp.tube((0.0, s * (XD.HUB_Y + 1.0), 0.0), (0.0, s * (XD.HUB_Y - RELIEF['case']), 0.0),
                       CASE_RING_OD / 2.0, DIA['case_shoulder'] / 2.0)
        tools.add(mp.add_body(case, ring, 'Relief ring'))
    ci = comp.features.combineFeatures.createInput(body, tools)
    ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    ci.isKeepToolBodies = False
    comp.features.combineFeatures.add(ci)


def _plane_y(comp, y):
    """A construction plane square to y at y (mm, the component's frame), its side checked."""
    for sgn in (1.0, -1.0):
        ci = comp.constructionPlanes.createInput()
        ci.setByOffset(comp.xZConstructionPlane, VI(sgn * y * CM))
        pl = comp.constructionPlanes.add(ci)
        if abs(pl.geometry.origin.y - y * CM) < 1e-6:
            return pl
        pl.deleteMe()
    raise RuntimeError('no plane at y = %.2f mm' % y)


def _is_y_bore(f, r):
    g = f.geometry
    return (isinstance(g, adsk.core.Cylinder) and abs(g.radius - r * CM) < TOL
            and abs(abs(_unit(g.axis)[1]) - 1.0) < 1e-6 and abs(g.origin.x) < TOL and abs(g.origin.z) < TOL)


def split_bores(root):
    """The bores of the case's cups in the casting and the caps split at the cups' back faces, where the adjusters
    begin: the seat inside, the thread beyond."""
    car = _child(root, 'Carrier')
    n = 0
    for name in ('Carrier casting', 'Bearing cap C1', 'Bearing cap C2'):
        comp = _child(car.component, name).component
        for s in (-1, 1):
            y = s * Y_CUP_C * CM
            faces = adsk.core.ObjectCollection.create()
            for b in comp.bRepBodies:
                for f in b.faces:
                    if _is_y_bore(f, DIA['cup_C'] / 2.0):
                        bb = f.boundingBox
                        if bb.minPoint.y < y - TOL and bb.maxPoint.y > y + TOL:
                            faces.add(f)
            if faces.count:
                pl = _plane_y(comp, s * Y_CUP_C)
                sf = comp.features.splitFaceFeatures
                sf.add(sf.createInput(faces, pl, True))
                pl.isLightBulbOn = False
                n += faces.count
    return n


def assembly_key():
    """MaterialsFits' name for the assembly RearAxle.py builds."""
    is_ra = lambda v: os.path.splitext(os.path.basename(str(v)))[0] == 'RearAxle'
    for m in MF.D.MACHINES:
        if any(is_ra(v) for v in m):
            return m[0]
    scripts = getattr(MF.D, 'SCRIPTS', {})
    if isinstance(scripts, dict):
        for k, v in scripts.items():
            if any(is_ra(x) for x in (v if isinstance(v, (list, tuple)) else (v,))):
                return k
    for m in MF.D.MACHINES:
        if any('rear axle' in str(v).lower() for v in m):
            return m[0]
    raise RuntimeError('MaterialsFits names no assembly built by RearAxle.py')


def spline_faces(occ, d_minor, d_major, w, hub):
    """The faces of straight-sided splines as mdfc_parts draws them: the shaft's minor cylinder 0.01 over and its
    teeth flat-topped at the major diameter, the hub's hole 0.1 larger all round. Returns the minor diameter's faces
    (d), the major's (D) and the flanks (B)."""
    gm, g = (0.1, 0.1) if hub else (0.01, 0.0)
    r_min, R, half = (d_minor / 2.0 + gm) * CM, (d_major / 2.0 + g) * CM, (w / 2.0 + g) * CM
    faces = [f for b in occ.bRepBodies for f in b.faces]
    minor = [f for f in faces if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius - r_min) < TOL]
    if not minor:
        raise RuntimeError('no splines of %g minor diameter on %s' % (d_minor, occ.component.name))
    c = minor[0].geometry
    e, o = _unit(c.axis), c.origin
    ts = [_dot(_sub(v.geometry, o), e) for f in minor for v in f.vertices]
    t0, t1 = min(ts) - TOL, max(ts) + TOL
    r_out = math.hypot(R, half) + 0.005
    major, flanks = [], []
    for f in faces:
        gg = f.geometry
        if not isinstance(gg, adsk.core.Plane):
            continue
        nrm = _unit(gg.normal)
        if abs(_dot(nrm, e)) > 1e-6:
            continue
        dq = _sub(f.pointOnFace, o)
        t = _dot(dq, e)
        if not t0 <= t <= t1 or math.sqrt(max(_dot(dq, dq) - t * t, 0.0)) > r_out:
            continue
        dist = abs(_dot(_sub(gg.origin, o), nrm))
        if abs(dist - R) < TOL:
            major.append(f)
        elif abs(dist - half) < TOL:
            flanks.append(f)
    return dict(d=minor, D=major, B=flanks)


def spline_ra(role, cls, size):
    """Ra of a spline's face: the major diameter fits nothing (ISO 14 centres on the minor), so OTHER_RA; a fitting
    face by SKF's table of materials_fits, which stops at IT7, a coarser grade taking its IT7 value."""
    if role == 'D':
        return M.OTHER_RA
    return M._seat_ra(min(int(cls[1:]), max(M.SEAT_RA[0][1])), size)


def _give(f, value, done):
    tok = f.entityToken
    if tok in done['seen']:
        return 0
    done['seen'].add(tok)
    a = f.attributes.itemByName('mdfc', 'fit')
    if a is None:
        f.attributes.add('mdfc', 'fit', value)
        done['new'].append(f)
    else:
        a.value = value
    done['changed'].append(f)
    return 1


def give_by_hand(root, A):
    """The threads' classes on the adjusters and on the bores beyond the cups; ISO 14's fits on the splines."""
    leaves = {o.component.name: o for o in root.allOccurrences if o.component.bRepBodies.count}
    done = dict(seen=set(), changed=[], new=[])
    ext, inn = XD.ADJ['cls'][1], XD.ADJ['cls'][0]
    r = DIA['cup_C'] / 2.0
    n = {'6g': 0, '6H': 0}
    for name, o in sorted(leaves.items()):
        if name.startswith('Adjuster'):
            for f in [f for b in o.bRepBodies for f in b.faces]:
                g = f.geometry
                if isinstance(g, adsk.core.Cylinder) and abs(g.radius - r * CM) < TOL:
                    n['6g'] += _give(f, '%s %s' % (A['thread'], ext), done)
    for name in ('Carrier casting', 'Bearing cap C1', 'Bearing cap C2'):
        for b in leaves[name].bRepBodies:
            for f in b.faces:
                if _is_y_bore(f, r):
                    bb = f.boundingBox
                    if bb.minPoint.y >= Y_CUP_C * CM - TOL or bb.maxPoint.y <= -Y_CUP_C * CM + TOL:
                        n['6H'] += _give(f, '%s %s' % (A['thread'], inn), done)
    spl = {}
    for prefix, hub, dims, fit in SPLINE_JOBS:
        cls = XD.FITS14[fit]
        size = dict(d=dims[0], D=dims[1], B=dims[3])
        for name, o in sorted(leaves.items()):
            if not name.startswith(prefix) or 'washer' in name.lower():
                continue
            fs = spline_faces(o, dims[0], dims[1], dims[3], hub)
            k = 0
            for role in ('d', 'D', 'B'):
                val = MF.finish(cls[role], dict(ra=spline_ra(role, cls[role], size[role])))
                for f in fs[role]:
                    k += _give(f, val, done)
            spl[name] = k
    return done, n, spl


# ---------- the build ----------
_last = {}


def _band(r):
    lo, hi = r['band']
    return ('%g to %g' % (lo, hi) if lo is not None and hi is not None else
            ('at most %g' % hi if lo is None else 'at least %g' % lo))


def build(pitch=PITCH):
    A = design(pitch)
    refuse(A)
    _last.clear()
    _HITS.clear()
    old = _patch(A)
    try:
        ra_msg = RA.build()
    finally:
        _restore(old)
    if _HITS != EXPECTED:
        raise RuntimeError('the replacements met %s of the calls they mean, not %s' % (_HITS, EXPECTED))
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    relieve_case(root)
    n_split = split_bores(root)
    mf_msg = MF.apply(assembly_key())
    done, n_thr, n_spl = give_by_hand(root, A)
    _last.update(mf=dict(MF._last), changed=done['changed'], new=done['new'], design=A, hits=dict(_HITS),
                 threads=n_thr, splines=n_spl, split=n_split)
    return message(A, ra_msg, mf_msg, len(done['changed']), len(done['new']), n_thr, n_spl, n_split)


def message(A, ra_msg, mf_msg, n_changed, n_new, n_thr, n_spl, n_split):
    a, sp, s, L = A['adj'], A['sp'], A['spl'], A['L']
    e = s['eng']
    abut = '\n'.join('  %s (%s, SKF %s): %g, Volume 2 %g' % (r['what'], r['bearing'], _band(r), r['new'], r['v2'])
                     for r in A['abut'])
    loads = '\n'.join('  %s %s: %.3f C cruising (%s), %.3f C at the peaks (%s); seat %s'
                      % (r['name'], r['bearing'], r['cruise'], r['cls_cruise'], r['peak'], r['cls_peak'], r['seat'])
                      for r in A['loads'])
    hs = sum(v for k, v in n_spl.items() if k.startswith('Half shaft'))
    sg = sum(v for k, v in n_spl.items() if k.startswith('Side gear'))
    pn = sum(v for k, v in n_spl.items() if k.startswith('Pinion shaft'))
    fl = sum(v for k, v in n_spl.items() if k.startswith('Companion flange'))
    return ('Rear axle in detail: %s\n\n'
            'Abutments, the detail\'s diameters against SKF\'s:\n%s\n'
            'Spacer from cone A\'s front face to cone B\'s, %.3f to %.3f mm: %.1f long (Volume 2: %.1f between the '
            'cups, %.2f and %.2f mm short of the cones).\n'
            'Cups\' bores %g, %g and %g, the cups\' own diameters, in the casting and the caps (Volume 2: %g over). '
            'Nose\'s middle bore %g from cup A to %.1f mm, %g on to cup B (Volume 2: %g).\n'
            'Companion flange\'s hub %g for %g mm at cone B; the case\'s ends cut to %g where the cones sit (a ring %g '
            'deep to %g); the adjusters %g for %g mm at the cups, %g beyond.\n'
            'Load classes:\n%s\n'
            'Splines: half shafts and side gears %d x %g x %g x %g (ISO 14 light series; Volume 2 %d x %g x %g): '
            '%.1f MPa in torsion at the minor diameter, factor %.2f (Volume 2 %.2f); engaged %.2f mm (%.1f needed), '
            '%.1f MPa on the flanks, factor %.2f. Pinion %d x %g x %g x %g: %s ISO 14\'s medium series.\n'
            'Adjusters\' thread %s %s/%s: minor diameter %.3f, pitch diameter %.3f mm, %.0f threads engaged. The case '
            'bearing\'s axial load at the peak, %.0f N, on one thread: %.1f MPa in shear, factor %.2f against the '
            '1045\'s %.1f MPa (%.0f needed); %.1f MPa bearing, %.1f MPa bending; on all %.0f threads %.1f MPa in '
            'shear.\n'
            'Faces given by hand: %s %s on the %d adjusters\' outsides, %s on %d faces of the casting\'s and the '
            'caps\' bores beyond the cups (%d bores split); ISO 14\'s sliding fit on the half shafts (%d faces) and '
            'the side gears (%d), its fixed fit on the pinion (%d) in the companion flange (%d); %d faces in all, %d '
            'of them new.\nBy the rules: %s\n\n[%s]'
            % (ra_msg.split('\n')[0], abut, sp['new'][0], sp['new'][1], sp['L'], sp['L_v2'], sp['gap_A'], sp['gap_B'],
               DIA['cup_A'], DIA['cup_B'], DIA['cup_C'], V2_OVER, DIA['middle_A'], L['step'], DIA['middle_B'],
               V2['middle'], DIA['hub_relief'], RELIEF['flange'], DIA['case_shoulder'], RELIEF['case'], CASE_RING_OD,
               DIA['adjuster_relief'], RELIEF['adjuster'], V2['adjuster_bore'], loads,
               XD.SHAFT[2], XD.SHAFT[0], XD.SHAFT[1], XD.SHAFT[3], XD.SHAFT_V2[2], XD.SHAFT_V2[0], XD.SHAFT_V2[1],
               s['tau'], s['n'], s['n_v2'], e['l'], e['l_need'], e['p'], e['n'],
               XD.PINION[2], XD.PINION[0], XD.PINION[1], XD.PINION[3], 'in' if s['pinion_iso'] else 'not in',
               A['thread'], XD.ADJ['cls'][0], XD.ADJ['cls'][1], a['dr'], a['dp'], a['nt'], a['Fa'], a['tau_one'],
               a['n_shear'], A['Ssy'], N_THREAD, a['sigB_one'], a['sigb_one'], a['nt'], a['tau_all'],
               A['thread'], XD.ADJ['cls'][1], n_thr['6g'], XD.ADJ['cls'][0], n_thr['6H'], n_split, hs, sg, pn, fl,
               n_changed, n_new, mf_msg.split('\n\n')[0], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            ui.messageBox(build(pitch=round(ins.itemById('pitch').value / CM, 9)), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('pitch', 'The adjusters\' thread pitch', 'mm', VI(PITCH * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'AxleDetailCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Rear axle in detail', 'Chapter 54: the rear axle in detail')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('AxleDetail could not start:\n' + traceback.format_exc())
