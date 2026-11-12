# =============================================================================
# StudyAssemblies.py  -  Chapters 59 and 62 of "Mechanical Design in Autodesk Fusion with Claude": the real assemblies
# Fusion's static and modal Simulation studies are run on. Each is the part under study and the neighbours that carry
# its load, copied out of the machine's own detailed model (built by RangeSweep.py with the machine's detail script)
# into a design of its own, one component each with its material, where it sits in the machine:
#   stop           jaw A's opening stop bracket, its arm turned 45 deg (StopBracket.py's change on the machine's own
#                  bracket) and its root rounded 2.5 mm, its DYSC-25-25-Y1F, the lock nut and the end plate
#   stop_diagonal  the same with the arm on its diagonal (Chapter 56's arm) and the same root
#   fitting        the spar fitting, both GE 20 ES, both trunnions, the rear spar web cut to 200 mm either side of the
#                  base plate
#   hinge          the right hinge bracket, its rib, the GE 20 ES in its eye, the hinge pin, the flap's two fittings
#   arms           the palletizer's arms at the cycle's high point: 27 of the robot's parts and the study's end plate
#                  in place of the upper arm box's last 10 mm
#   arms_closed    the same 28 parts and the two diaphragms the lower arm's box lacks
# It finds the faces each study picks on the bodies, gives the loads from the design libraries, and closes the
# machine's document unsaved.
# Written from design_office/StudyAssemblies/PROMPT.txt (a prompt test).
#
#   INPUTS        the dialog: the assembly (stop by default)
#   DERIVED       RangeSweep (the machines, the robot's pose), StopBracket (the bracket's change); range_sweep,
#                 flap_design, lugs_brackets, palletizer_design, palletizer_links
#   HARD RULES    every body copied from the machine, none rebuilt, but the study's plates; the bolts left out;
#                 nothing saved; an assembly not one of the six refused
#   CONVENTIONS   a new parametric design, each part a root component with its bodies in a base feature, no joints;
#                 lengths in mm, the API's cm inside
#   ACCEPTANCE    the message: the parts, their bodies, materials and volumes, the picks, the loads, the version
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import difflib
import importlib
import importlib.util
import math
import os
import sys
import time
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # noqa: E402  every lib/ folder on sys.path
import mdfc_parts as mp                                 # noqa: E402
import flap_design as FD                                # noqa: E402
import palletizer_design as PD                          # noqa: E402
import range_sweep as RS                                # noqa: E402
import lugs_brackets as LB                              # noqa: E402
import palletizer_links as PL                           # noqa: E402
for _m in (mp, FD, PD, RS, LB, PL):                     # Fusion keeps loaded modules: take the libraries as they are now
    importlib.reload(_m)

VERSION = 'StudyAssemblies 1.0 (2026-11-12)'
ASSEMBLIES = ('stop', 'stop_diagonal', 'fitting', 'hinge', 'arms', 'arms_closed')
MACHINE = dict(stop='bag', stop_diagonal='bag', fitting='flap', hinge='flap', arms='robot', arms_closed='robot')
WHAT = dict(stop="jaw A's opening stop, the bracket's arm turned 45 deg",
            stop_diagonal="jaw A's opening stop, the bracket's arm on its diagonal",
            fitting="the flap actuator's spar fitting", hinge="the flap's right hinge",
            arms="the palletizer's arms at the cycle's high point",
            arms_closed="the palletizer's arms at the cycle's high point, the lower arm's box closed")
DYSC_TYPE = 'DYSC-25-25-Y1F'
DYSC = [d for d in RS.DYSC.values() if d['type'] == DYSC_TYPE][0]
ROOT_R = 2.5                          # mm, the opening bracket's root
WEB_MARGIN = 200.0                    # mm of the rear spar web kept either side of the fitting's base plate
END_PLATE = (10.0, 340.0, 200.0)      # mm: the study's end plate on the upper arm's box, thick, wide, deep
DIAPHRAGM = dict(t=12.0, inside=(276.0, 196.0), fork=(650.0, 662.0), hub=(122.0, 110.0))   # mm
FLANGE_DROP = 30.0                    # mm, the tool flange's outer face under J4's output
HUB = dict(d=280.0, y=(190.0, 500.0))  # mm, the hub's bosses held on their outsides
PUSH = 1000.0                         # N across the arms' plane at the flange's outer face
ARMS = ('Fork plate 1', 'Fork plate 2', 'Hub boss J2 side', 'Hub boss J3 side', 'Balancer cross pin Q',
        'Lower arm box', 'Elbow hub', 'Elbow pin', 'Upper arm cheek 1', 'Upper arm cheek 2', 'Bearing 32213 elbow 1',
        'Bearing 32213 elbow 2', 'Yoke bulkhead', 'Upper arm box', 'Front fork cheek 1', 'Front fork cheek 2',
        'Bearing 32213 wrist 1', 'Bearing 32213 wrist 2', 'Torque tube', 'Rear lever', 'Pin D', 'Wrist pin',
        'Wrist hub', 'Wrist housing', 'RV-80N case J4', 'RV-80N output J4', 'Tool flange')
PLATE = 'Box end plate (study only)'
DIAPHRAGMS = ('Lower box diaphragm, fork end (study only)', 'Lower box diaphragm, hub end (study only)')
SMALL_HOLE = 6.0                      # mm: a bolt hole's radius is under this (the M8 clearance 4.5, the web's 4.05)
FASTENER = ('screw', 'bolt', 'nut', 'washer', 'rivet', 'cotter', 'split pin', 'circlip')
_last = {}
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create


def _load(name):
    """Another script of the companion, loaded (not copied) from where companion_paths finds it."""
    spec = importlib.util.spec_from_file_location('mdfc_sa59_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ---------- small geometry ----------
def _pt(p):
    return (p.x * 10.0, p.y * 10.0, p.z * 10.0)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _e(i):
    return tuple(1.0 if k == i else 0.0 for k in range(3))


def _par(a, b, tol=1e-4):
    return abs(_dot(a, b)) > 1.0 - tol


def _unit2(p, q):
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    return ((q[0] - p[0]) / d, (q[1] - p[1]) / d), d


def _mm3(body):
    return body.volume * 1000.0


def _extent(tbs, M=None):
    """The bodies' extremes along x, y and z (mm) over their vertices, after the matrix M if given: exact for the
    plane-faced boxes, plates and cheeks it is used on."""
    tbm = mp.tbm()
    lo, hi = [float('inf')] * 3, [-float('inf')] * 3
    for tb in tbs:
        c = tbm.copy(tb)
        if M is not None:
            tbm.transform(c, M)
        for v in c.vertices:
            p = _pt(v.geometry)
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
    return lo, hi


def _arm_frame(P0, e):
    """World to an arm's own frame and back: s along the arm from P0 (x, z in the arms' plane, mm) in the direction
    e, y across the plane as in the world, h square to the arm in the plane."""
    phi = math.atan2(e[1], e[0])
    for sign in (1.0, -1.0):
        T = adsk.core.Matrix3D.create()
        T.translation = V3(-P0[0] * CM, 0.0, -P0[1] * CM)
        R = adsk.core.Matrix3D.create()
        R.setToRotation(sign * phi, V3(0.0, 1.0, 0.0), P3(0.0, 0.0, 0.0))
        M = T.copy()
        M.transformBy(R)
        p = P3((P0[0] + 100.0 * e[0]) * CM, 0.0, (P0[1] + 100.0 * e[1]) * CM)
        p.transformBy(M)
        if abs(p.x * 10.0 - 100.0) < 1e-6 and abs(p.y) < 1e-9 and abs(p.z * 10.0) < 1e-6:
            Mi = M.copy()
            Mi.invert()
            return M, Mi
    raise RuntimeError('no frame along the arm from %s' % (P0,))


def _frame_box(s, y, h, Mi):
    """A box given in an arm's frame (mm), as a temporary body in the world."""
    tb = mp.box(s[0], s[1], y[0], y[1], h[0], h[1])
    mp.tbm().transform(tb, Mi)
    return tb


# ---------- the machine's model ----------
def _part(name, occ, root):
    """One part of the machine: its occurrence and, for a part kept as a subassembly, every occurrence under it that
    holds bodies."""
    occs = [occ] if occ.component.bRepBodies.count else []
    if occ.childOccurrences.count:
        pre = occ.fullPathName + '+'
        occs += [o for o in root.allOccurrences if o.fullPathName.startswith(pre) and o.component.bRepBodies.count]
    return dict(name=name, occs=occs, copies=1)


def _named(root, name):
    occs = list(root.allOccurrences)
    hit = [o for o in occs if o.component.name == name] or [o for o in occs if o.component.name.lower() == name.lower()]
    if not hit:
        near = difflib.get_close_matches(name, sorted({o.component.name for o in occs}), 6, 0.5)
        raise RuntimeError("The machine's model has no part called '%s' (the nearest names: %s)." % (
            name, ', '.join(near) or 'none'))
    p = _part(hit[0].component.name, hit[0], root)
    p['copies'] = len(hit)
    if not p['occs']:
        raise RuntimeError("'%s' holds no bodies in the machine's model." % name)
    return p


def _matching(root, pred):
    """The parts whose (lower-case) names pass pred, each once: not again for an occurrence inside one found."""
    hits = [o for o in root.allOccurrences if pred(o.component.name.lower())]
    paths = [o.fullPathName for o in hits]
    top = [o for o in hits if not any(o.fullPathName.startswith(p + '+') for p in paths)]
    return [p for p in (_part(o.component.name, o, root) for o in top) if p['occs']]


def _world(occ):
    """The occurrence's transform to the world: of transform2 and the product through its parents, the one that
    carries the component's own bodies onto the bodies as the machine holds them."""
    cands = [occ.transform2.copy()]
    try:
        cands.append(mp.world_matrix(occ))
    except Exception:
        pass
    nat, prx = occ.component.bRepBodies, occ.bRepBodies
    pairs = []
    for i in range(min(nat.count, prx.count, 2)):
        fn, fp = nat.item(i).faces, prx.item(i).faces
        for k in sorted({0, fn.count // 2, fn.count - 1}):
            pairs.append((fn.item(k).pointOnFace, fp.item(k).pointOnFace))
    best = None
    for M in cands:
        err = 0.0
        for pn, pp in pairs:
            q = pn.copy()
            q.transformBy(M)
            err = max(err, q.distanceTo(pp))
        if best is None or err < best[0]:
            best = (err, M)
    if best[0] > 1e-4:
        raise RuntimeError('%s: no transform puts its bodies where the machine holds them (%.4f mm off)' % (
            occ.fullPathName, best[0] * 10.0))
    return best[1]


def _copy(part):
    """Every body of the part as a temporary body in world coordinates, with its name, material and volume."""
    tbm = mp.tbm()
    out = []
    for o in part['occs']:
        W = _world(o)
        for b in o.component.bRepBodies:
            tb = tbm.copy(b)
            tbm.transform(tb, W)
            out.append(dict(tb=tb, name=b.name, material=b.material, volume=_mm3(b)))
    return out


def _gap(pa, pb):
    """The least distance between two parts of the machine, mm."""
    mm_ = adsk.core.Application.get().measureManager
    best = float('inf')
    for oa in pa['occs']:
        for ba in oa.bRepBodies:
            for ob in pb['occs']:
                for bb in ob.bRepBodies:
                    best = min(best, mm_.measureMinimumDistance(ba, bb).value * 10.0)
    return best


def _out(part, entries=None):
    entries = _copy(part) if entries is None else entries
    return dict(name=part['name'], entries=entries, machine_volume=sum(e['volume'] for e in entries),
                copies=part.get('copies', 1))


# ---------- each machine: what it gives the study ----------
def _machine_stop(a, root, ctx):
    """The bag gripper with its stops: StopBracket's change on jaw A's opening bracket (the arm turned 45 deg for
    stop, left on its diagonal for stop_diagonal; the root rounded either way), then the bracket, its absorber, the
    lock nut and jaw A's end plate."""
    sb = _load('StopBracket')
    sb.TURNED = (a == 'stop')
    names = ('Stop bracket open 1', 'Shock absorber %s open 1' % DYSC_TYPE, 'Lock nut %s open 1' % DYSC_TYPE)
    sb.change(_named(root, names[0])['occs'][0], 1, ROOT_R)
    adsk.doEvents()
    parts = [_named(root, n) for n in names]
    lip = RS.stop_layout()['closed']['lip']['x']             # the end plate's thickness along x, at jaw A's end
    xm = 0.5 * (lip[0] + lip[1])
    plates = _matching(root, lambda n: n.startswith('end plate'))
    if not plates:
        raise RuntimeError("The bag gripper's model has no part called 'End plate ...'.")

    def off(p):
        bb = p['occs'][0].boundingBox
        return abs(5.0 * (bb.minPoint.x + bb.maxPoint.x) - xm)
    plate = min(plates, key=off)
    parts.append(plate)
    ctx.update(bracket=names[0], absorber=names[1], nut=names[2], plate=plate['name'],
               hand=LB.volume(a == 'stop', ROOT_R), turned=(a == 'stop'))
    ctx['notes'].append("jaw A's end plate: %s, %.1f mm from the plate's middle at x %.1f" % (plate['name'], off(plate), xm))
    return [_out(p) for p in parts]


def _flap_axes(root, ctx):
    """The flap system's axes in the world, from the rear spar web: its longest side along the span (S), its
    thickness the section's x (X), the third the section's z (Z)."""
    web = _named(root, 'Rear spar web')
    entries = _copy(web)
    lo, hi = _extent([e['tb'] for e in entries])
    ext = [hi[i] - lo[i] for i in range(3)]
    S = max(range(3), key=lambda i: ext[i])
    X = min(range(3), key=lambda i: ext[i])
    ctx['axes'] = dict(S=S, X=X, Z=3 - S - X)
    ctx['notes'].append('the rear spar web %.1f x %.1f x %.1f mm: the span along %s, the web\'s thickness along %s' % (
        ext[0], ext[1], ext[2], 'xyz'[S], 'xyz'[X]))
    return web, entries


def _web_patch(entries, S, station):
    """The rear spar web cut to the base plate's half width and WEB_MARGIN either side of the base plate's middle."""
    half = FD.SYS['base_x'] + WEB_MARGIN
    lo, hi = _extent([e['tb'] for e in entries])
    rng = [[lo[i] - 5.0, hi[i] + 5.0] for i in range(3)]
    rng[S] = [station - half, station + half]
    keep = []
    for e in entries:
        l, h = _extent([e['tb']])
        if h[S] <= rng[S][0] or l[S] >= rng[S][1]:
            continue
        mp.intersect(e['tb'], mp.box(rng[0][0], rng[0][1], rng[1][0], rng[1][1], rng[2][0], rng[2][1]))
        if e['tb'].volume > 1e-9:
            keep.append(e)
    return keep, half


def _machine_flap(a, root, ctx):
    web, web_entries = _flap_axes(root, ctx)
    S = ctx['axes']['S']
    ges = _matching(root, lambda n: 'ge20' in n.replace(' ', ''))
    if a == 'fitting':
        fit = _named(root, 'Spar fitting')
        ges = sorted(ges, key=lambda p: _gap(p, fit))[:2]
        if len(ges) < 2 or _gap(ges[1], fit) > 0.5:
            raise RuntimeError('Two GE 20 ES in the spar fitting expected; found %s' % ', '.join(p['name'] for p in ges))
        tr = _matching(root, lambda n: n.startswith('trunnion'))
        tr = sorted(tr, key=lambda p: min(_gap(p, g) for g in ges))[:2]
        if len(tr) < 2:
            raise RuntimeError('Two trunnions expected; found %s' % ', '.join(p['name'] for p in tr))
        fe = _copy(fit)
        lo, hi = _extent([e['tb'] for e in fe])
        station = 0.5 * (lo[S] + hi[S])                        # the base plate's middle along the span
        out = [_out(fit, fe)] + [_out(p) for p in ges] + [_out(p) for p in tr]
        mv = sum(e['volume'] for e in web_entries)
        keep, half = _web_patch(web_entries, S, station)
        w = _out(web, keep)
        w['machine_volume'] = mv
        out.append(w)
        ctx.update(fitting=fit['name'], web=web['name'], ges=[p['name'] for p in ges], trunnions=[p['name'] for p in tr],
                   station=station, half=half, hand=LB.web_patch(WEB_MARGIN))
        ctx['notes'].append('the spar fitting %.1f mm along the span (its base plate %.0f by flap_design), its middle at '
                            '%.1f; the web kept %.1f either side of it' % (hi[S] - lo[S], 2 * FD.SYS['base_x'], station, half))
        return out
    br = _named(root, 'Hinge bracket R')
    rib = _named(root, 'Hinge rib R')
    pin = _named(root, 'Hinge pin R')
    ges = sorted(ges, key=lambda p: _gap(p, br))[:1]
    if not ges or _gap(ges[0], br) > 0.5:
        raise RuntimeError('No GE 20 ES found in the right hinge bracket\'s eye.')
    taken = {br['name'], rib['name'], pin['name'], ges[0]['name']}
    cands = [p for p in _matching(root, lambda n: 'fitting' in n and 'spar' not in n) if p['name'] not in taken]
    scored = sorted(((_gap(p, pin), p) for p in cands), key=lambda t: t[0])
    fits = [p for d, p in scored if d < 0.5][:2]
    how = 'by name'
    if len(fits) < 2:                                          # by contact with the pin
        how = 'by contact with the pin'
        bb = pin['occs'][0].boundingBox
        taken |= {p['name'] for p in fits}
        near = []
        for o in root.allOccurrences:
            n = o.component.name
            if not o.component.bRepBodies.count or n in taken or 'ge20' in n.lower().replace(' ', '') \
                    or any(w in n.lower() for w in FASTENER):
                continue
            ob = o.boundingBox
            if (ob.minPoint.x > bb.maxPoint.x + 0.5 or ob.maxPoint.x < bb.minPoint.x - 0.5 or ob.minPoint.y > bb.maxPoint.y + 0.5
                    or ob.maxPoint.y < bb.minPoint.y - 0.5 or ob.minPoint.z > bb.maxPoint.z + 0.5 or ob.maxPoint.z < bb.minPoint.z - 0.5):
                continue
            p = _part(n, o, root)
            d = _gap(p, pin)
            if d < 0.05:
                near.append((d, p))
        fits += [p for d, p in sorted(near, key=lambda t: t[0])][:2 - len(fits)]
    if len(fits) < 2:
        raise RuntimeError("The flap's two hinge fittings on the right hinge's pin not found (%d)." % len(fits))
    out = [_out(br), _out(rib), _out(ges[0]), _out(pin)] + [_out(p) for p in fits]
    ctx.update(bracket=br['name'], rib=rib['name'], pin=pin['name'], ge=ges[0]['name'], fittings=[p['name'] for p in fits])
    ctx['notes'].append("the flap's hinge fittings found %s: %s" % (how, ', '.join(p['name'] for p in fits)))
    return out


def _machine_arms(a, root, ctx, setter):
    """The robot at the cycle's high point (posed by RangeSweep's setter), its 27 parts, the upper arm's box cut 10 mm
    short with the study's end plate in its place, and for arms_closed the lower arm's two diaphragms."""
    hi = PD.waypoints(PD.brief())['hi']
    setter(('hi', (hi['q2'], hi['q3'])))
    adsk.doEvents()
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    try:
        if des.snapshots.hasPendingSnapshot:
            des.snapshots.add()
    except Exception:
        pass
    out = [_out(_named(root, n)) for n in ARMS]
    by = {p['name']: p for p in out}
    P = PD.pts(hi['q2'], hi['q3'])
    A, B, W = P['A'], P['B'], P['W']
    tbm = mp.tbm()
    # the upper arm's box: its last 10 mm cut off, the end plate in their place
    e3, L3 = _unit2(B, W)
    M, Mi = _arm_frame(B, e3)
    box = by['Upper arm box']['entries']
    lo, hi_ = _extent([e['tb'] for e in box], M)
    clo, chi = _extent([e['tb'] for n in ('Front fork cheek 1', 'Front fork cheek 2') for e in by[n]['entries']], M)
    t, w, d = END_PLATE
    s_end = hi_[0]
    slab = _frame_box((s_end - t, s_end + 5.0), (lo[1] - 5.0, hi_[1] + 5.0), (lo[2] - 5.0, hi_[2] + 5.0), Mi)
    for e in box:
        if _extent([e['tb']], M)[1][0] > s_end - t + 1e-6:
            mp.cut(e['tb'], tbm.copy(slab))
    yc, hc = 0.5 * (clo[1] + chi[1]), 0.5 * (lo[2] + hi_[2])
    plate = _frame_box((s_end - t, s_end), (yc - w / 2.0, yc + w / 2.0), (hc - d / 2.0, hc + d / 2.0), Mi)
    steel_up = box[0]['material']
    out.append(dict(name=PLATE, entries=[dict(tb=plate, name='End plate', material=steel_up, volume=None)],
                    machine_volume=None, copies=0))
    ctx['notes'].append("the upper arm's box ends %.1f mm from the elbow (the wrist pin %.1f, so %.1f short), %.1f deep "
                        "and %.1f wide; the front fork's cheeks from %.1f, their outer faces %.1f apart" % (
                            s_end, L3, L3 - s_end, hi_[2] - lo[2], hi_[1] - lo[1], clo[0], chi[1] - clo[1]))
    if a == 'arms_closed':
        e2, L2 = _unit2(A, B)
        M2, Mi2 = _arm_frame(A, e2)
        lb = by['Lower arm box']['entries']
        lo2, hi2 = _extent([e['tb'] for e in lb], M2)
        wide, narrow = max(DIAPHRAGM['inside']), min(DIAPHRAGM['inside'])
        wy, wh = (wide, narrow) if hi2[1] - lo2[1] >= hi2[2] - lo2[2] else (narrow, wide)
        yc2, hc2 = 0.5 * (lo2[1] + hi2[1]), 0.5 * (lo2[2] + hi2[2])
        spans = (DIAPHRAGM['fork'], (L2 - DIAPHRAGM['hub'][0], L2 - DIAPHRAGM['hub'][1]))
        for name, s in zip(DIAPHRAGMS, spans):
            tb = _frame_box(s, (yc2 - wy / 2.0, yc2 + wy / 2.0), (hc2 - wh / 2.0, hc2 + wh / 2.0), Mi2)
            out.append(dict(name=name, entries=[dict(tb=tb, name='Diaphragm', material=lb[0]['material'], volume=None)],
                            machine_volume=None, copies=0))
        ctx['notes'].append("the lower arm's box from %.1f to %.1f mm from the shoulder, %.1f wide and %.1f deep outside; "
                            "the diaphragms %.0f x %.0f from %.0f to %.0f and %.0f to %.0f" % (
                                lo2[0], hi2[0], hi2[1] - lo2[1], hi2[2] - lo2[2], wy, wh, spans[0][0], spans[0][1],
                                spans[1][0], spans[1][1]))
    ctx.update(q2=hi['q2'], q3=hi['q3'], fl=P['fl'])
    return out


# ---------- the study's design ----------
def _give(body, src):
    """The machine's material: the library's material of that name, else the machine's own."""
    if src is None:
        return
    app = adsk.core.Application.get()
    libs = [app.materialLibraries.item(i) for i in range(app.materialLibraries.count)]
    libs.sort(key=lambda lib: 0 if lib.name == 'Fusion Material Library' else 1)
    for lib in libs:
        try:
            m = lib.materials.itemByName(src.name)
        except Exception:
            m = None
        if m:
            try:
                body.material = m
                return
            except Exception:
                pass
    try:
        body.material = src
    except Exception:
        pass


def _study(parts):
    """A new parametric design, each part a component of the root at the identity, its bodies in a base feature."""
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    made = []
    for p in parts:
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = p['name']
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        for e in p['entries']:
            comp.bRepBodies.add(e['tb'], bf)
        bf.finishEdit()
        bodies = [bf.bodies.item(i) for i in range(bf.bodies.count)]
        for b, e in zip(bodies, p['entries']):
            b.name = e['name']
            _give(b, e['material'])
        made.append((p, occ, bodies))
        adsk.doEvents()
    return doc, des, made


# ---------- the faces each study picks ----------
def _inside(body, p):
    return body.pointContainment(p) == adsk.fusion.PointContainment.PointInsidePointContainment


def _cyls(body):
    """The body's cylindrical faces: radius, axis, a point on the axis (mm), and whether each is a hole."""
    out = []
    for f in body.faces:
        g = f.geometry
        if g.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
            continue
        a = g.axis.copy()
        a.normalize()
        o = g.origin
        p = f.pointOnFace
        v = V3(p.x - o.x, p.y - o.y, p.z - o.z)
        t = v.dotProduct(a)
        rad = V3(v.x - a.x * t, v.y - a.y * t, v.z - a.z * t)
        if rad.length < 1e-9:
            continue
        rad.normalize()
        q = P3(p.x + rad.x * 0.01, p.y + rad.y * 0.01, p.z + rad.z * 0.01)
        out.append(dict(face=f, r=g.radius * 10.0, axis=(a.x, a.y, a.z), origin=_pt(o), hole=_inside(body, q),
                        area=f.area * 100.0))
    return out


def _planes(body):
    """The body's plane faces: the outward normal and the centroid (mm)."""
    out = []
    for f in body.faces:
        g = f.geometry
        if g.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
            continue
        n = g.normal.copy()
        n.normalize()
        p = f.pointOnFace
        if _inside(body, P3(p.x + n.x * 0.01, p.y + n.y * 0.01, p.z + n.z * 0.01)):
            n = V3(-n.x, -n.y, -n.z)
        out.append(dict(face=f, n=(n.x, n.y, n.z), c=_pt(f.centroid), area=f.area * 100.0))
    return out


def _holes(cyls):
    """Cylindrical hole faces gathered into holes: one entry per radius and axis line, with its faces."""
    out = {}
    for h in cyls:
        k = max(range(3), key=lambda i: abs(h['axis'][i]))
        key = (round(h['r'], 2), k) + tuple(round(h['origin'][j], 1) for j in range(3) if j != k)
        out.setdefault(key, []).append(h)
    return out


def _picks_stop(B, ctx):
    lay = RS.stop_layout()
    o, c = lay['open'], lay['closed']
    X = (1.0, 0.0, 0.0)
    near = lambda h, pts: any(math.hypot(h['origin'][1] - y, h['origin'][2] - z) < 0.5 for y, z in pts)
    m8 = lambda h: h['hole'] and _par(h['axis'], X) and abs(h['r'] - 4.5) < 0.01
    out = []
    br = [h for b in B[ctx['bracket']] for h in _cyls(b) if m8(h)]
    out.append(("the bracket's M8 holes", [h['face'] for h in br]))
    plate = [h for b in B[ctx['plate']] for h in _cyls(b) if h['hole']]
    out.append(("the end plate's M8 holes under the bracket", [h['face'] for h in plate if m8(h) and near(h, o['bolts'])]))
    # fixed: the end plate on its frame holes (lugs_brackets.REAL): of its holes not the stop brackets', the set of four
    rest = _holes([h for h in plate if not (m8(h) and near(h, o['bolts'] + c['bolts']))])
    groups = {}
    for key, faces in rest.items():
        groups.setdefault(key[:2], []).append(faces)
    four = [g for g in groups.values() if len(g) == 4]
    if four:
        holes, rule = sorted(four, key=lambda g: g[0][0]['r'])[0], 'its four frame holes'
    elif groups:
        holes, rule = max(groups.values(), key=len), 'its largest set of like holes (no set of four)'
    else:
        holes, rule = [], None
    faces = [h['face'] for g in holes for h in g]
    if not faces:                                               # no frame holes: the plate's inner face
        x0 = RS.stop_layout()['closed']['lip']['x'][0]
        faces = [p['face'] for b in B[ctx['plate']] for p in _planes(b) if p['n'][0] < -0.9999 and abs(p['c'][0] - x0) < 0.01]
        rule = 'its inner face (no frame holes)'
    ctx['notes'].append("the end plate held on %s" % rule)
    out.append(("the end plate's frame holes (fixed)", faces))
    n = (0.0, o['axis'][0], o['axis'][1])
    front = [p['face'] for b in B[ctx['absorber']] for p in _planes(b)
             if _dot(p['n'], n) < -0.9999 and math.dist(p['c'], (o['x'], o['face'][0], o['face'][1])) < 0.5]
    out.append(("the DYSC's front face (the push)", front))
    return out


def _picks_fitting(B, ctx):
    ax = ctx['axes']
    S, eX, eS = ax['S'], _e(ax['X']), _e(ax['S'])
    bolt = lambda h: h['hole'] and _par(h['axis'], eX) and h['r'] < SMALL_HOLE
    out = [("the spar fitting's base-plate bolt holes", [h['face'] for b in B[ctx['fitting']] for h in _cyls(b) if bolt(h)]),
           ("the web's bolt holes", [h['face'] for b in B[ctx['web']] for h in _cyls(b) if bolt(h)]),
           ("the web's cut ends (fixed)", [p['face'] for b in B[ctx['web']] for p in _planes(b)
                                           if _par(p['n'], eS) and abs(abs(p['c'][S] - ctx['station']) - ctx['half']) < 0.01])]
    for t in ctx['trunnions']:
        ends = [p for b in B[t] for p in _planes(b) if _par(p['n'], eS)]
        best = min(ends, key=lambda p: abs(p['c'][S] - ctx['station'])) if ends else None
        out.append(("%s's inner end (the push, both ways)" % t, [best['face']] if best else []))
    return out


def _picks_hinge(B, ctx):
    eZ = _e(ctx['axes']['Z'])
    pin = sorted([h for b in B[ctx['pin']] for h in _cyls(b) if not h['hole']], key=lambda h: -h['area'])
    out = [("the hinge bracket's bolt holes", [h['face'] for b in B[ctx['bracket']] for h in _cyls(b)
                                                if h['hole'] and h['r'] < SMALL_HOLE]),
           ("the rib's bolt holes", [h['face'] for b in B[ctx['rib']] for h in _cyls(b) if h['hole'] and h['r'] < SMALL_HOLE]),
           ("the rib's top and bottom (fixed)", [p['face'] for b in B[ctx['rib']] for p in _planes(b) if abs(_dot(p['n'], eZ)) > 0.95])]
    if pin:
        o, a = pin[0]['origin'], pin[0]['axis']

        def off_axis(c):
            v = [c[i] - o[i] for i in range(3)]
            t = _dot(v, a)
            return math.sqrt(max(0.0, _dot(v, v) - t * t))
    else:
        off_axis = None
    for f in ctx['fittings']:
        faces = [p for b in B[f] for p in _planes(b)]
        best = max(faces, key=lambda p: off_axis(p['c'])) if faces and off_axis else None
        out.append(("%s's face on the flap (the load)" % f, [best['face']] if best else []))
    return out


def _picks_arms(B, ctx):
    Y = (0.0, 1.0, 0.0)
    out = []
    for name in ('Hub boss J2 side', 'Hub boss J3 side'):
        faces = [h for b in B[name] for h in _cyls(b) if not h['hole'] and _par(h['axis'], Y) and abs(h['r'] - HUB['d'] / 2.0) < 0.01]
        ys = []
        for h in faces:
            bb = h['face'].boundingBox
            ys += [abs(bb.minPoint.y * 10.0), abs(bb.maxPoint.y * 10.0)]
        if ys:
            ctx['notes'].append('%s held on its outside %.0f across, |y| from %.1f to %.1f' % (name, HUB['d'], min(ys), max(ys)))
        out.append(('%s: its outside (held)' % name, [h['face'] for h in faces]))
    z = ctx['fl'][1] - FLANGE_DROP
    face = [p['face'] for b in B['Tool flange'] for p in _planes(b) if p['n'][2] < -0.9999 and abs(p['c'][2] - z) < 0.01]
    out.append(("the tool flange's outer face (the push)", face))
    return out


PICKS = dict(stop=_picks_stop, stop_diagonal=_picks_stop, fitting=_picks_fitting, hinge=_picks_hinge, arms=_picks_arms,
             arms_closed=_picks_arms)


# ---------- the loads ----------
def _loads(a, ctx):
    if a in ('stop', 'stop_diagonal'):
        o = RS.stop_layout()['open']
        F = RS.jaw_stops()['open']['F']
        n = (0.0, o['axis'][0], o['axis'][1])
        return dict(F=F, design=tuple(F * v for v in n), axis=n, F_max=float(DYSC['F_stop']),
                    rating=tuple(DYSC['F_stop'] * v for v in n), on="the DYSC's front face, along its axis")
    if a == 'fitting':
        act = FD.actuator_design()['act']
        row = max(act['rows'], key=lambda r: r['F'])
        return dict(F=act['F_ult'], ways=(1.0, -1.0), delta=row['delta'], u_section=FD.linkage(row['delta'])['u'],
                    on="the trunnions' inner ends, along the actuator's axis, pushing and pulling")
    if a == 'hinge':
        sd = FD.system_design()
        R, Rm = sd['ult']['R'], sd['ult']['R_mag']
        return dict(F=sd['R_hinge'], direction_section=(-R[0] / Rm, -R[1] / Rm), on="the flap's two hinge fittings")
    fl = ctx['fl']
    at = (fl[0], fl[1] - FLANGE_DROP)
    h = PL.lateral(at=(at[0] / 1000.0, at[1] / 1000.0))
    own = PL.lateral()
    return dict(F=PUSH, lateral=(0.0, PUSH, 0.0), at=(at[0], 0.0, at[1]), c_hand=h['c_flange'],
                give_hand=PUSH * h['c_flange'] * 1000.0, c_flange=own['c_flange'], give_flange=PUSH * own['c_flange'] * 1000.0,
                on="the tool flange's outer face, across the arms' plane")


# ---------- the build ----------
def _box(bodies):
    lo, hi = [float('inf')] * 3, [-float('inf')] * 3
    for b in bodies:
        bb = b.boundingBox
        l, h = _pt(bb.minPoint), _pt(bb.maxPoint)
        lo = [min(lo[i], l[i]) for i in range(3)]
        hi = [max(hi[i], h[i]) for i in range(3)]
    return tuple(lo), tuple(hi)


def build(assembly='stop'):
    a = str(assembly).strip()
    if a not in ASSEMBLIES:
        raise ValueError('No assembly called %s: StudyAssemblies builds %s.' % (a, ', '.join(ASSEMBLIES)))
    app = adsk.core.Application.get()
    t0 = time.time()
    rsm = _load('RangeSweep')
    machine_msg, setter = rsm.machine(MACHINE[a])
    mdoc = app.activeDocument
    try:
        mdoc.attributes.add('mdfc', 'run', 'StudyAssemblies')
    except Exception:
        pass
    sdoc = None
    ctx = dict(notes=[])
    try:
        root = adsk.fusion.Design.cast(app.activeProduct).rootComponent
        if a in ('stop', 'stop_diagonal'):
            parts = _machine_stop(a, root, ctx)
        elif a in ('fitting', 'hinge'):
            parts = _machine_flap(a, root, ctx)
        else:
            parts = _machine_arms(a, root, ctx, setter)
        adsk.doEvents()
        sdoc, sdes, made = _study(parts)
        B = {p['name']: bodies for p, occ, bodies in made}
        picks = PICKS[a](B, ctx)
        for name, faces in picks:
            for f in faces:
                try:
                    f.attributes.add('mdfc_study', 'pick', name)
                except Exception:
                    pass
    finally:
        try:
            mdoc.close(False)
        except Exception:
            pass
        if sdoc is not None:
            try:
                sdoc.activate()
            except Exception:
                pass
    rows = []
    for p, occ, bodies in made:
        mats = sorted({b.material.name for b in bodies if b.material})
        rows.append(dict(part=p['name'], bodies=[b.name for b in bodies], material=' + '.join(mats) or None,
                         volume=sum(_mm3(b) for b in bodies), machine_volume=p['machine_volume'], box=_box(bodies),
                         copies=p.get('copies', 1)))
    found = {name: bool(faces) for name, faces in picks}
    loads = _loads(a, ctx)
    out = dict(assembly=a, machine=MACHINE[a], parts=rows, found=found, picks={n: len(f) for n, f in picks}, loads=loads,
               notes=ctx['notes'], seconds=time.time() - t0, version=VERSION)
    for k in ('hand', 'turned', 'station', 'half', 'q2', 'q3', 'fl'):
        if k in ctx:
            out[k] = ctx[k]
    _last.clear()
    _last.update(out)
    # the message
    lines = ['Study assembly %s (%s): %d parts copied from the machine\'s model into a new design, one component '
             'each, no joints; the machine\'s document closed unsaved.' % (a, WHAT[a], len(rows))]
    for r in rows:
        mv = r['machine_volume']
        lines.append('  %s: %d bod%s (%s), %s, %.1f mm3 (%s)' % (
            r['part'], len(r['bodies']), 'y' if len(r['bodies']) == 1 else 'ies', ', '.join(r['bodies']), r['material'],
            r['volume'], 'the study\'s own' if mv is None else 'the machine\'s %.1f' % mv))
    if a in ('stop', 'stop_diagonal'):
        b = [r for r in rows if r['part'] == ctx['bracket']][0]
        lines.append('  The bracket by hand (lugs_brackets, the arm %s, root r %g): %.1f mm3, %+.4f %%.' % (
            'turned' if ctx['turned'] else 'on its diagonal', ROOT_R, ctx['hand'], 100.0 * (b['volume'] / ctx['hand'] - 1.0)))
    if a == 'fitting':
        w = [r for r in rows if r['part'] == ctx['web']][0]
        lines.append('  The web patch by hand (lugs_brackets.web_patch): %.1f mm3, %+.4f %%.' % (
            ctx['hand'], 100.0 * (w['volume'] / ctx['hand'] - 1.0)))
    if a in ('arms', 'arms_closed'):
        tf = [r for r in rows if r['part'] == 'Tool flange'][0]
        (x0, y0, z0), (x1, y1, z1) = tf['box']
        lines.append('  At q2 %.3f, q3 %.3f deg the tool flange spans x %.2f to %.2f, y %.2f to %.2f, z %.2f to %.2f; '
                     'the high point\'s flange at (%.2f, %.2f).' % (ctx['q2'], ctx['q3'], x0, x1, y0, y1, z0, z1,
                                                                   ctx['fl'][0], ctx['fl'][1]))
    lines.append('Picks: ' + '; '.join('%s %d' % (n, len(f)) for n, f in picks) +
                 ('; all found.' if all(found.values()) else '; NOT FOUND: %s.' % ', '.join(n for n, v in found.items() if not v)))
    if a in ('stop', 'stop_diagonal'):
        lines.append("Loads: the design's push %.1f N along the absorber's axis (0, %.4f, %.4f) and the DYSC's rating "
                     "%.0f N, on %s." % (loads['F'], loads['axis'][1], loads['axis'][2], loads['F_max'], loads['on']))
    elif a == 'fitting':
        lines.append("Loads: the actuator's ultimate push %.1f N both ways (at %.0f deg of flap), on %s." % (
            loads['F'], loads['delta'], loads['on']))
    elif a == 'hinge':
        lines.append("Loads: the hinge's ultimate load %.1f N, on %s." % (loads['F'], loads['on']))
    else:
        lines.append("Loads: %.0f N across the arms' plane at (%.1f, 0, %.1f), %s; by hand (palletizer_links.lateral) it "
                     "gives %.4f mm there, %.4f mm at the flange's own point." % (
                         loads['F'], loads['at'][0], loads['at'][2], loads['on'], loads['give_hand'], loads['give_flange']))
    if ctx['notes']:
        lines.append('Found in the model: ' + '; '.join(ctx['notes']) + '.')
    lines.append('Built in %.0f s.' % out['seconds'])
    return '\n'.join(lines) + '\n\n[%s]' % VERSION


# ---------- the command dialog ----------
_handlers = []
EVENT_ID = 'mdfcStudyAssembliesBuild'


class _Build(adsk.core.CustomEventHandler):
    """The build, fired from the dialog's execute: Fusion refuses a new document inside a command's events."""
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.additionalInfo or 'stop'))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('StudyAssemblies failed:\n%s' % traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            key = args.command.commandInputs.itemById('assembly').selectedItem.name
            if not app.fireCustomEvent(EVENT_ID, key):
                ui.messageBox(build(key))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('StudyAssemblies failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('assembly', 'Assembly', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in ASSEMBLIES:
            dd.listItems.add(k, k == 'stop')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        try:
            app.unregisterCustomEvent(EVENT_ID)
        except Exception:
            pass
        ev = app.registerCustomEvent(EVENT_ID)
        hb = _Build()
        ev.add(hb)
        _handlers.append(hb)
        cid = 'mdfcStudyAssemblies'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Study assemblies',
                                                         'Chapters 59 and 62: the assemblies the Simulation studies are run on')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StudyAssemblies failed:\n%s' % traceback.format_exc())


def stop(_context):
    try:
        adsk.core.Application.get().unregisterCustomEvent(EVENT_ID)
    except Exception:
        pass
