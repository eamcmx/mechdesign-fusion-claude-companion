# =============================================================================
# FatigueSpots.py  -  Chapter 60 of "Mechanical Design in Autodesk Fusion with Claude": the bag gripper with its
# stops, built by design_office/RangeSweep/RangeSweep.py's machine('bag') (loaded, not copied), measured on its faces
# where the fatigue calculations of design_office/lib/fatigue_detail.py look at the gripper's ends: the stop
# brackets' bolts, the brackets' roots, the long stub shaft's keyseat with its key and the short stub shaft's circlip
# groove. The measures go to design_office/results/fatigue_spots.json.
#
#   INPUTS        the dialog: the spot (all, bolts, roots, keyseat or groove; all by default)
#   DERIVED       RangeSweep.machine('bag'); the parts by their names
#   HARD RULES    measured from the parts' faces (the key from its mesh); a spot not one of the four refused
#   CONVENTIONS   the gripper's own new design document; lengths in mm, the API's cm inside; no joints, nothing moved
#   ACCEPTANCE    the message: each spot's measures, the gripper's first line, the version tag
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib.util
import json
import math
import os
import re
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
import companion_paths as CP                            # noqa: E402  every lib/ folder on sys.path


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_fs60_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


VERSION = 'FatigueSpots 1.0 (2026-11-12)'
SPOTS = ('bolts', 'roots', 'keyseat', 'groove')
CHOICES = ('all',) + SPOTS
RESULT = os.path.join(CP.ROOT, 'design_office', 'results', 'fatigue_spots.json')

BOLT = 'Cap screw ISO 4762 M8x40 stop 1'
NUT = 'Nut ISO 4032 M8 stop 1'
BRACKETS = (('closed', 'Stop bracket closed 1'), ('open', 'Stop bracket open 1'))
LONG, KEY, SHORT = 'Stub shaft long A', 'Key 8x7x25 A', 'Stub shaft short A'
BEARING = 'Bearing 6206'
R_ROOT = 10.0                  # mm: the roots' rule lists every cylinder under this radius that is not a bolt hole
MESH_TOL = 0.001               # mm: the key's mesh, its chords at most this far from its round ends
RED, YELLOW = 'Paint - Enamel Glossy (Red)', 'Paint - Enamel Glossy (Yellow)'
_last = {}


# ---------- vectors in mm ----------
def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = math.sqrt(_dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def _mm(p):
    return (p.x * 10.0, p.y * 10.0, p.z * 10.0)


def _dir(v):
    return _unit((v.x, v.y, v.z))


def _r3(p):
    return [round(v, 3) for v in p]


class Axis:
    """A line through o along the unit vector u: t is the distance along it, r the distance from it."""

    def __init__(self, o, u):
        self.o, self.u = o, _unit(u)

    def t(self, p):
        return _dot(_sub(p, self.o), self.u)

    def radial(self, p):
        d = _sub(p, self.o)
        t = _dot(d, self.u)
        return (d[0] - t * self.u[0], d[1] - t * self.u[1], d[2] - t * self.u[2])

    def r(self, p):
        q = self.radial(p)
        return math.sqrt(_dot(q, q))

    def parallel(self, v, tol=1e-4):
        c = _cross(self.u, _unit(v))
        return math.sqrt(_dot(c, c)) < tol

    def same(self, other, tol=0.02):
        return self.parallel(other.u) and self.r(other.o) < tol

    def at(self, t):
        return (self.o[0] + t * self.u[0], self.o[1] + t * self.u[1], self.o[2] + t * self.u[2])


# ---------- the parts, their faces ----------
def _part(root, name):
    """The occurrence (in the root's context) of the component of that name, and its bodies."""
    for o in root.allOccurrences:
        if o.component.name == name and o.component.bRepBodies.count:
            return o, list(o.bRepBodies)
    for b in root.bRepBodies:
        if b.name == name:
            return None, [b]
    for o in root.allOccurrences:
        for b in o.bRepBodies:
            if b.name == name:
                return o, [b]
    raise RuntimeError('No part called %r in the gripper.' % name)


def _series(root, first):
    """Every occurrence named as `first` with its number changed: 'Nut ISO 4032 M8 stop 1' gives stop 1, 2, ..."""
    head = first.rsplit(' ', 1)[0]
    pat = re.compile(r'^%s \d+$' % re.escape(head))
    return [o for o in root.allOccurrences if pat.match(o.component.name) and o.component.bRepBodies.count]


def _faces(bodies):
    return [f for b in bodies for f in b.faces]


def _pts(face):
    """The face's vertices, mm, world."""
    out = []
    for e in face.edges:
        for v in (e.startVertex, e.endVertex):
            if v is not None:
                out.append(_mm(v.geometry))
    if not out:
        out.append(_mm(face.pointOnFace))
    return out


def _body_pts(bodies):
    return [p for f in _faces(bodies) for p in _pts(f)]


def _cyls(bodies):
    out = []
    for f in _faces(bodies):
        g = f.geometry
        if isinstance(g, adsk.core.Cylinder):
            out.append(dict(face=f, axis=Axis(_mm(g.origin), _dir(g.axis)), r=g.radius * 10.0, pts=_pts(f)))
    return out


def _span(axis, pts):
    ts = [axis.t(p) for p in pts]
    return (min(ts), max(ts))


def _overlap(a, b, tol=1e-3):
    return a[0] < b[1] - tol and a[1] > b[0] + tol


def _coaxial(axis, bodies):
    """The body's cylindrical faces on this axis, each with its span along it."""
    cs = [c for c in _cyls(bodies) if axis.same(c['axis'])]
    for c in cs:
        c['span'] = _span(axis, c['pts'])
    return cs


def _main_axis(bodies):
    """A shaft's axis: that of its largest cylindrical face."""
    cs = _cyls(bodies)
    if not cs:
        raise RuntimeError('a shaft with no cylindrical face')
    return max(cs, key=lambda c: c['face'].area)['axis']


def _shank(bodies):
    """A bolt's axis, its shank (the longest cylinder along its own axis) and its faces on that axis."""
    cs = _cyls(bodies)
    if not cs:
        raise RuntimeError('a bolt with no cylindrical face')
    for c in cs:
        s = _span(c['axis'], c['pts'])
        c['len'] = s[1] - s[0]
    A = max(cs, key=lambda c: c['len'])['axis']
    co = _coaxial(A, bodies)
    shank = max(co, key=lambda c: c['span'][1] - c['span'][0])
    return A, shank, co


def _stop_bolts(root):
    """Every stop bolt: its name, occurrence, axis and the span of its shank along the axis."""
    out = []
    for o in _series(root, BOLT):
        A, sh, _ = _shank(list(o.bRepBodies))
        out.append(dict(name=o.component.name, occ=o, axis=A, span=sh['span']))
    return out


def _on_bolt(bodies, bolt):
    """True when a cylindrical face of the bodies is on the bolt's axis and along its shank."""
    return any(_overlap(c['span'], bolt['span']) for c in _coaxial(bolt['axis'], bodies))


# ---------- the key's mesh ----------
def _nodes(body):
    calc = body.meshManager.createMeshCalculator()
    try:
        calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.VeryHighQualityTriangleMesh)
    except Exception:
        pass
    calc.surfaceTolerance = MESH_TOL / 10.0
    c = calc.calculate().nodeCoordinatesAsDouble
    return [(c[i] * 10.0, c[i + 1] * 10.0, c[i + 2] * 10.0) for i in range(0, len(c) - 2, 3)]


def _box_err(pts, bb):
    lo, hi = _mm(bb.minPoint), _mm(bb.maxPoint)
    return max(max(abs(min(p[i] for p in pts) - lo[i]), abs(max(p[i] for p in pts) - hi[i])) for i in range(3))


def _apply(M, pts):
    a = M.asArray()
    return [(a[0] * x + a[1] * y + a[2] * z + a[3] * 10.0, a[4] * x + a[5] * y + a[6] * z + a[7] * 10.0,
             a[8] * x + a[9] * y + a[10] * z + a[11] * 10.0) for x, y, z in pts]


def _chain(occ):
    m = occ.transform2.copy()
    p = occ.assemblyContext
    while p:
        m.transformBy(p.transform2)
        p = p.assemblyContext
    return m


def _mesh_points(occ, body):
    """The body's mesh nodes in world mm: the proxy's mesh if it sits where the body's box is, else the native
    body's mesh carried by whichever of the occurrence's transforms puts it there."""
    bb = body.boundingBox
    cands = []
    try:
        p = _nodes(body)
        cands.append((_box_err(p, bb), p))
    except Exception:
        pass
    if (not cands or cands[0][0] > 0.05) and occ is not None:
        nat = body.nativeObject or body
        base = _nodes(nat)
        cands.append((_box_err(base, bb), base))
        for M in (occ.transform2, _chain(occ)):
            q = _apply(M, base)
            cands.append((_box_err(q, bb), q))
    if not cands:
        raise RuntimeError('no mesh for %s' % body.name)
    err, pts = min(cands, key=lambda c: c[0])
    if err > 0.5:
        raise RuntimeError('the mesh of %s is %.2f mm off its body' % (body.name, err))
    return pts


# ---------- the spots ----------
def bolts(root):
    """Bolt 1 and its nut: the grip along the bolt's axis from the head's underside (the head cylinder's edge nearer
    the shank) to the nut's face nearer the head; the head's height and diameter; the shank's diameter and length."""
    occ, bodies = _part(root, BOLT)
    A, shank, co = _shank(bodies)
    head = max(co, key=lambda c: c['r'])
    if head is shank:
        raise RuntimeError('%s has no head larger than its shank' % BOLT)
    s0, s1 = shank['span']
    mid = (s0 + s1) / 2.0
    ends = []
    for e in head['face'].edges:
        g = e.geometry
        if isinstance(g, (adsk.core.Circle3D, adsk.core.Arc3D)):
            ends.append(A.t(_mm(g.center)))
    if not ends:
        ends = list(head['span'])
    under = min(ends, key=lambda t: abs(t - mid))
    bolt = dict(axis=A, span=shank['span'])
    nocc, nb = _part(root, NUT)
    nut_name = NUT
    if not _on_bolt(nb, bolt):
        on = [o for o in _series(root, NUT) if _on_bolt(list(o.bRepBodies), bolt)]
        if not on:
            raise RuntimeError('No stop nut on the axis of %s.' % BOLT)
        nocc, nb, nut_name = on[0], list(on[0].bRepBodies), on[0].component.name
    planes = []
    for f in _faces(nb):
        g = f.geometry
        if isinstance(g, adsk.core.Plane) and A.parallel(_dir(g.normal)):
            planes.append(A.t(_mm(f.pointOnFace)))
    if not planes:
        raise RuntimeError('%s has no face square to the bolt' % nut_name)
    face = min(planes, key=lambda t: abs(t - under))
    far = s1 if abs(s1 - under) > abs(s0 - under) else s0
    h0, h1 = head['span']
    return dict(bolt=BOLT, nut=nut_name, grip=abs(face - under), head=h1 - h0, head_d=2.0 * head['r'],
                d=2.0 * shank['r'], L=abs(far - under), under=_r3(A.at(under)), nut_face=_r3(A.at(face)),
                axis=_r3(A.u))


def _bracket(root, name, stop_bolts):
    """A bracket's bolt holes (its cylinders on a stop bolt's axis and along its shank), and every other cylinder
    under R_ROOT in radius and every torus."""
    occ, bodies = _part(root, name)
    holes, fillets = set(), []
    for f in _faces(bodies):
        g = f.geometry
        if isinstance(g, adsk.core.Cylinder):
            ax, r = Axis(_mm(g.origin), _dir(g.axis)), g.radius * 10.0
            through = [b['name'] for b in stop_bolts
                       if b['axis'].same(ax) and _overlap(_span(b['axis'], _pts(f)), b['span'])]
            if through:
                holes.update(through)
            elif r < R_ROOT:
                fillets.append(dict(kind='cylinder', r=round(r, 3), at=_r3(_mm(f.pointOnFace))))
        elif isinstance(g, adsk.core.Torus):
            fillets.append(dict(kind='torus', r=round(g.minorRadius * 10.0, 3), R=round(g.majorRadius * 10.0, 3),
                                at=_r3(_mm(f.pointOnFace))))
    return occ, bodies, dict(name=name, holes=len(holes), bolts=sorted(holes), fillets=fillets, sharp=not fillets)


def roots(root, stop_bolts):
    return {k: _bracket(root, name, stop_bolts)[2] for k, name in BRACKETS}


def keyseat(root):
    """The long stub's diameter at the key; the key's length along the stub's axis, its height out from the axis and
    its width, from its mesh; the keyseat's depth, the stub's radius less the key's nearest distance from the axis."""
    socc, sb = _part(root, LONG)
    A = _main_axis(sb)
    co = _coaxial(A, sb)
    kocc, kb = _part(root, KEY)
    P = [p for b in kb for p in _mesh_points(kocc, b)]
    rad = [A.radial(p) for p in P]
    w = _unit((sum(q[0] for q in rad), sum(q[1] for q in rad), sum(q[2] for q in rad)))
    normals = []
    for f in _faces(kb):                                   # out from the axis: square to the key's top and bottom
        g = f.geometry
        if isinstance(g, adsk.core.Plane):
            n = _dir(g.normal)
            if abs(_dot(n, A.u)) < 1e-4:
                normals.append(n)
    if normals:
        n = max(normals, key=lambda n: abs(_dot(n, w)))
        w = n if _dot(n, w) > 0 else (-n[0], -n[1], -n[2])
    v = _unit(_cross(A.u, w))
    ts = [A.t(p) for p in P]
    ws = [_dot(_sub(p, A.o), w) for p in P]
    vs = [_dot(_sub(p, A.o), v) for p in P]
    near = min(A.r(p) for p in P)
    span = (min(ts), max(ts))
    tm = (span[0] + span[1]) / 2.0
    there = [c for c in co if c['span'][0] - 1e-6 <= tm <= c['span'][1] + 1e-6 and c['r'] > near]
    if not there:
        raise RuntimeError('%s has no cylinder round the middle of %s' % (LONG, KEY))
    seat = max(there, key=lambda c: min(c['span'][1], span[1]) - max(c['span'][0], span[0]))
    return dict(stub=LONG, key=KEY, d=2.0 * seat['r'], L=span[1] - span[0], h=max(ws) - min(ws), b=max(vs) - min(vs),
                near=near, t1=seat['r'] - near, nodes=len(P), axis=_r3(A.u))


def _middle(root, A):
    """The gripper's middle along the axis: halfway between its two end plates (or its whole box if it has none)."""
    pat = re.compile(r'^End plate \d+$')
    plates = [o for o in root.allOccurrences if pat.match(o.component.name) and o.component.bRepBodies.count]
    if len(plates) >= 2:
        ts = [A.t(p) for o in plates for p in _body_pts(list(o.bRepBodies))]
        return (min(ts) + max(ts)) / 2.0, 'halfway between the end plates'
    bb = root.boundingBox
    lo, hi = _mm(bb.minPoint), _mm(bb.maxPoint)
    ts = [A.t((x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    return (min(ts) + max(ts)) / 2.0, 'the middle of the whole gripper\'s box'


def _out(span, m):
    a, b = abs(span[0] - m), abs(span[1] - m)
    return (min(a, b), max(a, b))


def groove(root):
    """The short stub's groove (a cylinder on its axis smaller than its neighbours at both ends): its diameter and
    its place, and the place of the Bearing 6206 round the stub, as distances from the gripper's middle."""
    socc, sb = _part(root, SHORT)
    A = _main_axis(sb)
    co = _coaxial(A, sb)
    stub = _span(A, _body_pts(sb))
    found = []
    for c in co:
        a, b = c['span']
        lo = [d for d in co if d is not c and abs(d['span'][1] - a) < 0.01 and d['r'] > c['r'] + 0.01]
        hi = [d for d in co if d is not c and abs(d['span'][0] - b) < 0.01 and d['r'] > c['r'] + 0.01]
        if lo and hi:
            found.append(c)
    if not found:                       # a cylinder smaller than the rest, touching neither of the stub's ends
        inner = [c for c in co if c['span'][0] > stub[0] + 0.01 and c['span'][1] < stub[1] - 0.01]
        if inner:
            found = [min(inner, key=lambda c: c['r'])]
    if not found:
        raise RuntimeError('No groove on %s.' % SHORT)
    m, how = _middle(root, A)
    g = max(found, key=lambda c: abs((c['span'][0] + c['span'][1]) / 2.0 - m))
    rings, names = [], set()
    for o in root.allOccurrences:
        if not o.component.name.startswith(BEARING) or not o.component.bRepBodies.count:
            continue
        cs = _coaxial(A, list(o.bRepBodies))
        if cs:
            s = (min(c['span'][0] for c in cs), max(c['span'][1] for c in cs))
            if _overlap(s, stub):
                rings.extend(cs)
                names.add(o.component.name)
    if not rings:
        raise RuntimeError('No %s round %s.' % (BEARING, SHORT))
    bspan = (min(c['span'][0] for c in rings), max(c['span'][1] for c in rings))
    gout, bout = _out(g['span'], m), _out(bspan, m)
    return dict(stub=SHORT, d=2.0 * g['r'], groove=gout, width=gout[1] - gout[0], bearing=bout,
                bearing_names=sorted(names), bore=2.0 * min(c['r'] for c in rings), od=2.0 * max(c['r'] for c in rings),
                beyond=gout[0] >= bout[1] - 1e-6, middle=_r3(A.at(m)), middle_is=how, axis=_r3(A.u),
                others=[2.0 * c['r'] for c in found if c is not g])


# ---------- colours ----------
def _appearance(des, name):
    ap = des.appearances.itemByName(name)
    if ap:
        return ap
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
    src = None
    try:
        src = lib.appearances.itemByName(name)
    except Exception:
        src = None
    if src is None:                              # the same colour under another paint's name
        word = name[name.find('('):] if '(' in name else name
        for i in range(lib.appearances.count):
            a = lib.appearances.item(i)
            if a.name.startswith('Paint') and word in a.name:
                src = a
                break
    return des.appearances.addByCopy(src, name) if src is not None else None


def _paint(occ, bodies, ap):
    if ap is None:
        return False
    if occ is not None:
        try:
            occ.appearance = ap
            return True
        except Exception:
            pass
    for b in bodies:
        b.appearance = ap
    return True


def paint(des, root, stop_bolts):
    """The two brackets and the key red; the bolts through the brackets and the nuts on them yellow. The stubs are
    left as MaterialsFits coloured their fits."""
    red, yellow = _appearance(des, RED), _appearance(des, YELLOW)
    done = dict(red=[], yellow=[])
    through = set()
    for _, name in BRACKETS:
        occ, bodies, rec = _bracket(root, name, stop_bolts)
        through.update(rec['bolts'])
        if _paint(occ, bodies, red):
            done['red'].append(name)
    occ, bodies = _part(root, KEY)
    if _paint(occ, bodies, red):
        done['red'].append(KEY)
    mine = [b for b in stop_bolts if b['name'] in through] or [b for b in stop_bolts if b['name'] == BOLT]
    for b in mine:
        if _paint(b['occ'], list(b['occ'].bRepBodies), yellow):
            done['yellow'].append(b['name'])
    for o in _series(root, NUT):
        nb = list(o.bRepBodies)
        if any(_on_bolt(nb, b) for b in mine) and _paint(o, nb, yellow):
            done['yellow'].append(o.component.name)
    return done


# ---------- the build ----------
def _fillets(v):
    if v['sharp']:
        return 'no cylinder under %.0f mm in radius other than its bolt holes and no torus: sharp roots' % R_ROOT
    return 'filleted: ' + ', '.join('%s r %.2f at (%.1f, %.1f, %.1f)' % ((f['kind'], f['r']) + tuple(f['at']))
                                    for f in v['fillets'])


def build(spot='all'):
    if spot not in CHOICES:
        raise ValueError('No spot called %s: Chapter 60 measures bolts, roots, keyseat or groove (or all four).' % spot)
    RS = _load('RangeSweep')
    msg, _setter = RS.machine('bag')                       # nothing is moved: the setter is not called
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    first = msg.split('\n')[0]
    stop_bolts = _stop_bolts(root)
    want = SPOTS if spot == 'all' else (spot,)
    res = {}
    if 'bolts' in want:
        res['bolts'] = bolts(root)
    if 'roots' in want:
        res['roots'] = roots(root, stop_bolts)
    if 'keyseat' in want:
        res['keyseat'] = keyseat(root)
    if 'groove' in want:
        res['groove'] = groove(root)
    colours = paint(des, root, stop_bolts)

    out = dict(version=VERSION, spot=spot, units='mm', gripper=first, colours=colours)
    out.update(res)
    os.makedirs(os.path.dirname(RESULT), exist_ok=True)
    with open(RESULT, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=2)
    _last.clear()
    _last.update(out)
    _last['file'] = RESULT

    lines = ['Fatigue spots at the bag gripper\'s ends, measured on the parts as drawn:']
    if 'bolts' in res:
        b = res['bolts']
        lines.append('Bolts: %s with %s: the grip %.2f mm along the axis from the head\'s underside to the nut\'s '
                     'face; the head %.2f high and %.2f across; the shank %.2f across and %.2f long.'
                     % (b['bolt'], b['nut'], b['grip'], b['head'], b['head_d'], b['d'], b['L']))
    if 'roots' in res:
        for k, name in BRACKETS:
            v = res['roots'][k]
            lines.append('Roots: %s, %d bolt holes; %s.' % (name, v['holes'], _fillets(v)))
    if 'keyseat' in res:
        k = res['keyseat']
        lines.append('Keyseat: %s %.2f across at the key; %s %.2f wide, %.2f high and %.2f long (its mesh, %d nodes), '
                     'its nearest %.2f mm from the axis, so the keyseat is %.2f deep.'
                     % (k['stub'], k['d'], k['key'], k['b'], k['h'], k['L'], k['nodes'], k['near'], k['t1']))
    if 'groove' in res:
        g = res['groove']
        lines.append('Groove: %s %.2f across, %.2f to %.2f mm from the gripper\'s middle (%s); %s %.2f to %.2f mm: '
                     'the groove is %s the bearing.'
                     % (g['stub'], g['d'], g['groove'][0], g['groove'][1], g['middle_is'], ', '.join(g['bearing_names']),
                        g['bearing'][0], g['bearing'][1], 'outside' if g['beyond'] else 'NOT outside'))
    lines.append('Red: %s. Yellow: %s. The stubs keep their fits\' colours.'
                 % (', '.join(colours['red']) or 'none', ', '.join(colours['yellow']) or 'none'))
    lines.append('Written to design_office/results/fatigue_spots.json.')
    lines.append(first)
    return '\n'.join(lines) + '\n\n[%s]' % VERSION


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            spot = args.command.commandInputs.itemById('spot').selectedItem.name
            ui.messageBox(build(spot))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('FatigueSpots failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('spot', 'The spot', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in CHOICES:
            dd.listItems.add(k, k == 'all')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcFatigueSpots'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Fatigue spots', 'Chapter 60: the gripper\'s ends measured')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('FatigueSpots failed:\n%s' % traceback.format_exc())
