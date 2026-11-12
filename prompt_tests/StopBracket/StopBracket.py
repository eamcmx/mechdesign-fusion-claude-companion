# =============================================================================
# StopBracket.py  -  Chapter 62 of "Mechanical Design in Autodesk Fusion with Claude": the bag gripper's two opening
# stop brackets changed in the machine's own model. RangeSweep.py builds the bag gripper with its stops; here the arm
# of each opening bracket, as RangeSweep builds it (its sides along the end plate's edges, the absorber's push along
# its diagonal), is cut off and put back turned 45 deg about its own axis (its sides along the absorber's axis and
# the bolts' line), the lug made whole and bored again for the DYSC-25's M30 x 1.5, all in the base feature that
# holds the bracket; the root's four edges are filleted. Then each bracket is measured (the arm's sides against the
# absorber's axis, the root's faces, the gaps to its two M8 heads, its volume and mass against the hand volume) and
# the whole gripper is swept through its range with RangeSweep's own sweep.
#
#   INPUTS        the dialog: the root radius of the arm (mm), 2.5
#   DERIVED       lugs_brackets: TURN, arm(), clearance(), volume(); range_sweep: stop_layout(), DYSC, STATES['bag'];
#                 RangeSweep.py: machine('bag') and sweep(), loaded and called
#   HARD RULES    the bracket changed in its base feature (BaseFeature.updateBody); jaw B's bracket is jaw A's turned
#                 half a turn about z (the machine's own turn); a fillet feature on the bracket's component; the
#                 material kept; nothing saved; a radius below 0 or at or above the toe's limit refused
#   CONVENTIONS   RangeSweep's new design document; temporary bodies in world mm turned into the component's frame;
#                 no joint added; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the arm square to the push; four root faces; the volume; the heads' gaps; the sweep
# =============================================================================
import importlib
import importlib.util
import json
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
import companion_paths as CP                            # noqa: E402  every lib/ folder on sys.path

import mdfc_parts as mp                                 # noqa: E402
import range_sweep as RS                                # noqa: E402
importlib.reload(RS)
import lugs_brackets as LB                              # noqa: E402
importlib.reload(LB)

VERSION = 'StopBracket 1.0 (2026-11-11)'
CM = 0.1                                                # cm per mm
DEFAULT_R = 2.5                                         # mm, the dialog's default root radius
NAME = 'Stop bracket open %d'                           # the opening brackets as RangeSweep names them: 1 jaw A, 2 jaw B
_last = {}
_handlers = []

PLANE = adsk.core.SurfaceTypes.PlaneSurfaceType
CYLINDER = adsk.core.SurfaceTypes.CylinderSurfaceType


# ---------- small vector sums, world mm ----------
def _sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _mul(a, s):
    return tuple(v * s for v in a)


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return tuple(v / n for v in a)


def _mm(p):
    return (p.x / CM, p.y / CM, p.z / CM)


def _vec(v):
    return (v.x, v.y, v.z)


def _apply(turn, p):
    R, t = turn
    return tuple(sum(R[i][j] * p[j] for j in range(3)) + t[i] for i in range(3))


def _matrix(turn):
    """The turn (R, t in mm) as a Matrix3D on temporary bodies, which are in cm."""
    R, t = turn
    m = adsk.core.Matrix3D.create()
    m.setWithArray([R[0][0], R[0][1], R[0][2], t[0] * CM,
                    R[1][0], R[1][1], R[1][2], t[1] * CM,
                    R[2][0], R[2][1], R[2][2], t[2] * CM,
                    0.0, 0.0, 0.0, 1.0])
    return m


IDENTITY = (((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0))


# ---------- the model ----------
def _design():
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


def _find(test):
    """The occurrences, in the root's context, whose component's name passes the test."""
    root = _design().rootComponent
    return [root.allOccurrences.item(i) for i in range(root.allOccurrences.count)
            if test(root.allOccurrences.item(i).component.name)]


def _occ(name):
    hits = _find(lambda n: n == name)
    if not hits:
        raise RuntimeError('no component named %r in the bag gripper' % name)
    return hits[0]


def _centroid(f):
    try:
        return _mm(f.centroid)
    except Exception:
        return _mm(f.pointOnFace)


def _props(body):
    try:
        return body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
    except Exception:
        return body.physicalProperties


def _load_range_sweep():
    spec = importlib.util.spec_from_file_location('mdfc_sb62_RangeSweep', CP.script('RangeSweep'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _turn_of(g):
    """The machine's own half turn about z that makes jaw B's parts from jaw A's (g.turned, as RangeSweep uses it),
    read back as R and t (mm) by turning four small spheres."""
    turned = getattr(g, 'turned', None)
    if turned is None:
        raise RuntimeError("the bag gripper has no 'turned': jaw B's bracket cannot be made from jaw A's")

    def at(p):
        s0 = mp.sphere(p, 1.0)
        s = turned(s0) or s0
        try:
            return _mm(s.faces.item(0).geometry.origin)
        except Exception:
            bb = s.boundingBox
            return tuple((getattr(bb.minPoint, k) + getattr(bb.maxPoint, k)) / 2.0 / CM for k in 'xyz')
    o = at((0.0, 0.0, 0.0))
    cols = [_mul(_sub(at(e), o), 0.01) for e in ((100.0, 0.0, 0.0), (0.0, 100.0, 0.0), (0.0, 0.0, 100.0))]
    R = tuple(tuple(cols[j][i] for j in range(3)) for i in range(3))
    return (R, o)


def _world(pocc):
    """The bracket component's frame in the world (cm), checked on its body's vertices: the native vertex carried by
    the matrix must land on its proxy."""
    pb = pocc.bRepBodies.item(0)
    cands = []
    try:
        cands.append(pocc.transform2.copy())
    except Exception:
        pass
    cands.append(mp.world_matrix(pocc))
    cands.append(adsk.core.Matrix3D.create())
    n = min(6, pb.vertices.count)
    for M in cands:
        good = n > 0
        for i in range(n):
            pv = pb.vertices.item(i)
            nv = pv.nativeObject or pv
            q = nv.geometry.copy()
            q.transformBy(M)
            if q.distanceTo(pv.geometry) > 1e-6:
                good = False
                break
        if good:
            return M
    raise RuntimeError("no transform carries %s's body to where the model shows it" % pocc.component.name)


def _ref(k, turn):
    """Jaw A's opening stop in world mm (lugs_brackets.arm, turned), carried to bracket k by the machine's turn: the
    arm's centre on the foot's face, its axis, and the end plate's edges y and z."""
    A = LB.arm(True)
    c = A['centre']
    root, top = A['x']
    F = (lambda p: p) if k == 1 else (lambda p: _apply(turn, p))
    P0 = F((root, c[0], c[1]))
    return dict(P0=P0, X=_unit(_sub(F((root + 1.0, c[0], c[1])), P0)), Y=_unit(_sub(F((root, c[0] + 1.0, c[1])), P0)),
                Z=_unit(_sub(F((root, c[0], c[1] + 1.0)), P0)), h=top - root, a=A['a'])


def _base_feature(comp, body):
    bfs = comp.features.baseFeatures
    for i in range(bfs.count):
        bf = bfs.item(i)
        for j in range(bf.bodies.count):
            if bf.bodies.item(j) == body:
                return bf, j
    for i in range(bfs.count):
        bf = bfs.item(i)
        for j in range(bf.bodies.count):
            if bf.bodies.item(j).name == body.name:
                return bf, j
    raise RuntimeError('%s: no base feature holds its body' % comp.name)


# ---------- the change ----------
def _tools():
    """Jaw A's opening bracket in world mm: the arm as RangeSweep builds it (range_sweep.stop_layout's box), the arm
    turned (lugs_brackets.arm(True): its sides along the absorber's axis n and the bolts' line u, from the foot's face
    to the arm's top), the lug whole round the absorber's axis, and its bore for the DYSC-25's thread."""
    o = RS.stop_layout()['open']
    ar = o['arm']
    old = mp.box(ar['x'][0], ar['x'][1], ar['y'][0], ar['y'][1], ar['z'][0], ar['z'][1])
    A = LB.arm(True)
    x0, x1 = A['x']
    a = A['a']
    c = A['centre']
    e1 = A['e1']
    obb = adsk.core.OrientedBoundingBox3D.create(mp.P3((x0 + x1) / 2.0 * CM, c[0] * CM, c[1] * CM), mp.V3(1.0, 0.0, 0.0),
                                                 mp.V3(0.0, e1[0], e1[1]), (x1 - x0) * CM, a * CM, a * CM)
    new = mp.tbm().createBox(obb)
    P = o['at']
    t0, t1 = o['lug']['t']
    p0, p1 = P(t0), P(t1)
    lug = mp.cyl((o['x'], p0[0], p0[1]), (o['x'], p1[0], p1[1]), o['lug']['r'])
    q0, q1 = P(t0 - 1.0), P(t1 + 1.0)
    bore = mp.cyl((o['x'], q0[0], q0[1]), (o['x'], q1[0], q1[1]), RS.DYSC[25]['D1'] / 2.0)
    return dict(old=old, new=new, lug=lug, bore=bore,
                old_volume=(ar['x'][1] - ar['x'][0]) * (ar['y'][1] - ar['y'][0]) * (ar['z'][1] - ar['z'][0]))


def _change(k, turn):
    """Bracket k's body rebuilt in its base feature: the old arm cut off, the turned arm put back, the lug made whole
    and bored again. Its foot, M8 holes and lug stay where RangeSweep put them."""
    pocc = _occ(NAME % k)
    comp = pocc.component
    nb = comp.bRepBodies.item(0)
    mat = nb.material
    M = _world(pocc)
    Minv = M.copy()
    Minv.invert()
    T = _matrix(turn if k == 2 else IDENTITY)
    tl = _tools()
    for key in ('old', 'new', 'lug', 'bore'):
        mp.tbm().transform(tl[key], T)
        mp.tbm().transform(tl[key], Minv)
    tb = mp.tbm().copy(nb)
    # the arm must be where range_sweep puts it before it is cut off: all of the box is the bracket's material
    try:
        got = mp.intersect(mp.tbm().copy(tb), mp.tbm().copy(tl['old'])).volume / CM ** 3
    except Exception:
        got = None
    if got is not None and abs(got - tl['old_volume']) > 1e-3 * tl['old_volume']:
        raise RuntimeError('%s: the arm is not where range_sweep places it (%.1f of %.1f mm3 in its box)' % (
            comp.name, got, tl['old_volume']))
    mp.cut(tb, tl['old'])
    mp.union(tb, tl['new'])
    mp.union(tb, tl['lug'])
    mp.cut(tb, tl['bore'])
    bf, j = _base_feature(comp, nb)
    bf.startEdit()
    try:
        if not bf.updateBody(bf.bodies.item(j), tb):
            raise RuntimeError('%s: BaseFeature.updateBody refused the new body' % comp.name)
    finally:
        bf.finishEdit()
    try:
        _design().timeline.moveToEnd()
    except Exception:
        pass
    body = comp.bRepBodies.item(0)
    if mat is not None and (body.material is None or body.material.name != mat.name):
        body.material = mat
    return mat.name if mat is not None else None


def _fillet(k, r, ref):
    """The root's four edges (straight, a long, both ends on the foot's face) filleted r, a fillet feature on the
    bracket's own component."""
    pocc = _occ(NAME % k)
    comp = pocc.component
    pb = pocc.bRepBodies.item(0)
    edges = []
    for i in range(pb.edges.count):
        e = pb.edges.item(i)
        if e.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
            continue
        s0 = _dot(_sub(_mm(e.startVertex.geometry), ref['P0']), ref['X'])
        s1 = _dot(_sub(_mm(e.endVertex.geometry), ref['P0']), ref['X'])
        if abs(s0) > 1e-4 or abs(s1) > 1e-4 or abs(e.length / CM - ref['a']) > 1e-3:
            continue
        mid = _mul(tuple(a + b for a, b in zip(_mm(e.startVertex.geometry), _mm(e.endVertex.geometry))), 0.5)
        if _norm(_sub(mid, ref['P0'])) > ref['a']:
            continue
        edges.append(e.nativeObject or e)
    if len(edges) != 4:
        raise RuntimeError('%s: %d edges at the arm\'s root, not 4' % (comp.name, len(edges)))
    col = adsk.core.ObjectCollection.create()
    for e in edges:
        col.add(e)
    fl = comp.features.filletFeatures
    fi = fl.createInput()
    try:
        fi.edgeSetInputs.addConstantRadiusEdgeSet(col, mp.VI(r * CM), False)
    except AttributeError:
        fi.addConstantRadiusEdgeSet(col, mp.VI(r * CM), False)
    ff = fl.add(fi)
    try:
        ff.name = 'Arm root R%g' % r
    except Exception:
        pass
    try:
        return ff.faces.count
    except Exception:
        return None


# ---------- the measures ----------
def _absorber_axis(k):
    hits = _find(lambda n: n.startswith('Shock absorber') and n.endswith(' open %d' % k))
    if not hits:
        raise RuntimeError('no opening shock absorber %d' % k)
    b = hits[0].bRepBodies.item(0)
    for i in range(b.faces.count):
        g = b.faces.item(i).geometry
        if g.surfaceType == CYLINDER:
            return _unit(_vec(g.axis)), hits[0].component.name
    raise RuntimeError('%s has no cylindrical face' % hits[0].component.name)


def _measure(k, r, ref):
    """Bracket k as the model has it: the arm's sides (planar faces square to the arm's axis between the foot's face
    and the arm's top) against the absorber's axis and the plate's edges, the root's faces (cylinders of radius r at
    the root), the gaps from its two M8 heads (the two cap screws nearest the bracket) to the arm, volume, mass and
    material."""
    app = adsk.core.Application.get()
    mm_ = app.measureManager
    pocc = _occ(NAME % k)
    pb = pocc.bRepBodies.item(0)
    nb = pocc.component.bRepBodies.item(0)
    ax, ab_name = _absorber_axis(k)
    sides, fil = [], []
    for i in range(pb.faces.count):
        f = pb.faces.item(i)
        g = f.geometry
        d = _sub(_centroid(f), ref['P0'])
        s = _dot(d, ref['X'])
        lat = _norm(_sub(d, _mul(ref['X'], s)))
        if g.surfaceType == PLANE:
            nv = _unit(_vec(g.normal))
            if abs(_dot(nv, ref['X'])) < 1e-6 and 1e-3 < s < ref['h'] and lat < ref['a']:
                sides.append((f, nv))
        elif g.surfaceType == CYLINDER and r:
            if (abs(g.radius / CM - r) < 1e-3 and abs(_dot(_unit(_vec(g.axis)), ref['X'])) < 1e-6
                    and -1e-3 < s < r + 1e-3 and lat < ref['a']):
                fil.append((f, g.radius / CM))
    rows = []
    for f, nv in sides:
        th = math.degrees(math.acos(min(1.0, abs(_dot(nv, ax)))))
        plate = math.degrees(math.atan2(abs(_dot(nv, ref['Z'])), abs(_dot(nv, ref['Y']))))
        rows.append(dict(kind='square' if th < 45.0 else 'along', to_axis=th, off=min(th, 90.0 - th),
                         turned=min(plate, 90.0 - plate)))
    screws = [o for o in _find(lambda n: n.startswith('Cap screw ISO 4762')) if o.bRepBodies.count]
    near = sorted(((mm_.measureMinimumDistance(o.bRepBodies.item(0), pb).value / CM, o) for o in screws),
                  key=lambda x: x[0])[:2]
    faces = [f for f, _ in sides] + [f for f, _ in fil]
    gaps = [min(mm_.measureMinimumDistance(o.bRepBodies.item(0), f).value / CM for f in faces) if faces else None
            for _, o in near]
    pp = _props(nb)
    return dict(name=NAME % k, absorber=ab_name, axis=ax, sides=len(rows), side_rows=rows,
                square=sum(1 for x in rows if x['kind'] == 'square'), along=sum(1 for x in rows if x['kind'] == 'along'),
                worst=max(x['off'] for x in rows) if rows else None,
                turned=sum(x['turned'] for x in rows) / len(rows) if rows else None,
                fillets=len(fil), radii=[x for _, x in fil],
                heads=[o.component.name for _, o in near], head_touch=[d for d, _ in near], gaps=gaps,
                volume=pp.volume / CM ** 3, mass=pp.mass, material=nb.material.name if nb.material else None)


def _before_contacts():
    """The contacts made in motion in Chapter 56's sweep of the bag gripper with its stops, before the change."""
    try:
        with open(os.path.join(CP.ROOT, 'design_office', 'results', 'range_sweep.json'), encoding='utf-8') as f:
            rows = json.load(f)['bag']['rows']
        return [(x['a'], x['b']) for x in rows if x['min'] < 0.01]
    except Exception:
        return None


def _same(a, b):
    return a is not None and sorted(tuple(sorted(p)) for p in a) == sorted(tuple(sorted(p)) for p in b)


# ---------- build ----------
def build(r=DEFAULT_R):
    r = float(r)
    cl = LB.clearance(True, r)
    if not (r >= 0.0) or r >= cl['r_max']:
        raise ValueError('A root radius of %g mm refused: it must be 0 or more and below %.2f mm, where the fillet\'s toe '
                         'reaches the M8 heads (the heads %g across, %g mm either side of the arm\'s middle; '
                         'lugs_brackets.clearance).' % (r, cl['r_max'], cl['dk'], cl['d']))
    hv = LB.volume(True, r)
    hv0 = LB.volume(False, None)
    cl0 = LB.clearance(False, 0.0)
    rs56 = _load_range_sweep()
    states = RS.STATES['bag']()
    msg, setter = rs56.machine('bag')
    g = rs56._mods['bag']._last['g']
    turn = _turn_of(g)
    brackets = []
    for k in (1, 2):
        ref = _ref(k, turn)
        before = _measure(k, None, ref)
        kept = _change(k, turn)
        ff = _fillet(k, r, ref) if r > 0 else 0
        b = _measure(k, r, ref)
        b.update(hand=hv, err=(b['volume'] - hv) / hv, fillet_feature_faces=ff, material_before=kept,
                 hand_gap=cl['toe_drawn'], hand_gap_dk13=cl['toe'],
                 before=dict(volume=before['volume'], hand=hv0, worst=before['worst'], turned=before['turned'],
                             sides=before['sides'], gaps=before['gaps'], hand_gap=cl0['drawn'], mass=before['mass']))
        brackets.append(b)
    sw = rs56.sweep('bag', states, setter)
    contacts = [tuple(x['pair']) for x in sw['contacts']]
    was = _before_contacts()
    near = [dict(pair=tuple(x['pair']), min=x['min'], at=x['at'], max=x['max'], seen=x['seen'])
            for x in sw['rows'] if any(p.startswith('Stop bracket') for p in x['pair'])]
    sweep = dict(states=sw['states'], parts=sw['parts'], groups=sw['groups'], hits=sw['hits'], contacts=contacts,
                 before=was, same=_same(was, contacts), near=near, pairs=sw['pairs'], seconds=sw['seconds'],
                 rows=[dict(pair=tuple(x['pair']), min=x['min'], at=x['at'], max=x['max']) for x in sw['rows']])
    R, t = turn
    _last.clear()
    _last.update(r=r, version=VERSION, turn=dict(R=R, t=t, about=(t[0] / 2.0, t[1] / 2.0)), hand=dict(volume=hv, clearance=cl),
                 brackets=brackets, sweep=sweep, machine=msg.split('\n')[0])

    lines = ['Opening stop brackets (P13-018) on the bag gripper: the arm turned %.0f deg about its own axis, its root '
             'R %g mm; jaw B\'s bracket made from jaw A\'s by the gripper\'s half turn about z.' % (LB.TURN, r)]
    for b in brackets:
        bb = b['before']
        gaps = ' and '.join('%.2f' % x for x in b['gaps'] if x is not None)
        lines.append(
            '%s: the arm\'s %d sides, %d square to the absorber\'s axis and %d along it, the worst %.4f deg off (before '
            '%.2f deg off); turned %.2f deg from the plate\'s edges (before %.2f). Root R %g mm on %d faces. '
            '%.1f mm3 against %.1f by hand (%+.4f %%), %.4f kg, %s (before %.1f mm3, %.1f by hand). M8 heads %s mm from '
            'the arm (by hand %.2f with the heads drawn 12 across, %.2f at dk 13; before %s, by hand %.2f).' % (
                b['name'], b['sides'], b['square'], b['along'], b['worst'] if b['worst'] is not None else -1,
                bb['worst'] if bb['worst'] is not None else -1, b['turned'] or 0.0, bb['turned'] or 0.0, r,
                b['fillets'], b['volume'], b['hand'], 100.0 * b['err'], b['mass'], b['material'], bb['volume'],
                bb['hand'], gaps, b['hand_gap'], b['hand_gap_dk13'],
                ' and '.join('%.2f' % x for x in bb['gaps'] if x is not None), bb['hand_gap']))
    con = '; '.join('%s / %s' % c for c in contacts) or 'none'
    nr = '; '.join('%s / %s %.2f mm at %s' % (x['pair'][0], x['pair'][1], x['min'], x['at'][1]) for x in near[:6])
    lines.append(
        'Range sweep of the bag gripper (RangeSweep\'s own): %d states, %d parts in %d groups; %s; %d contacts made in '
        'motion: %s; %s. The brackets\' closest approaches: %s.' % (
            sweep['states'], sweep['parts'], sweep['groups'],
            'no interference' if not sweep['hits'] else 'interference at %d states: %s' % (len(sweep['hits']), sweep['hits'][:2]),
            len(contacts), con,
            'the same as before the change (range_sweep.json)' if sweep['same'] else
            'before the change (range_sweep.json): %s' % ('; '.join('%s / %s' % c for c in was) if was else 'not read'),
            nr or 'none within %g mm' % RS.GAP))
    lines.append(_last['machine'])
    lines.append('[%s]' % VERSION)
    return '\n'.join(lines)


# ---------- the command dialog ----------
class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            r = round(args.command.commandInputs.itemById('r').value / CM, 9)
            ui.messageBox(build(r))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('StopBracket failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        cmd.commandInputs.addValueInput('r', 'Root radius of the arm', 'mm',
                                        adsk.core.ValueInput.createByString('%g mm' % DEFAULT_R))


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcStopBracketTest'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Stop bracket',
                                                         'Chapter 62: the opening stop bracket with its arm turned')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StopBracket failed:\n%s' % traceback.format_exc())
