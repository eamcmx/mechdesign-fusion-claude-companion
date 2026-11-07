# =============================================================================
# StopBracket.py  -  Chapter 62 of "Mechanical Design in Autodesk Fusion with Claude": the opening stop bracket as
# Part 10 leaves it. It builds the bag gripper with Chapter 56's stops (RangeSweep.py), then in both opening brackets
# turns the arm 45 deg about its own axis, so that the DYSC's push lies square to a face instead of along a diagonal,
# and rounds the arm's root; the foot, its two M8, the lug and the absorber stay where they were. Then it measures the
# arm's faces against the absorber's axis, the root's fillets, each M8 head's clearance to the arm, the bracket's
# volume and mass, and sweeps the gripper through Chapter 56's states for interference.
# Written from PROMPT.txt beside it; checked by checks/stop_bracket_check.py.
#
#   INPUTS        the dialog: the root radius (2.5 mm by default)
#   DERIVED       lugs_brackets: TURN, arm(), clearance(), volume(); range_sweep: stop_layout(), DYSC, STATES
#   HARD RULES    the bracket changed in the machine's own model, nothing rebuilt beside it; nothing saved
#   CONVENTIONS   the machine's new design document; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the check: faces, fillets, clearance, volume, the sweep
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
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
SCRIPTS = os.path.dirname(HERE)
LIB = os.path.join(os.path.dirname(SCRIPTS), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import mdfc_parts as mp          # noqa: E402
import range_sweep as RS         # noqa: E402
import lugs_brackets as LB       # noqa: E402
importlib.reload(LB)

VERSION = 'StopBracket 1.0 (2026-11-07)'
TURNED = True                    # the arm's sides along the absorber's axis and the bolts' line
NAMES = ('Stop bracket open 1', 'Stop bracket open 2')
_last = {}
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_sb62_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _half_turn(tb):
    """Jaw B's end from jaw A's: half a turn about z, as the bag gripper makes it."""
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.pi, V3(0, 0, 1), P3(0, 0, 0))
    mp.tbm().transform(tb, m)
    return tb


def _arm_box(a_):
    """The arm as a temporary body, world mm: its sides along e1 and e2, from the foot's face up to the lug."""
    c = a_['centre']
    x0, x1 = a_['x']
    e1, e2 = a_['e1'], a_['e2']
    return mp.tbm().createBox(adsk.core.OrientedBoundingBox3D.create(
        P3((x0 + x1) / 20.0, c[0] / 10.0, c[1] / 10.0), V3(0, e1[0], e1[1]), V3(0, e2[0], e2[1]),
        a_['a'] / 10.0, a_['a'] / 10.0, (x1 - x0) / 10.0))


def tools(side):
    """What the change cuts and adds at jaw A's end (side 1) or jaw B's (side 2), world mm: the arm as Chapter 56
    made it, the arm turned, and the lug with its M30 bore (the arm's top lies inside the lug, so the lug is put back
    whole and bored again)."""
    o = RS.stop_layout()['open']
    old = o['arm']
    off = mp.box(old['x'][0], old['x'][1], old['y'][0], old['y'][1], old['z'][0], old['z'][1])
    on = _arm_box(LB.arm(TURNED))
    P = o['at']
    t0, t1 = o['lug']['t']
    p0, p1 = P(t0), P(t1)
    lug = mp.cyl((o['x'], p0[0], p0[1]), (o['x'], p1[0], p1[1]), o['lug']['r'])
    q0, q1 = P(t0 - 1.0), P(t1 + 1.0)
    bore = mp.cyl((o['x'], q0[0], q0[1]), (o['x'], q1[0], q1[1]), RS.DYSC[25]['D1'] / 2.0)
    out = [off, on, lug, bore]
    return [_half_turn(t) for t in out] if side == 2 else out


def turn_arm(tb, side=1):
    """The arm of an opening bracket given as a temporary body in world mm, turned: cut off, put back turned, the lug
    made whole and bored again. Chapter 63's drawings call this on their copy of the bracket."""
    off, on, lug, bore = tools(side)
    mp.cut(tb, off)
    mp.union(tb, on, lug)
    mp.cut(tb, bore)
    return tb


def corners(side, m=None):
    """The arm's four corners on the foot's face, world mm (or through the matrix m), side 2 turned half a turn."""
    out = []
    for p in LB.arm(TURNED)['corners']:
        q = (-p[0], -p[1], p[2]) if side == 2 else p
        if m is not None:
            pt = P3(q[0] / 10.0, q[1] / 10.0, q[2] / 10.0)
            pt.transformBy(m)
            q = (pt.x * 10.0, pt.y * 10.0, pt.z * 10.0)
        out.append(q)
    return out


def _near(v, p, tol=1e-3):
    g = v.geometry
    return abs(g.x * 10 - p[0]) < tol and abs(g.y * 10 - p[1]) < tol and abs(g.z * 10 - p[2]) < tol


def root_edges(body, cs):
    """The edges round the arm's root: both ends on two of its corners."""
    return [e for e in body.edges if any(_near(e.startVertex, a) for a in cs) and any(_near(e.endVertex, b) for b in cs)]


def change(occ, side, r):
    """Turn the arm of the bracket in this occurrence and fillet its root, in the machine's own model."""
    comp = occ.component
    body = comp.bRepBodies.item(0)
    mat = body.material
    W = mp.world_matrix(occ)
    Wi = W.copy()
    Wi.invert()
    tbm = mp.tbm()
    tb = tbm.copy(body)
    tbm.transform(tb, W)
    turn_arm(tb, side)
    tbm.transform(tb, Wi)
    bf = [f for f in comp.features.baseFeatures if f.bodies.count][0]
    bf.startEdit()
    bf.updateBody(bf.bodies.item(0), tb)
    bf.finishEdit()
    body = comp.bRepBodies.item(0)
    edges = root_edges(body, corners(side, Wi))
    if len(edges) != 4:
        raise RuntimeError('%s: %d edges round the arm\'s root, 4 expected' % (comp.name, len(edges)))
    if r > 0:
        coll = adsk.core.ObjectCollection.create()
        for e in edges:
            coll.add(e)
        fi = comp.features.filletFeatures.createInput()
        fi.addConstantRadiusEdgeSet(coll, adsk.core.ValueInput.createByReal(r / 10.0), True)
        comp.features.filletFeatures.add(fi)
        body = comp.bRepBodies.item(0)
    body.material = mat
    return body


def _face_rows(pb):
    """The arm's sides: plane faces square to x between the root's fillet and the lug, their normals (the bracket's
    body in the machine, world cm)."""
    rows = []
    for f in pb.faces:
        if f.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
            continue
        ok, n = f.evaluator.getNormalAtPoint(f.pointOnFace)
        c = f.centroid
        if abs(n.x) < 1e-6 and 437.0 < abs(c.x) * 10 < 465.0:
            rows.append((n.y, n.z))
    return rows


def measure(pb, side, screws, r):
    """The arm's faces against the absorber's axis, the root's fillets, the M8 heads' clearance to the arm, all on
    the bodies as the machine holds them (world)."""
    o = RS.stop_layout()['open']
    n = o['axis'] if side == 1 else (-o['axis'][0], o['axis'][1])
    sides = _face_rows(pb)
    ang = lambda a, b: math.degrees(math.acos(min(1.0, abs(a * n[0] + b * n[1]))))
    worst = max(min(ang(a, b), 90.0 - ang(a, b)) for a, b in sides) if sides else None
    fillets = [f for f in pb.faces if f.geometry.surfaceType in (adsk.core.SurfaceTypes.CylinderSurfaceType,
                                                                 adsk.core.SurfaceTypes.TorusSurfaceType)
               and abs(getattr(f.geometry, 'radius', getattr(f.geometry, 'minorRadius', 0)) * 10 - r) < 1e-4]
    near = []
    for f in pb.faces:
        x = abs(f.centroid.x) * 10
        top = f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType and abs(x - 437.0) < 1e-4
        if 437.0 - 1e-6 <= x < 461.0 and not top:
            near.append(f)
    mm = adsk.core.Application.get().measureManager
    gaps = [min(mm.measureMinimumDistance(s.bRepBodies.item(0), f).value * 10.0 for f in near) for s in screws]
    return dict(sides=len(sides), worst=worst, fillets=len(fillets), gaps=gaps)


def build(r=2.5):
    r = float(r)
    lim = LB.clearance(True, r)['r_max']
    if r < 0 or r >= lim:
        raise ValueError('A root radius of %g mm is refused: it must be at least 0 and under %.1f mm, where the '
                         'fillet\'s toe would reach the M8 heads\' seats.' % (r, lim))
    app = adsk.core.Application.get()
    t0 = time.time()
    rsm = _load('RangeSweep')
    msg, setter = rsm.machine('bag')
    doc = app.activeDocument
    try:
        doc.attributes.add('mdfc', 'run', 'StopBracket')
    except Exception:
        pass
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    o = RS.stop_layout()['open']
    rows = []
    for side, name in enumerate(NAMES, 1):
        occ = [x for x in leaves if x.component.name == name][0]
        body = change(occ, side, r)
        cy, cz = o['centre'] if side == 1 else (-o['centre'][0], o['centre'][1])
        sx = 1.0 if side == 1 else -1.0

        def by_bracket(x):
            b = x.bRepBodies.item(0).boundingBox
            c = [(b.minPoint.x + b.maxPoint.x) * 5, (b.minPoint.y + b.maxPoint.y) * 5, (b.minPoint.z + b.maxPoint.z) * 5]
            return sx * c[0] > 380 and abs(c[1] - cy) < 35 and abs(c[2] - cz) < 35
        screws = [x for x in leaves if x.component.name.startswith('Cap screw ISO 4762') and by_bracket(x)]
        if len(screws) != 2:
            raise RuntimeError('%s: %d M8 found by the bracket, 2 expected' % (name, len(screws)))
        m = measure(occ.bRepBodies.item(0), side, screws, r)
        m.update(name=name, volume=body.volume * 1000.0, mass=body.physicalProperties.mass,
                 material=body.material.name if body.material else None, faces=body.faces.count)
        rows.append(m)
    t_change = time.time() - t0
    sw = rsm.sweep('bag', RS.STATES['bag'](), setter)
    near = [x for x in sw['rows'] if any(p.startswith('Stop bracket open') for p in x['pair'])]
    out = dict(r=r, turned=TURNED, brackets=rows, hand_volume=LB.volume(TURNED, r), sweep=dict(
        states=sw['states'], parts=sw['parts'], hits=sw['hits'], contacts=[c['pair'] for c in sw['contacts']],
        near=[dict(pair=x['pair'], min=x['min']) for x in near[:6]], seconds=sw['seconds']), t_change=t_change,
        seconds=time.time() - t0)
    _last.clear()
    _last.update(out)
    b = rows[0]
    return ('Opening stop brackets: the arm %s, its %d sides within %.3f deg of square to the push; the root r %g, %d '
            'fillet faces; %.1f mm3, %.4f kg each (by hand %.1f mm3); the M8 heads %.2f and %.2f mm from the arm.\n'
            'Sweep of the bag gripper: %d states, %s, %d contacts made in motion.\n\n[%s]' % (
                'turned 45 deg' if TURNED else 'as Chapter 56 made it', b['sides'], b['worst'], r, b['fillets'],
                b['volume'], b['mass'], out['hand_volume'], b['gaps'][0], b['gaps'][1], sw['states'],
                'no interference' if not sw['hits'] else 'interference at %d states' % len(sw['hits']),
                len(sw['contacts']), VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            v = args.command.commandInputs.itemById('r').value * 10.0
            ui.messageBox(build(v))
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
        cmd.commandInputs.addValueInput('r', 'Root radius of the arm', 'mm', adsk.core.ValueInput.createByString('2.5 mm'))


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcStopBracket'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Stop bracket', 'Chapter 62: the opening stop bracket, its arm turned')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StopBracket failed:\n%s' % traceback.format_exc())
