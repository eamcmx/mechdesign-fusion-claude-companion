# =============================================================================
# FatigueSpots.py  -  Chapter 60 of "Mechanical Design in Autodesk Fusion with Claude": the fatigue spots at the bag
# gripper's ends, measured in the model the hand calculations of lib/fatigue_detail.py take their geometry from. It
# builds the bag gripper with its stops through RangeSweep.py, finds each spot's parts and measures them from their
# faces: the stop bolts' grip from the head's underside to the nut's face; whether the brackets' roots carry a fillet;
# the long stub's diameter at its keyseat, the key's size and the keyseat's depth; the short stub's groove and where it
# sits against its bearing. It colours the brackets, the key, the bolts and the nuts and writes the measures to
# results/fatigue_spots.json.
# Written from PROMPT.txt beside it; checked by checks/fatigue_spots_check.py.
#
#   INPUTS        the dialog: the spot (all by default)
#   DERIVED       the parts' names and the stop brackets from RangeSweep.py's build of the bag gripper
#   HARD RULES    nothing of the machines' scripts copied; built through RangeSweep.machine()
#   CONVENTIONS   a new design document; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the check: each spot measured as the hand calculation takes it
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib.util
import json
import math
import os
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
COMP = os.path.dirname(SCRIPTS)

VERSION = 'FatigueSpots 1.0 (2026-11-03)'
SPOTS = ('bolts', 'roots', 'keyseat', 'groove')
RED, YELLOW = 'Paint - Enamel Glossy (Red)', 'Paint - Enamel Glossy (Yellow)'
_last = {}


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_fs60_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _occ(leaves, name):
    for o in leaves:
        if o.component.name == name:
            return o
    raise RuntimeError('no part called %s' % name)


def _cylinders(o):
    out = []
    for b in o.bRepBodies:
        for f in b.faces:
            g = f.geometry
            if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                out.append((f, g.radius * 10.0, [g.origin.x * 10, g.origin.y * 10, g.origin.z * 10],
                            [g.axis.x, g.axis.y, g.axis.z]))
    return out


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _unit(v):
    n = math.sqrt(_dot(v, v)) or 1.0
    return [x / n for x in v]


def _edge_positions(face, axis):
    """Where a face's circular edges lie along an axis, mm."""
    out = []
    for e in face.edges:
        g = e.geometry
        if g.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
            c = g.center
            out.append(_dot([c.x * 10, c.y * 10, c.z * 10], axis))
    return sorted(set(round(x, 4) for x in out))


def _vertices(o):
    return [[v.geometry.x * 10, v.geometry.y * 10, v.geometry.z * 10] for b in o.bRepBodies for v in b.vertices]


def _nodes(o):
    """The points of a part's mesh, mm: round faces have no vertices at their extremes, a mesh has."""
    pts = []
    for b in o.bRepBodies:
        calc = b.meshManager.createMeshCalculator()
        calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
        c = calc.calculate().nodeCoordinatesAsDouble
        pts += [[c[k] * 10, c[k + 1] * 10, c[k + 2] * 10] for k in range(0, len(c), 3)]
    return pts


def _paint(des, occs, name):
    try:
        ap = des.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = des.appearances.addByCopy(lib.appearances.itemByName(name), name)
        for o in occs:
            o.appearance = ap
            for b in o.bRepBodies:                  # a body's own appearance outranks its occurrence's
                b.appearance = ap
    except Exception:
        pass


def measure_bolts(leaves):
    """The grip of the closing bracket's first bolt: from the head's underside (the head's cylinder's edge nearer the
    shank) to the nut's face nearer the head, along the bolt's axis."""
    bolt = _occ(leaves, 'Cap screw ISO 4762 M8x40 stop 1')
    nut = _occ(leaves, 'Nut ISO 4032 M8 stop 1')
    cyl = _cylinders(bolt)
    shank = max((c for c in cyl if abs(c[1] - 4.0) < 0.05), key=lambda c: c[0].area)
    head = max(cyl, key=lambda c: c[1])                       # the head: the bolt's largest cylinder
    ax = _unit(shank[3])
    s_shank = _edge_positions(shank[0], ax)
    s_head = _edge_positions(head[0], ax)
    under = min(s_head, key=lambda s: min(abs(s - t) for t in s_shank))
    top = max(s_head, key=lambda s: abs(s - under))
    planes = []
    for b in nut.bRepBodies:
        for f in b.faces:
            g = f.geometry
            if g.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType and abs(abs(_dot([g.normal.x, g.normal.y, g.normal.z], ax)) - 1) < 1e-6:
                p = f.pointOnFace
                planes.append(_dot([p.x * 10, p.y * 10, p.z * 10], ax))
    near = min(planes, key=lambda s: abs(s - under))
    return dict(grip=abs(near - under), head=abs(top - under), d=2 * shank[1], nut_faces=sorted(planes))


FILLET_MAX = 10.0         # mm: a cylindrical face this small that is not a hole is a fillet; the absorber's M30 bore
#                           and the lug's round end are larger


def measure_roots(leaves):
    """Each stop bracket's fillets: any cylindrical face under FILLET_MAX in radius that is not a bolt hole (9 across),
    and any torus; the bracket's larger round faces listed apart."""
    out = {}
    for name in ('Stop bracket closed 1', 'Stop bracket open 1'):
        o = _occ(leaves, name)
        radii = sorted(round(c[1], 3) for c in _cylinders(o))
        fillets = [r for r in radii if abs(r - 4.5) > 0.01 and r < FILLET_MAX]
        tori = sum(1 for b in o.bRepBodies for f in b.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.TorusSurfaceType)
        out[name] = dict(holes=sum(1 for r in radii if abs(r - 4.5) <= 0.01), fillets=fillets, tori=tori,
                         larger=[r for r in radii if r >= FILLET_MAX], sharp=not fillets and not tori)
    return out


def measure_keyseat(leaves):
    """The long stub at its keyseat and the key: the stub's diameter, the key's length along the axis, its height out
    from the axis and its width, and the keyseat's depth t1 = r - the key's nearest distance from the axis."""
    stub = _occ(leaves, 'Stub shaft long A')
    key = _occ(leaves, 'Key 8x7x25 A')
    pts = _nodes(key)
    c = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
    best = None
    for f, r, o, a in _cylinders(stub):
        a = _unit(a)
        v = [c[i] - o[i] for i in range(3)]
        along = _dot(v, a)
        radial = [v[i] - along * a[i] for i in range(3)]
        dist = math.sqrt(_dot(radial, radial))
        if abs(r - 15.0) < 0.05 and (best is None or f.area > best[0].area):
            best = (f, r, o, a, radial)
    f, r, o, a, radial = best
    n = _unit(radial)
    w = [a[1] * n[2] - a[2] * n[1], a[2] * n[0] - a[0] * n[2], a[0] * n[1] - a[1] * n[0]]
    rel = [[p[i] - o[i] for i in range(3)] for p in pts]
    along = [_dot(p, a) for p in rel]
    out_r = [_dot(p, n) for p in rel]
    across = [_dot(p, w) for p in rel]
    return dict(d=2 * r, L=max(along) - min(along), h=max(out_r) - min(out_r), b=max(across) - min(across),
                t1=r - min(out_r), key_out=max(out_r) - r)


def measure_groove(leaves):
    """The short stub's DIN 471 groove: its diameter and its place along the stub, against the bearing round the
    stub, both as distances from the gripper's middle (x = 0)."""
    stub = _occ(leaves, 'Stub shaft short A')
    cyl = _cylinders(stub)
    g = min(cyl, key=lambda c: c[1])
    ax = _unit(g[3])
    s_g = [abs(s) for s in _edge_positions(g[0], ax)]
    seat = max((c for c in cyl if abs(c[1] - 15.0) < 0.05), key=lambda c: c[0].area)
    s_seat = [abs(s) for s in _edge_positions(seat[0], ax)]
    brgs = [o for o in leaves if o.component.name.startswith('Bearing 6206')]

    def gap(o):
        ps = _vertices(o)
        c = [sum(p[i] for p in ps) / len(ps) for i in range(3)]
        v = [c[i] - g[2][i] for i in range(3)]
        al = _dot(v, ax)
        rr = [v[i] - al * ax[i] for i in range(3)]
        return math.sqrt(_dot(rr, rr)) + abs(abs(al) - max(s_seat))
    brg = min(brgs, key=gap)
    bp = [abs(_dot(p, ax)) for p in _vertices(brg)]
    return dict(d=2 * g[1], groove=(min(s_g), max(s_g)), bearing=(min(bp), max(bp)), beyond=min(s_g) >= max(bp) - 1e-6,
                bearing_name=brg.component.name)


def build(which='all'):
    spots = SPOTS if which == 'all' else (which,)
    if any(s not in SPOTS for s in spots):
        raise ValueError('No spot called %s: Chapter 60 measures %s.' % (which, ', '.join(SPOTS)))
    RS = _load('RangeSweep')
    msg, _setter = RS.machine('bag')
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    leaves = [o for o in des.rootComponent.allOccurrences if o.component.bRepBodies.count]
    fn = dict(bolts=measure_bolts, roots=measure_roots, keyseat=measure_keyseat, groove=measure_groove)
    out = {s: fn[s](leaves) for s in spots}
    names = lambda pre: [o for o in leaves if o.component.name.startswith(pre)]
    _paint(des, names('Stop bracket') + names('Key 8x7x25'), RED)
    _paint(des, names('Cap screw ISO 4762 M8x40 stop') + names('Nut ISO 4032 M8 stop'), YELLOW)   # the stubs keep their fits' colours
    json.dump(out, open(os.path.join(COMP, 'results', 'fatigue_spots.json'), 'w'), indent=1)
    _last.clear()
    _last.update(out)
    lines = []
    if 'bolts' in out:
        b = out['bolts']
        lines.append('bolts: the grip %.2f mm, the head %.2f, the shank %.2f across' % (b['grip'], b['head'], b['d']))
    if 'roots' in out:
        for k, v in out['roots'].items():
            lines.append('%s: %d bolt holes, fillets %s, %s (rounds over %g: %s)' % (k, v['holes'], v['fillets'] or 'none',
                     'sharp' if v['sharp'] else 'filleted', FILLET_MAX, v['larger']))
    if 'keyseat' in out:
        k = out['keyseat']
        lines.append('keyseat: the stub %.2f across, the key %.2f x %.2f x %.2f, the keyseat %.2f deep' % (k['d'], k['b'], k['h'], k['L'], k['t1']))
    if 'groove' in out:
        g = out['groove']
        lines.append('groove: %.2f across, from %.2f to %.2f mm out; the bearing %.2f to %.2f; beyond it: %s' % (
            g['d'], g['groove'][0], g['groove'][1], g['bearing'][0], g['bearing'][1], g['beyond']))
    return 'Fatigue spots: %d measured.\n%s\n%s\n\n[%s]' % (len(out), msg.splitlines()[0], '\n'.join(lines), VERSION)


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.command.commandInputs.itemById('spot').selectedItem.name))
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
        dd = cmd.commandInputs.addDropDownCommandInput('spot', 'Spot', adsk.core.DropDownStyles.TextListDropDownStyle)
        dd.listItems.add('all', True)
        for s in SPOTS:
            dd.listItems.add(s, False)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcFatigueSpots'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Fatigue spots', 'Chapter 60: the fatigue spots measured')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('FatigueSpots failed:\n%s' % traceback.format_exc())
