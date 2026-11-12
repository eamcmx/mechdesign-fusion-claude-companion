# =============================================================================
# PartDrawings.py  -  Chapter 63 of "Mechanical Design in Autodesk Fusion with Claude": the drawings. It builds each
# machine through RangeSweep.py's machine(), takes out of it the part Part 10 changed most (the five of
# lib/drawings_detail.py) into a design of its own, in the part's own frame, and asks Fusion for its automatic ISO
# drawing of it; on the sheet it adds what the checks decided (sizes, datums, a position frame) and a table of
# requirements with the source of each row, then exports the sheet as a PDF. A last design puts the bag gripper's
# opening stop together (the bracket, the DYSC-25 and its lock nut, the M8 screws, washers and nuts) for Fusion's
# assembly sheet with its parts list and the fitter's table.
# Written from PROMPT.txt (design_office/PartDrawings).
#
#   INPUTS        the dialog: the part (all, bracket, segment, spacer, fitting, disc or stop; all by default)
#   DERIVED       drawings_detail: PARTS, STOP_ITEMS, rows(), stop_rows(), assembly_items(); materials_fits:
#                 material(), FUSION; the classes read off the faces MaterialsFits.py left; RangeSweep.machine()
#   HARD RULES    machines built by RangeSweep.py, the arm turned by StopBracket.py (both loaded, not copied); a
#                 file never saved over another; a part not one of the six refused
#   CONVENTIONS   a new design document for each part and for the stop, no joints; ISO, first angle, A3, mm on
#                 the sheet, the API's cm in the model
#   ACCEPTANCE    the message; results/part_drawings.json; results/drawings/<number>.pdf
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import json
import math
import os
import sys
import time
import traceback

import adsk.core
import adsk.fusion
import adsk.drawing

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP     # noqa: E402  every lib/ folder on sys.path; CP.script(name) finds a script
import mdfc_parts as mp          # noqa: E402
import materials_fits as M       # noqa: E402
import lugs_brackets as LB       # noqa: E402
import fatigue_detail as FDT     # noqa: E402
import drawings_detail as DD     # noqa: E402
importlib.reload(M)
importlib.reload(LB)
importlib.reload(FDT)
importlib.reload(DD)             # only here: a caller may set DD.FOLDER after loading, build() reads it then

VERSION = 'PartDrawings 1.0 (2026-11-12)'
KEYS = tuple(p['key'] for p in DD.PARTS)
CHOICES = ('all',) + KEYS
# what the prompt asks each sheet to gain: a size given once for each set of equal holes (as '2 x Ø9' is written)
WANT = {'bracket': dict(dims=2, datums=1, frames=1),       # the holes' size, their distance; datum A; the position
        'segment': dict(dims=0, datums=2, frames=0),       # datums A and B; the profiles go in the table
        'spacer': dict(dims=1, datums=0, frames=0),        # the bore; datum A and the parallelism in the table
        'fitting': dict(dims=2, datums=0, frames=0),       # the bores' size, the bolt holes' size
        'disc': dict(dims=1, datums=0, frames=0),          # the bore
        'stop': dict(dims=0, datums=0, frames=0)}          # the parts list and the fitter's table only
TIDY = ('segment', 'spacer', 'disc')                       # Fusion sets the table over a view on these sheets
RESULTS = os.path.join(CP.ROOT, 'design_office', 'results')
PDF_DIR = os.path.join(RESULTS, 'drawings')
TABLE_HEADS = ('Requirement', 'Value', 'Source')
_last = {}

P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
ST = adsk.core.SurfaceTypes
CT = adsk.core.Curve3DTypes
VCT = adsk.drawing.ViewCurveTypes


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_pd63_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _wait(sec):
    """Wait while Fusion goes on drawing and saving."""
    t_end = time.time() + sec
    while True:
        adsk.doEvents()
        if time.time() >= t_end:
            break
        time.sleep(0.05)


# ---------- small vector helpers (tuples) ----------
def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _mul(a, k):
    return tuple(x * k for x in a)


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return tuple(x / n for x in a)


def _t3(p):
    return (p.x, p.y, p.z)


def _mm(p):
    return (p.x * 10.0, p.y * 10.0, p.z * 10.0)


def _off_line(p, o, a):
    """Distance of point p from the line through o along the unit vector a."""
    r = _sub(p, o)
    return _norm(_sub(r, _mul(a, _dot(r, a))))


def _docs():
    app = adsk.core.Application.get()
    return [app.documents.item(i) for i in range(app.documents.count)]


def _material(name):
    """A material of Fusion's Material Library by name, or None."""
    if not name:
        return None
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    try:
        return lib.materials.itemByName(name)
    except Exception:
        return None


def _family(name):
    """The model's own material as materials_fits.material() reads it (its FROM_MODEL names)."""
    if not name:
        return 'Steel'
    if name in M.FROM_MODEL:
        return name
    low = name.lower()
    if 'alumin' in low:
        return 'Aluminum'
    if 'steel' in low or 'iron' in low:
        return 'Steel'
    if 'rubber' in low:
        return 'Rubber'
    if 'abs' in low:
        return 'ABS Plastic'
    return name


# ---------- the machine ----------
def _world(occ):
    """The occurrence's transform to the machine's world, checked against where its body proxy actually lies."""
    cands = []
    try:
        cands.append(mp.world_matrix(occ))
    except Exception:
        pass
    cands.append(occ.transform2.copy())
    if len(cands) == 1:
        return cands[0]
    try:
        bn = occ.component.bRepBodies.item(0).boundingBox
        bp = occ.bRepBodies.item(0).boundingBox
        cn = P3((bn.minPoint.x + bn.maxPoint.x) / 2, (bn.minPoint.y + bn.maxPoint.y) / 2, (bn.minPoint.z + bn.maxPoint.z) / 2)
        cp = P3((bp.minPoint.x + bp.maxPoint.x) / 2, (bp.minPoint.y + bp.maxPoint.y) / 2, (bp.minPoint.z + bp.maxPoint.z) / 2)

        def err(m):
            q = cn.copy()
            q.transformBy(m)
            return q.distanceTo(cp)
        return min(cands, key=err)
    except Exception:
        return cands[0]


def _com_world(occ, W):
    """The centre of mass of the occurrence's bodies in the machine's world, cm."""
    tot, acc = 0.0, (0.0, 0.0, 0.0)
    for b in occ.component.bRepBodies:
        pp = b.physicalProperties
        c = pp.centerOfMass.copy()
        c.transformBy(W)
        v = b.volume
        tot += v
        acc = _add(acc, _mul(_t3(c), v))
    return _mul(acc, 1.0 / tot) if tot else (0.0, 0.0, 0.0)


def _copy_world(occ, W):
    """The occurrence's bodies as one temporary body in the machine's world."""
    tbm = mp.tbm()
    tb = None
    for nb in occ.component.bRepBodies:
        t = tbm.copy(nb)
        tbm.transform(t, W)
        tb = t if tb is None else mp.union(tb, t)
    return tb


def _find(leaves, name):
    """The part by its name in the machine: its component's name, or its occurrence's name less ':n'."""
    hits = [o for o in leaves if o.component.name == name] or [o for o in leaves if o.name.rsplit(':', 1)[0] == name]
    if not hits:
        word = name.split()[0].lower()
        like = sorted({o.component.name for o in leaves if word in o.component.name.lower()})
        raise RuntimeError('No part called %r in the machine; parts with a like name: %s' % (name, ', '.join(like[:10]) or 'none'))
    return hits[0]


def _fit_records(occ, des, W):
    """The classes and roughnesses MaterialsFits.py left on the part's faces (attribute 'mdfc' 'fit'), with each
    face's place in the machine's world (centroid, cm) so that the same face is found in the part's own design."""
    natives = list(occ.component.bRepBodies)
    recs = {}

    def add(f, value):
        nf = f.nativeObject if f.nativeObject is not None else f
        if not any(nf.body == nb for nb in natives):
            return
        key = nf.entityToken
        r = recs.get(key)
        if r is None:
            g = nf.geometry
            c = nf.centroid.copy()
            c.transformBy(W)
            r = dict(type=g.surfaceType, centroid=_t3(c), area=nf.area, values=[],
                     r_mm=(g.radius * 10.0 if g.surfaceType == ST.CylinderSurfaceType else None))
            recs[key] = r
        for v in str(value).split(' + '):
            v = v.strip()
            if v and v not in r['values']:
                r['values'].append(v)

    for nb in natives:
        for f in nb.faces:
            a = f.attributes.itemByName('mdfc', 'fit')
            if a:
                add(f, a.value)
    for pb in occ.bRepBodies:
        for f in pb.faces:
            a = f.attributes.itemByName('mdfc', 'fit')
            if a:
                add(f, a.value)
    try:
        for a in des.findAttributes('mdfc', 'fit'):
            ents = [a.parent]
            try:
                if a.otherParents:
                    ents += [a.otherParents.item(i) for i in range(a.otherParents.count)]
            except Exception:
                pass
            for e in ents:
                f = adsk.fusion.BRepFace.cast(e)
                if f is not None:
                    add(f, a.value)
    except Exception:
        pass
    return list(recs.values())


def _extract_part(spec, leaves, des, SB):
    occ = _find(leaves, spec['part'])
    W = _world(occ)
    natives = list(occ.component.bRepBodies)
    tb = _copy_world(occ, W)
    changes = {}
    for ch in spec.get('changes', []):
        if ch['kind'] == 'turn_arm':           # Chapter 62's own function, on this copy of the bracket
            SB.turn_arm(tb, ch.get('side', 1))
            changes['turn_arm'] = dict(turn=LB.TURN, side=ch.get('side', 1), by='StopBracket.turn_arm')
    mat = natives[0].material
    return dict(tb=tb, path=occ.fullPathName, bodies=len(natives), material=mat.name if mat else None,
                fits=_fit_records(occ, des, W), changes=changes,
                machine_volume=sum(b.volume for b in natives) * 1000.0)


def _extract_stop(spec, leaves, SB, bracket_spec):
    """The opening stop's items among the machine's parts (drawings_detail.assembly_items), each piece copied."""
    prefixes = [it[3] for it in DD.STOP_ITEMS]
    named, by_name = [], {}
    for o in leaves:
        nm = o.component.name
        if any(nm == pre or nm.startswith(pre) for pre in prefixes):
            W = _world(o)
            named.append((nm, _mul(_com_world(o, W), 10.0)))
            by_name.setdefault(nm, (o, W))
    items = DD.assembly_items(spec, named)
    out, changes = [], {}
    for it in items:
        pieces = []
        for nm in it['names']:
            o, W = by_name[nm]
            tb = _copy_world(o, W)
            if nm == spec['part']:
                for ch in bracket_spec.get('changes', []):
                    if ch['kind'] == 'turn_arm':
                        SB.turn_arm(tb, ch.get('side', 1))
                        changes['turn_arm'] = dict(turn=LB.TURN, side=ch.get('side', 1), by='StopBracket.turn_arm')
            nbs = list(o.component.bRepBodies)
            bb = nbs[0].boundingBox
            pieces.append(dict(name=nm, tb=tb, com=_com_world(o, W), volume=sum(b.volume for b in nbs),
                               ext=(bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z),
                               material=nbs[0].material.name if nbs[0].material else None))
        out.append(dict(number=it['number'], title=it['title'], designation=it['designation'], names=it['names'],
                        pieces=pieces))
    return dict(items=out, changes=changes)


def _from_machine(mk, mspecs, RSM, SB, MF, bracket_spec, mdocs):
    """Build one machine as its detail script builds it, take out what its drawings need (a part that cannot be
    taken out gets its error instead); the machine's documents go into mdocs, to be closed."""
    app = adsk.core.Application.get()
    before = _docs()
    try:
        msg, _setter = RSM.machine(mk)
    finally:
        mdocs.extend(d for d in _docs() if not any(d == b for b in before))
    note = msg.split('\n')[0]
    if any(p.get('apply51') for p in mspecs):   # the tool changer: its classes put on its faces first
        keys = list(MF.D.SCRIPTS)
        mf_key = mk if mk in keys else next((k for k in keys if 'chang' in k or 'tool' in k), mk)
        note += ' | ' + MF.apply(mf_key).split('\n')[0]
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    out = {}
    for p in mspecs:
        try:
            out[p['key']] = _extract_stop(p, leaves, SB, bracket_spec) if p.get('assembly') else _extract_part(p, leaves, des, SB)
        except Exception:
            out[p['key']] = 'taking it out of the %s: %s' % (mk, traceback.format_exc())
    return out, note


# ---------- the part's own design ----------
def _frame(body):
    """The part's frame: z along its largest cylinder's axis, x towards its centre of mass, the origin on that axis
    at the body's lowest point. Returns (part to world, world to part, what was found)."""
    app = adsk.core.Application.get()
    cyl = []
    z_from = 'its largest cylinder'
    for f in body.faces:
        g = f.geometry
        if g.surfaceType == ST.CylinderSurfaceType:
            cyl.append((g.radius, f.area, _t3(g.origin), _unit(_t3(g.axis))))
    if not cyl:                     # no cylinder at all: the largest cone, by the largest circle on its edges
        z_from = 'its largest cone (it has no cylinder)'
        for f in body.faces:
            g = f.geometry
            if g.surfaceType == ST.ConeSurfaceType:
                rs = [e.geometry.radius for e in f.edges
                      if e.geometry.curveType in (CT.Circle3DCurveType, CT.Arc3DCurveType)]
                cyl.append((round(max(rs), 7) if rs else g.radius, f.area, _t3(g.origin), _unit(_t3(g.axis))))
    if not cyl:
        raise RuntimeError('%s has no cylinder to take its z axis from' % body.name)
    R = max(c[0] for c in cyl)
    groups = []
    for c in cyl:
        if abs(c[0] - R) > 1e-7:
            continue
        for gr in groups:
            if abs(abs(_dot(gr['a'], c[3])) - 1.0) < 1e-7 and _off_line(c[2], gr['o'], gr['a']) < 1e-6:
                gr['area'] += c[1]
                break
        else:
            groups.append(dict(o=c[2], a=c[3], area=c[1]))
    gr = max(groups, key=lambda x: x['area'])
    o, z = gr['o'], gr['a']
    k = max((2, 1, 0), key=lambda i: round(abs(z[i]), 9))     # z along the world axis it lies nearest, up first
    if z[k] < 0:
        z = _mul(z, -1.0)
    com = _t3(body.physicalProperties.centerOfMass)
    r = _sub(com, o)
    xr = _sub(r, _mul(z, _dot(r, z)))
    how = 'towards the centre of mass'
    if _norm(xr) < 1e-5:
        w = min(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), key=lambda e: round(abs(_dot(e, z)), 9))
        xr = _sub(w, _mul(z, _dot(w, z)))
        how = 'the world axis most square to z (the centre of mass lies on the axis)'
    x = _unit(xr)
    y = _cross(z, x)
    tmin = None
    try:
        ob = app.measureManager.getOrientedBoundingBox(body, V3(*z), V3(*x))
        ext = ob.length
        for dv, e in ((ob.lengthDirection, ob.length), (ob.widthDirection, ob.width), (ob.heightDirection, ob.height)):
            if abs(_dot(_unit(_t3(dv)), z)) > 0.999:
                ext = e
        tmin = _dot(_sub(_t3(ob.centerPoint), o), z) - ext / 2.0
    except Exception:
        bb = body.boundingBox
        pts = [(a, b, c) for a in (bb.minPoint.x, bb.maxPoint.x) for b in (bb.minPoint.y, bb.maxPoint.y)
               for c in (bb.minPoint.z, bb.maxPoint.z)]
        tmin = min(_dot(_sub(p, o), z) for p in pts)
    origin = _add(o, _mul(z, tmin))
    F = adsk.core.Matrix3D.create()
    if not F.setWithCoordinateSystem(P3(*origin), V3(*x), V3(*y), V3(*z)):
        raise RuntimeError("the part's frame was refused (x %s, y %s, z %s)" % (x, y, z))
    Fi = F.copy()
    Fi.invert()
    return F, Fi, dict(origin_mm=list(_mul(origin, 10.0)), x=list(x), y=list(y), z=list(z), radius_mm=R * 10.0, x_is=how,
                       z_from=z_from)


def _place(comp, tb_world):
    """The body into a base feature of `comp` (its occurrence at the identity), first where the machine has it,
    then moved into its own frame."""
    tbm = mp.tbm()
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tbm.copy(tb_world), bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    F, Fi, frame = _frame(body)
    tbp = tbm.copy(tb_world)
    tbm.transform(tbp, Fi)
    bf.startEdit()
    bf.updateBody(bf.bodies.item(0), tbp)
    bf.finishEdit()
    return bf.bodies.item(0), F, Fi, frame


def _root_fillet(comp, body, SB, ch, side, Fi):
    """The arm's root rounded on its four edges, found by the arm's corners (drawings_detail's, world mm) moved into
    the part's frame."""
    if ch.get('corners'):
        cs = []
        for p in ch['corners']:
            q = P3(p[0] / 10.0, p[1] / 10.0, p[2] / 10.0)
            q.transformBy(Fi)
            cs.append(_mm(q))
    else:
        cs = SB.corners(side, Fi)
    edges = SB.root_edges(body, cs)
    if len(edges) != 4:
        raise RuntimeError('%d edges round the arm\'s root, 4 expected' % len(edges))
    coll = adsk.core.ObjectCollection.create()
    for e in edges:
        coll.add(e)
    fi = comp.features.filletFeatures.createInput()
    try:
        fi.addConstantRadiusEdgeSet(coll, VI(ch['r'] / 10.0), True)
    except Exception:
        fi.edgeSetInputs.addConstantRadiusEdgeSet(coll, VI(ch['r'] / 10.0), True)
    comp.features.filletFeatures.add(fi)
    return comp.bRepBodies.item(0), len(edges)


def _give_material(body, alloy, machine_name):
    m = _material(M.FUSION.get(alloy)) or _material(machine_name)
    if m is not None:
        body.material = m
    return body.material.name if body.material else None


def _carry_fits(body, recs, Fi):
    """The classes read off the machine's faces put on the same faces of the part's own body."""
    faces = list(body.faces)
    missed = []
    n = 0
    for rec in recs:
        c = P3(*rec['centroid'])
        c.transformBy(Fi)
        hit = None
        for f in faces:
            if f.geometry.surfaceType != rec['type']:
                continue
            if f.centroid.distanceTo(c) < 1e-3 and abs(f.area - rec['area']) <= 1e-4 * max(rec['area'], 1e-6):
                hit = f
                break
        if hit is None:
            missed.append(rec)
            continue
        old = hit.attributes.itemByName('mdfc', 'fit')
        vals = list(rec['values'])
        if old:
            vals = sorted(set(vals) | set(v.strip() for v in old.value.split(' + ')))
            old.deleteMe()
        hit.attributes.add('mdfc', 'fit', ' + '.join(sorted(vals)))
        n += 1
    return n, missed


def _read_fits(body, missed=()):
    """The classes as drawings_detail.fit_rows() takes them, read off the part's own faces (and any face of the
    machine's that could not be found again, as the machine has it)."""
    out = []
    for f in body.faces:
        a = f.attributes.itemByName('mdfc', 'fit')
        if not a:
            continue
        g = f.geometry
        kind = 'cylinder' if g.surfaceType == ST.CylinderSurfaceType else (
            'cone' if g.surfaceType == ST.ConeSurfaceType else 'plane')
        d = round(2.0 * g.radius * 10.0, 3) if kind == 'cylinder' else None
        for v in a.value.split(' + '):
            if v.strip():
                out.append(dict(kind=kind, d=d, value=v.strip(), on='part'))
    for rec in missed:
        kind = 'cylinder' if rec['type'] == ST.CylinderSurfaceType else 'plane'
        for v in rec['values']:
            out.append(dict(kind=kind, d=round(2.0 * rec['r_mm'], 3) if rec['r_mm'] else None, value=v, on='machine'))
    return out


def _cylinders(body):
    out = []
    for f in body.faces:
        g = f.geometry
        if g.surfaceType != ST.CylinderSurfaceType:
            continue
        o, a, r = _mm(g.origin), _unit(_t3(g.axis)), g.radius * 10.0
        p = f.pointOnFace
        ok, n = f.evaluator.getNormalAtPoint(p)
        P = _mm(p)
        rad = _sub(P, _add(o, _mul(a, _dot(_sub(P, o), a))))
        at = f.attributes.itemByName('mdfc', 'fit')
        out.append(dict(r=r, o=o, a=a, internal=bool(ok) and _dot(_t3(n), rad) < 0, area=f.area * 100.0,
                        fit=at.value if at else None))
    return out


def _samples(face, n=10):
    """Points of a face with its normals (part mm), inside it and along its edges, to find it on the sheet."""
    ev = face.evaluator
    out = []
    try:
        ok, lo, hi = ev.getParameterExtents()
    except Exception:
        ok = False
    if ok:
        for i in range(n):
            for j in range(n):
                prm = adsk.core.Point2D.create(lo.x + (hi.x - lo.x) * (i + 0.5) / n, lo.y + (hi.y - lo.y) * (j + 0.5) / n)
                if not ev.isParameterOnFace(prm):
                    continue
                ok1, p = ev.getPointAtParameter(prm)
                ok2, nv = ev.getNormalAtParameter(prm)
                if ok1 and ok2:
                    out.append((_mm(p), _unit(_t3(nv)), False))
    for e in face.edges:
        try:
            ok, t0, t1 = e.evaluator.getParameterExtents()
        except Exception:
            continue
        if not ok:
            continue
        for k in range(9):
            ok1, p = e.evaluator.getPointAtParameter(t0 + (t1 - t0) * k / 8.0)
            if not ok1:
                continue
            ok2, nv = ev.getNormalAtPoint(p)
            if ok2:
                out.append((_mm(p), _unit(_t3(nv)), True))
    return out


def _outward(f):
    ok, n = f.evaluator.getNormalAtPoint(f.pointOnFace)
    return _unit(_t3(n))


def _incline(f):
    """The face's angle to the part's z axis, degrees: a plane's or a cone's (None for others)."""
    g = f.geometry
    if g.surfaceType == ST.PlaneSurfaceType:
        n = _unit(_t3(g.normal))
        return 90.0 - math.degrees(math.acos(min(1.0, abs(n[2]))))
    if g.surfaceType == ST.ConeSurfaceType:
        return math.degrees(g.halfAngle)
    return None


def _datum_faces(key, body):
    """The faces the datums go on, found by their geometry in the part's frame."""
    out, notes = {}, {}
    planes = [f for f in body.faces if f.geometry.surfaceType == ST.PlaneSurfaceType]
    if key == 'bracket':
        # the foot's bolting face: square to x, the body's far face along x (x points from the lug to the foot)
        c = [f for f in planes if _dot(_outward(f), (1.0, 0.0, 0.0)) > 0.999]
        if c:
            f = max(c, key=lambda q: q.centroid.x)
            out['A'] = _samples(f)
            notes['A'] = "the foot's bolting face at x = %.2f mm" % (f.centroid.x * 10)
    elif key == 'segment':
        c = [f for f in planes if _dot(_outward(f), (0.0, 0.0, 1.0)) > 0.999]
        if c:
            f = max(c, key=lambda q: q.centroid.z)
            out['A'] = _samples(f)
            notes['A'] = 'the top face at z = %.2f mm' % (f.centroid.z * 10)
        cand = []
        for f in body.faces:
            th = _incline(f)
            if th is None:
                continue
            if abs(th - 15.0) < 0.5:
                cand.append((0, f.area, f, th))
            elif abs(th - 75.0) < 0.5:
                cand.append((1, f.area, f, th))
        if cand:
            cand.sort(key=lambda t: (t[0], -t[1]))
            f, th = cand[0][2], cand[0][3]
            out['B'] = _samples(f)
            notes['B'] = 'the face at %.2f deg to the axis (%s)' % (th, 'cone' if f.geometry.surfaceType == ST.ConeSurfaceType else 'plane')
    return out, notes


def _model_data(key, body):
    """What the drawing needs of the model, read while its design is open: points to register the views by, the
    largest radius, the cylinders, the datum faces."""
    E = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    keys, rmax = [], 0.0
    for v in body.vertices:
        keys.append(_mm(v.geometry))
    for e in body.edges:
        g = e.geometry
        if g.curveType not in (CT.Circle3DCurveType, CT.Arc3DCurveType):
            continue
        C, n, r = _mm(g.center), _unit(_t3(g.normal)), g.radius * 10.0
        rmax = max(rmax, r)
        keys.append(C)
        for ax in E:
            if abs(_dot(n, ax)) < 1e-6:
                keys.append(_add(C, _mul(ax, r)))
                keys.append(_sub(C, _mul(ax, r)))
    seen, uniq = set(), []
    for k in keys:
        t = tuple(round(x, 2) for x in k)
        if t not in seen:
            seen.add(t)
            uniq.append(k)
    cyl = _cylinders(body)
    if rmax == 0.0 and cyl:
        rmax = max(c['r'] for c in cyl)
    faces, notes = _datum_faces(key, body)
    return dict(keys=uniq, rmax=rmax, cylinders=cyl, datum_faces=faces, datum_notes=notes)


def _radii(body):
    out = {}
    for f in body.faces:
        g = f.geometry
        if g.surfaceType == ST.CylinderSurfaceType:
            k = '%g' % round(g.radius * 10.0, 3)
            out[k] = out.get(k, 0) + 1
    return out


def _snapshot(des):
    """Placed parts held where they were put (a transform in a parametric design is pending until a snapshot)."""
    try:
        if des.snapshots.hasPendingSnapshot:
            des.snapshots.add()
    except Exception:
        pass


def _new_design():
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    try:
        des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    return doc, des


def _part_design(spec, ex, SB, machine):
    doc, des = _new_design()
    root = des.rootComponent
    body, F, Fi, frame = _place(root, ex['tb'])
    changes = dict(ex['changes'])
    side = (changes.get('turn_arm') or {}).get('side', 1)
    for ch in spec.get('changes', []):
        if ch['kind'] == 'root_fillet':
            body, n = _root_fillet(root, body, SB, ch, side, Fi)
            changes['root_fillet'] = dict(r=ch['r'], edges=n)
    body.name = spec['title']
    for attr, val in (('partNumber', spec['number']), ('description', spec['title'])):
        try:
            setattr(root, attr, val)
        except Exception:
            pass
    try:
        material = M.material(machine, spec['kind'], _family(ex['material']))
    except Exception:
        material = (_family(ex['material']).lower(), None, 'the model')
    given = _give_material(body, material[0], ex['material'])
    carried, missed = _carry_fits(body, ex['fits'], Fi)
    fits = _read_fits(body, missed)
    rows = [list(r) for r in DD.rows(spec['key'], fits, material)]
    _snapshot(des)
    return dict(doc=doc, des=des, body=body, frame=frame, changes=changes, material=list(material), given=given,
                fits=fits, carried=carried, missed=len(missed), rows=rows, volume=body.volume * 1000.0,
                model=dict(radii=_radii(body), faces=body.faces.count, bodies_in_machine=ex['bodies'],
                           machine_volume=ex['machine_volume'], path=ex['path']),
                md=_model_data(spec['key'], body))


def _stop_design(spec, ex, SB, bracket_spec):
    """The opening stop put together: one component for each kind of item with its part number and designation,
    a piece of each where the machine has it, in the bracket's frame; no joints."""
    doc, des = _new_design()
    root = des.rootComponent
    for attr, val in (('partNumber', spec['number']), ('description', spec['title'])):
        try:
            setattr(root, attr, val)
        except Exception:
            pass
    tbm = mp.tbm()
    items = ex['items']
    bi = [it for it in items if spec['part'] in it['names']][0]

    def component(it):
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = it['title']
        if it['number']:
            comp.partNumber = it['number']
        comp.description = it['designation']
        return occ, comp

    occ_b, comp_b = component(bi)
    body, F, Fi, frame = _place(comp_b, bi['pieces'][0]['tb'])
    changes = dict(ex['changes'])
    side = (changes.get('turn_arm') or {}).get('side', 1)
    for ch in bracket_spec.get('changes', []):
        if ch['kind'] == 'root_fillet':
            body, n = _root_fillet(comp_b, body, SB, ch, side, Fi)
            changes['root_fillet'] = dict(r=ch['r'], edges=n)
    body.name = bi['title']
    try:
        alloy = M.material(bracket_spec['machine'], bracket_spec['kind'], _family(bi['pieces'][0]['material']))[0]
    except Exception:
        alloy = None
    _give_material(body, alloy, bi['pieces'][0]['material'])
    made = [(bi, comp_b, [])]
    extra = []
    for it in items:
        if it is bi:
            continue
        occ, comp = component(it)
        p0 = it['pieces'][0]
        t = tbm.copy(p0['tb'])
        tbm.transform(t, Fi)
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        comp.bRepBodies.add(t, bf)
        bf.finishEdit()
        b = bf.bodies.item(0)
        b.name = it['title']
        m = _material(p0['material'])
        if m is not None:
            b.material = m
        notes = []
        made.append((it, comp, notes))
        c0 = P3(*p0['com'])
        c0.transformBy(Fi)
        for pk in it['pieces'][1:]:
            same = abs(pk['volume'] - p0['volume']) <= 1e-6 * p0['volume'] and \
                all(abs(a - b_) < 1e-4 for a, b_ in zip(pk['ext'], p0['ext']))
            ck = P3(*pk['com'])
            ck.transformBy(Fi)
            if same:
                extra.append((comp, V3(ck.x - c0.x, ck.y - c0.y, ck.z - c0.z), pk['name']))
            else:                              # not a moved copy of the first: its own body in the same component
                t2 = tbm.copy(pk['tb'])
                tbm.transform(t2, Fi)
                bf2 = comp.features.baseFeatures.add()
                bf2.startEdit()
                comp.bRepBodies.add(t2, bf2)
                bf2.finishEdit()
                notes.append('%s as a second body' % pk['name'])
    placed = []
    for comp, d, nm in extra:                  # the second pieces last, so that no later feature puts them back
        m = adsk.core.Matrix3D.create()
        m.translation = d
        placed.append((root.occurrences.addExistingComponent(comp, m), d, nm))
    _snapshot(des)
    for o, d, nm in placed:                    # read back where each piece stands
        t = o.transform2.translation
        err = math.sqrt((t.x - d.x) ** 2 + (t.y - d.y) ** 2 + (t.z - d.z) ** 2) * 10.0
        if err > 1e-4:
            for it, comp, notes in made:
                if comp == o.component:
                    notes.append('%s stands %.4f mm from the machine\'s place' % (nm, err))
    out_items = []
    for it, comp, notes in made:
        occs = [o for o in root.occurrences if o.component == comp]
        n = len(occs) + sum(1 for x in notes if 'second body' in x)
        out_items.append(dict(number=it['number'], title=it['title'], designation=it['designation'], n=n,
                              names=it['names'], part_number=comp.partNumber, description=comp.description,
                              notes=notes))
    return dict(doc=doc, des=des, body=body, frame=frame, changes=changes, items=out_items,
                rows=[list(r) for r in DD.stop_rows()], volume=body.volume * 1000.0,
                model=dict(radii=_radii(body), faces=body.faces.count), md=None)


# ---------- the cloud folder ----------
def _folder(path):
    app = adsk.core.Application.get()
    if isinstance(path, str):
        path = [p for p in path.replace('\\', '/').split('/') if p]
    path = list(path)
    proj = None
    for p in app.data.dataProjects.asArray():
        if p.name == path[0]:
            proj = p
            break
    if proj is None:
        proj = app.data.dataProjects.add(path[0])
    if proj is None:
        raise RuntimeError('No project %r and none could be made' % path[0])
    f = proj.rootFolder
    for name in path[1:]:
        g = f.dataFolders.itemByName(name)
        if g is None:
            g = f.dataFolders.add(name)
        if g is None:
            raise RuntimeError('No folder %r in %s and none could be made' % (name, '/'.join(path)))
        f = g
    return f, '/'.join(path)


def _save(doc, folder, base):
    """Save as '<number> <title>', or with ' (2)' and on when a file of that name is there already."""
    names = {f.name for f in folder.dataFiles.asArray()}
    name, k = base, 2
    while name in names:
        name = '%s (%d)' % (base, k)
        k += 1
    doc.activate()
    if not doc.saveAs(name, folder, 'Chapter 63: %s' % base, ''):
        raise RuntimeError('Fusion did not save %s' % name)
    t0 = time.time()
    while time.time() - t0 < 180.0:
        try:
            if doc.dataFile is not None:
                return name
        except Exception:
            pass
        _wait(1.0)
    raise RuntimeError('%s saved but no data file came back' % name)


def _names(folder, want, timeout=90.0):
    t0 = time.time()
    names = []
    while time.time() - t0 < timeout:
        names = ['%s.%s' % (f.name, f.fileExtension) for f in folder.dataFiles.asArray()]
        if len(names) >= want:
            break
        _wait(3.0)
    return names


# ---------- the drawing ----------
def _set(obj, attr, val):
    try:
        setattr(obj, attr, val)
        return True
    except Exception:
        return False


def _drawing_input(inp, assembly, tailored):
    _set(inp, 'standard', adsk.drawing.DrawingStandardTypes.ISODrawingStandardType)
    _set(inp, 'units', adsk.drawing.DrawingUnitTypes.MillimeterDrawingUnitType)
    _set(inp, 'sheetSize', adsk.drawing.SheetSizes.A3ISOSheetSize)
    _set(inp, 'orientationType', adsk.drawing.SheetOrientationTypes.LandscapeSheetOrientationType)
    _set(inp, 'content', adsk.drawing.DrawingContentTypes.FullAssemblyDrawingContentType)
    if not tailored:
        return
    try:
        ap = inp.automationPreferences
        gp = ap.globalPreferences
    except Exception:
        return
    for attr in ('isAnimationSheetGenerated', 'isSubAssemblySheetGenerated', 'isFoldedModelSheetGenerated',
                 'isFlatPatternSheetGenerated'):
        _set(gp, attr, False)
    if assembly:                    # the assembly's own sheets: the ISO view with its parts list, the projections
        _set(gp, 'isMainAssemblySheetGenerated', True)
        _set(gp, 'isComponentSheetGenerated', False)
        _set(gp, 'omitComponentsWithKeywords', '')
        try:
            ma = ap.mainAssemblyPreferences
            _set(ma.isoViewSheetPreferences, 'isSheetCreated', True)
            _set(ma.isoViewSheetPreferences, 'isPartsListIncluded', True)
            _set(ma.orthogonalViewSheetPreferences, 'isSheetCreated', True)
        except Exception:
            pass
    else:                           # a part: its one sheet of projections
        _set(gp, 'isMainAssemblySheetGenerated', False)
        _set(gp, 'isComponentSheetGenerated', True)
        try:
            _set(ap.componentPreferences.sheetViewPreferences, 'isOrthogonalViewAdded', True)
        except Exception:
            pass


def _create_drawing(df, assembly, timeout=420.0):
    """Fusion's automatic drawing of the saved design, asked for again every 10 s while the design is not ready.
    The first asks carry the sheets wanted; later ones alternate with Fusion's defaults."""
    dm = adsk.drawing.DrawingManager.get()
    t0, k, errs = time.time(), 0, []
    while True:
        tailored = k < 4 or k % 2 == 0
        try:
            inp = dm.createDrawingInput(df, adsk.drawing.DrawingCreationModes.AutomaticDrawingCreationMode)
            _drawing_input(inp, assembly, tailored)
            out = dm.createDrawing(inp)
            if out:
                return out, dict(asks=k + 1, tailored=tailored, seconds=round(time.time() - t0, 1), errors=errs[-3:])
            errs.append('no drawing returned')
        except Exception as e:
            errs.append(str(e).strip().split('\n')[-1][:200])
        k += 1
        if time.time() - t0 > timeout:
            raise RuntimeError('Fusion made no drawing of %s in %.0f s: %s' % (df.name, time.time() - t0, errs[-1]))
        _wait(10.0)


def _open_doc(df, timeout=180.0):
    app = adsk.core.Application.get()
    t0 = time.time()
    while time.time() - t0 < timeout:
        for d in _docs():
            try:
                if d.dataFile is not None and d.dataFile.id == df.id:
                    return d
            except Exception:
                pass
        try:
            d = app.documents.open(df, True)
            if d:
                return d
        except Exception:
            pass
        _wait(5.0)
    raise RuntimeError('the drawing %s could not be opened' % df.name)


def _dims_all(drawing):
    sh = drawing.sheets
    return sum(sh.item(i).drawingDimensions.count for i in range(sh.count))


def _settle(drawing, ui, timeout=90.0):
    """End the auto-dimension command the automatic drawing leaves open and wait until the count of dimensions
    stays the same."""
    t0 = time.time()
    last, same, ended = None, 0, 0
    try:
        ui.terminateActiveCommand()
        ended += 1
    except Exception:
        pass
    while time.time() - t0 < timeout:
        try:
            ac = ui.activeCommand
            if ac and ac != 'SelectCommand':
                ui.terminateActiveCommand()
                ended += 1
        except Exception:
            pass
        _wait(0.5)
        try:
            n = _dims_all(drawing)
        except Exception:
            continue
        if n == last:
            same += 1
        else:
            last, same = n, 0
        if same >= 6 and time.time() - t0 >= 4.0:
            break
    return dict(dims=last, terminated=ended, seconds=round(time.time() - t0, 1))


def _pt(dp):
    if dp is None:
        return None
    c = dp.coordinate
    return (c.x, c.y)


def _enum_names(cls, suffix):
    return {getattr(cls, n): n for n in dir(cls) if n.endswith(suffix)}


def _views(sheet):
    """Each view's curves in sheet mm; an arc's or a circle's radius from its start point to its centre."""
    kinds = {VCT.LineViewCurveType: 'line', VCT.ArcViewCurveType: 'arc', VCT.CircleViewCurveType: 'circle'}
    vt = _enum_names(adsk.drawing.ViewTypes, 'ViewType')
    out = []
    for i in range(sheet.views.count):
        v = sheet.views.item(i)
        curves = []
        vc = None
        try:
            vc = v.viewCurves
        except Exception:
            pass
        for j in range(vc.count if vc else 0):
            try:
                c = vc.item(j)
                kind = kinds.get(c.type, 'other')
                p0, p1 = _pt(c.startPoint), _pt(c.endPoint)
                cen = _pt(c.centerPoint) if kind in ('arc', 'circle') else None
            except Exception:
                continue
            r = math.hypot(p0[0] - cen[0], p0[1] - cen[1]) if (cen and p0) else None
            curves.append(dict(kind=kind, obj=c, p0=p0, p1=p1, c=cen, r=r))
        xs, ys = [], []
        for c in curves:
            for p in (c['p0'], c['p1']):
                if p:
                    xs.append(p[0])
                    ys.append(p[1])
            if c['kind'] == 'circle' and c['c']:
                xs += [c['c'][0] - c['r'], c['c'][0] + c['r']]
                ys += [c['c'][1] - c['r'], c['c'][1] + c['r']]
        centre = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2) if xs else (0.0, 0.0)
        try:
            tname = vt.get(v.type, str(v.type))
        except Exception:
            tname = 'unknown'
        out.append(dict(obj=v, index=i, type=tname, curves=curves, centre=centre, map=None))
    return out


_AX = ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, -1.0))


def _near(grid, q, tol):
    cx, cy = int(math.floor(q[0])), int(math.floor(q[1]))
    best, bd = None, tol
    for i in (cx - 1, cx, cx + 1):
        for j in (cy - 1, cy, cy + 1):
            for p in grid.get((i, j), ()):
                d = math.hypot(p[0] - q[0], p[1] - q[1])
                if d <= bd:
                    best, bd = p, d
    return best


def _register(view, keys, s, tol=0.3):
    """Which of the part's axes run along the sheet's x and y in this view, and where its origin lies: the
    orthographic projection (of the 24) that lays the most curve ends and centres on the model's points."""
    pts = []
    for c in view['curves']:
        if c['kind'] == 'line':
            pts += [c['p0'], c['p1']]
        elif c['kind'] == 'arc':
            pts += [c['p0'], c['p1'], c['c']]
        elif c['kind'] == 'circle':
            pts.append(c['c'])
    pts = [p for p in pts if p]
    seen, uniq = set(), []
    for p in pts:
        t = (round(p[0], 2), round(p[1], 2))
        if t not in seen:
            seen.add(t)
            uniq.append(p)
    pts = uniq
    if len(pts) < 4 or not s or not keys:
        return None
    step = max(1, len(pts) // 24)
    anchors = pts[::step][:24]
    best = None
    for a in _AX:
        for b in _AX:
            if abs(_dot(a, b)) > 0.5:
                continue
            P = [(s * _dot(k, a), s * _dot(k, b)) for k in keys]
            grid = {}
            for q in P:
                grid.setdefault((int(math.floor(q[0])), int(math.floor(q[1]))), []).append(q)
            votes = {}
            for sx, sy in anchors:
                for px, py in P:
                    kk = (int(round((sx - px) * 2.0)), int(round((sy - py) * 2.0)))
                    votes[kk] = votes.get(kk, 0) + 1
            for kk, _n in sorted(votes.items(), key=lambda t: -t[1])[:3]:
                O = (kk[0] / 2.0, kk[1] / 2.0)
                for _ in range(2):
                    res = []
                    for p in pts:
                        q = _near(grid, (p[0] - O[0], p[1] - O[1]), 0.8)
                        if q:
                            res.append((p[0] - O[0] - q[0], p[1] - O[1] - q[1]))
                    if res:
                        O = (O[0] + sum(r[0] for r in res) / len(res), O[1] + sum(r[1] for r in res) / len(res))
                hit = sum(1 for p in pts if _near(grid, (p[0] - O[0], p[1] - O[1]), tol))
                score = hit / float(len(pts))
                if best is None or score > best['score'] + 1e-9:
                    best = dict(a=a, b=b, O=O, score=score, s=s)
    return best if best and best['score'] >= 0.5 else None


def _dseg(q, p0, p1):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(q[0] - p0[0], q[1] - p0[1])
    t = max(0.0, min(1.0, ((q[0] - p0[0]) * dx + (q[1] - p0[1]) * dy) / L2))
    return math.hypot(q[0] - p0[0] - t * dx, q[1] - p0[1] - t * dy)


def _face_line(views, samples):
    """The line on the sheet that is the face seen edge on (or its outline): in a view whose projection is known,
    the line along which the face's points fall, the face running along the line there."""
    best = None
    for vi, v in enumerate(views):
        mp_ = v.get('map')
        if not mp_:
            continue
        a, b, O, s = mp_['a'], mp_['b'], mp_['O'], mp_['s']
        d = _cross(a, b)
        proj = []
        for P, n, edge in samples:
            if not edge and abs(_dot(n, d)) > 0.08:
                continue
            proj.append(((O[0] + s * _dot(P, a), O[1] + s * _dot(P, b)), n))
        if len(proj) < 3:
            continue
        for c in v['curves']:
            if c['kind'] != 'line' or not c['p0'] or not c['p1']:
                continue
            ux, uy = c['p1'][0] - c['p0'][0], c['p1'][1] - c['p0'][1]
            L = math.hypot(ux, uy)
            if L < 0.5:
                continue
            D = _add(_mul(a, ux / L), _mul(b, uy / L))
            cnt = sum(1 for q, n in proj if abs(_dot(D, n)) <= 0.08 and _dseg(q, c['p0'], c['p1']) <= 0.25)
            if cnt >= 3 and (best is None or cnt > best[0] or (cnt == best[0] and L > best[3])):
                best = (cnt, vi, c, L)
    return best


def _away(v, p):
    ux, uy = p[0] - v['centre'][0], p[1] - v['centre'][1]
    L = math.hypot(ux, uy)
    if L < 1e-6:
        return (math.sqrt(0.5), math.sqrt(0.5))
    return (ux / L, uy / L)


def _dpt(x, y):
    return adsk.drawing.DrawingPoint.create(adsk.core.Point2D.create(x, y))


def _circles(views, r, tol=0.05):
    """The curves where a hole of radius r (sheet mm) shows round, circles before arcs."""
    out = []
    for vi, v in enumerate(views):
        for c in v['curves']:
            if c['kind'] in ('circle', 'arc') and c['r'] is not None and abs(c['r'] - r) <= tol:
                out.append((0 if c['kind'] == 'circle' else 1, vi, c))
    out.sort(key=lambda t: (t[0], t[1]))
    return [(vi, c) for _k, vi, c in out]


def _annotate(key, sheet, views, md, s, log):
    """What the checks decided for this part, put on its sheet."""
    dims, syms = sheet.drawingDimensions, sheet.drawingSymbols
    made = dict(dims=0, datums=0, frames=0)
    if not s:
        log.append('no scale: no arc or circle on the sheet to take it from')
        return made

    def diameter(vi, c, what):
        v = views[vi]
        u = _away(v, c['c'])
        try:
            d = dims.addDiameterDimension(c['obj'], _dpt(c['c'][0] + u[0] * (c['r'] + 12.0), c['c'][1] + u[1] * (c['r'] + 12.0)))
            if d:
                made['dims'] += 1
                log.append('%s: diameter %.2f mm on the sheet in view %d' % (what, 2 * c['r'], vi))
                return d
        except Exception as e:
            log.append('%s: no diameter (%s)' % (what, str(e).strip()[:120]))
        return None

    def linear(vi, c1, c2, what):
        v = views[vi]
        m = ((c1['c'][0] + c2['c'][0]) / 2, (c1['c'][1] + c2['c'][1]) / 2)
        dx, dy = c2['c'][0] - c1['c'][0], c2['c'][1] - c1['c'][1]
        L = math.hypot(dx, dy)
        n = (-dy / L, dx / L)
        if (m[0] - v['centre'][0]) * n[0] + (m[1] - v['centre'][1]) * n[1] < 0:
            n = (-n[0], -n[1])
        off = c1['r'] + 10.0
        try:
            d = dims.addLinearDimension(c1['obj'].centerPoint, c2['obj'].centerPoint, _dpt(m[0] + n[0] * off, m[1] + n[1] * off), True)
            if d:
                made['dims'] += 1
                log.append('%s: %.2f mm between centres on the sheet' % (what, L))
                return d
        except Exception as e:
            log.append('%s: no linear dimension (%s)' % (what, str(e).strip()[:120]))
        return None

    def datum(letter, what):
        samples = md['datum_faces'].get(letter)
        if not samples:
            log.append('datum %s: %s not found in the model' % (letter, what))
            return None
        hit = _face_line(views, samples)
        if not hit:
            log.append('datum %s: %s not found edge on in any view' % (letter, what))
            return None
        _cnt, vi, c, L = hit
        v = views[vi]
        mid = ((c['p0'][0] + c['p1'][0]) / 2, (c['p0'][1] + c['p1'][1]) / 2)
        n = (-(c['p1'][1] - c['p0'][1]) / L, (c['p1'][0] - c['p0'][0]) / L)
        if (mid[0] - v['centre'][0]) * n[0] + (mid[1] - v['centre'][1]) * n[1] < 0:
            n = (-n[0], -n[1])
        end = (mid[0] + n[0] * 12.0, mid[1] + n[1] * 12.0)
        try:
            di = syms.createDatumIdentifierInput()
            di.identifier = letter
            di.viewCurve = c['obj']
            di.leaderPoints = [_dpt(*mid), _dpt(*end)]
            di.placementPoint = _dpt(*end)
            sym = syms.addDatumIdentifier(di)
            if sym:
                made['datums'] += 1
                log.append('datum %s: %s, on a %.1f mm line of view %d' % (letter, md['datum_notes'].get(letter, what), L, vi))
                return sym
        except Exception as e:
            log.append('datum %s: not added (%s)' % (letter, str(e).strip()[:120]))
        return None

    def position(vi, c, t, letter):
        v = views[vi]
        u = _away(v, c['c'])
        at = (c['c'][0] + u[0] * c['r'], c['c'][1] + u[1] * c['r'])
        end = (c['c'][0] + u[0] * (c['r'] + 18.0), c['c'][1] + u[1] * (c['r'] + 18.0))
        try:
            fi = syms.createFeatureControlFrameInput()
            fi.viewCurve = c['obj']
            fi.leaderPoints = [_dpt(*at), _dpt(*end)]
            fi.placementPoint = _dpt(*end)
            ff = fi.firstFrame
            ff.geometricCharacteristic = adsk.drawing.GeometricCharacteristicTypes.PositionGeometricCharacteristicType
            tol_ = ff.primaryTolerance
            tol_.clear()
            tol_.append(adsk.drawing.SymbolPaletteTypes.DiameterSymbolPaletteType)
            tol_.appendText('%.1f' % t)
            tol_.append(adsk.drawing.SymbolPaletteTypes.MaximumMaterialRequirementSymbolPaletteType)
            ff.primaryTolerance = tol_
            dat = ff.primaryDatum
            dat.clear()
            dat.appendText(letter)
            ff.primaryDatum = dat
            sym = syms.addFeatureControlFrame(fi)
            if sym:
                made['frames'] += 1
                log.append('position: Ø%.1f (M) to %s on the hole in view %d' % (t, letter, vi))
                return sym
        except Exception as e:
            log.append('position: not added (%s)' % str(e).strip()[:120])
        return None

    def bore_of(cyl):
        on_axis = [c for c in cyl if c['internal'] and abs(abs(c['a'][2]) - 1.0) < 1e-6
                   and math.hypot(c['o'][0], c['o'][1]) < 0.01]
        pool = on_axis or [c for c in cyl if c['internal']]
        if not pool:
            return None
        pool.sort(key=lambda c: (c['fit'] is None, -(c['area'] / (2 * math.pi * c['r'])), c['r']))
        return pool[0]

    if key == 'bracket':
        b = DD.bracket()
        hits = _circles(views, b['H'] / 2.0 * s)
        pair = None
        for vi in sorted({h[0] for h in hits}):
            cs = [c for v_, c in hits if v_ == vi]
            for i in range(len(cs)):
                for j in range(i + 1, len(cs)):
                    dd = math.hypot(cs[i]['c'][0] - cs[j]['c'][0], cs[i]['c'][1] - cs[j]['c'][1])
                    if pair is None and abs(dd - b['pitch'] * s) <= 0.1:
                        pair = (vi, cs[i], cs[j])
        if pair:
            diameter(pair[0], pair[1], 'the bolt holes')
            linear(pair[0], pair[1], pair[2], "the holes' centres")
        elif hits:
            diameter(hits[0][0], hits[0][1], 'the bolt holes')
            log.append("the holes' centres: the two holes not found %.2f apart in one view" % (b['pitch'] * s))
        else:
            log.append('the bolt holes: no curve of radius %.3f mm on the sheet' % (b['H'] / 2.0 * s))
        datum('A', "the foot's bolting face")
        tgt = (pair[0], pair[2]) if pair else (hits[0] if hits else None)
        if tgt:
            position(tgt[0], tgt[1], b['t'], 'A')
    elif key == 'segment':
        datum('A', 'the top face')
        datum('B', 'the 15 deg face')
    elif key in ('spacer', 'disc'):
        bo = bore_of(md['cylinders'])
        if bo is None:
            log.append('the bore: none in the model')
        else:
            hits = _circles(views, bo['r'] * s)
            if hits:
                diameter(hits[0][0], hits[0][1], 'the bore (%.3f mm)' % (2 * bo['r']))
            else:
                log.append('the bore: no curve of radius %.3f mm on the sheet' % (bo['r'] * s))
    elif key == 'fitting':
        f = DD.fitting()
        rb = f['D'] / 2.0
        hits = _circles(views, rb * s)
        if hits:
            diameter(hits[0][0], hits[0][1], 'the bores (%g mm)' % f['D'])
        else:
            log.append('the bores: no curve of radius %.3f mm on the sheet' % (rb * s))
        groups = {}
        for c in md['cylinders']:
            if c['internal'] and abs(c['r'] - rb) > 0.5 and 2.0 <= c['r'] <= 10.0:
                groups.setdefault(round(c['r'], 3), []).append(c)
        if groups:
            rh = max(groups, key=lambda r: (len(groups[r]), -r))
            hits = _circles(views, rh * s)
            if hits:
                diameter(hits[0][0], hits[0][1], 'the bolt holes (%d of %g mm)' % (len(groups[rh]), 2 * rh))
            else:
                log.append('the bolt holes: no curve of radius %.3f mm on the sheet' % (rh * s))
        else:
            log.append('the bolt holes: none in the model')
    return made


def _table(sheet, rows, heads=TABLE_HEADS):
    """A table of requirements on the sheet: the headings, then one row each with its source; read back."""
    ti = sheet.customTables.createInput()
    ti.columnCount = 3
    ti.rowCount = len(rows) + 1
    t = sheet.customTables.add(ti)
    if t is None:
        raise RuntimeError('Fusion made no table')
    data = [list(heads)] + [list(r) for r in rows]
    for i, row in enumerate(data):
        for j, cell in enumerate(row):
            t.setCellData(i, j, str(cell))
    return [[t.getCellData(i, j) for j in range(3)] for i in range(len(data))]


def _sheet_info(sheet, views):
    w, h = sheet.width, sheet.height
    if w < 100.0:                    # read in cm
        w, h = w * 10.0, h * 10.0
    return dict(name=sheet.name, size=[round(w, 2), round(h, 2)],
                views=[dict(type=v['type'], curves=len(v['curves']),
                            axes=([list(v['map']['a']), list(v['map']['b'])] if v.get('map') else None),
                            fit=(round(v['map']['score'], 3) if v.get('map') else None)) for v in views])


def _counts(sheets):
    return (sum(s.drawingDimensions.count for s in sheets), sum(s.drawingSymbols.count for s in sheets))


def _draw(spec, info, rec):
    """The automatic drawing of the saved design, what was decided added to it, its PDF."""
    app = adsk.core.Application.get()
    ui = app.userInterface
    assembly = bool(spec.get('assembly'))
    log = rec.setdefault('log', [])
    df = info['doc'].dataFile
    ddf, asked = _create_drawing(df, assembly)
    rec['asked'] = asked
    ddoc = _open_doc(ddf)
    ddoc.activate()
    info['drawing_doc'] = ddoc
    drawing = adsk.drawing.DrawingDocument.cast(ddoc).drawing
    t0 = time.time()
    while time.time() - t0 < 60.0 and (drawing.sheets.count == 0 or drawing.sheets.item(0).views.count == 0):
        _wait(1.0)
    rec['settle'] = _settle(drawing, ui)
    try:
        rec['drawing'] = '%s.%s' % (ddf.name, ddf.fileExtension)
    except Exception:
        rec['drawing'] = None
    try:
        rec['projection'] = _enum_names(adsk.drawing.ProjectionAngleTypes, 'ProjectionAngleType').get(
            drawing.documentSettings.projectionAngle)
    except Exception:
        rec['projection'] = None
    sheets = [drawing.sheets.item(i) for i in range(drawing.sheets.count)]
    deleted = []
    if not assembly and len(sheets) > 1:          # a part: its one sheet, the one with the most views
        keep = max(range(len(sheets)), key=lambda i: (sheets[i].views.count, -i))
        for i in reversed(range(len(sheets))):
            if i != keep:
                nm = sheets[i].name
                try:
                    if sheets[i].deleteMe():
                        deleted.append(nm)
                except Exception:
                    pass
    elif assembly and len(sheets) > 2:            # the assembly's two sheets; any sheet of a part dropped
        for i in reversed(range(2, len(sheets))):
            nm = sheets[i].name
            try:
                if sheets[i].deleteMe():
                    deleted.append(nm)
            except Exception:
                pass
    rec['sheets_deleted'] = deleted
    sheets = [drawing.sheets.item(i) for i in range(drawing.sheets.count)]
    sheet = sheets[0] if assembly else max(sheets, key=lambda sh: sh.views.count)
    on = sheets if assembly else [sheet]
    d0, s0 = _counts(on)
    views = _views(sheet)
    if not assembly:
        md = info['md']
        rs = [c['r'] for v in views for c in v['curves'] if c['kind'] in ('circle', 'arc') and c['r']]
        s = max(rs) / md['rmax'] if rs and md['rmax'] else None
        rec['scale'] = s
        for v in views:
            v['map'] = _register(v, md['keys'], s)
        rec['added'] = _annotate(spec['key'], sheet, views, md, s, log)
        try:
            rec['table'] = _table(sheet, info['rows'])
        except Exception as e:
            log.append('table: %s' % str(e).strip()[:160])
    else:
        try:
            rec['table'] = _table(sheets[0], info['rows'], ('Item', 'What the fitter does', 'Source'))
        except Exception as e:
            log.append('table: %s' % str(e).strip()[:160])
    try:
        ui.terminateActiveCommand()
    except Exception:
        pass
    if spec['key'] in TIDY:
        try:
            rec['tidy'] = bool(sheet.tidyUp)       # a property: reading it runs Fusion's Tidy Up
        except Exception as e:
            rec['tidy'] = False
            log.append('tidy up: %s' % str(e).strip()[:120])
    _wait(1.0)
    sheets = [drawing.sheets.item(i) for i in range(drawing.sheets.count)]
    on = sheets if assembly else [sheet]
    d1, s1 = _counts(on)
    rec['read'] = dict(dims=d1 - d0, symbols=s1 - s0, dims_on_sheet=d1, symbols_on_sheet=s1,
                       dims_before=d0, symbols_before=s0)
    rec['sheets'] = len(sheets)
    rec['sheet'] = _sheet_info(sheets[0] if assembly else sheet, _views(sheets[0] if assembly else sheet))
    if assembly:
        rec['all_sheets'] = [_sheet_info(sh, _views(sh)) for sh in sheets]
    # the PDF
    os.makedirs(PDF_DIR, exist_ok=True)
    path = os.path.join(PDF_DIR, spec['number'] + '.pdf')
    try:
        if os.path.exists(path):
            os.remove(path)                       # this run's copy replaces the last one
    except Exception:
        pass
    try:
        opt = drawing.exportManager.createPDFExportOptions(path)
        opt.sheetsToExport = adsk.drawing.PDFSheetsExport.AllPDFSheetsExport
        _set(opt, 'openPDF', False)
        _set(opt, 'useLineWeights', True)
        drawing.exportManager.execute(opt)
    except Exception as e:
        log.append('PDF: %s' % str(e).strip()[:160])
    t1 = time.time()
    while not os.path.exists(path) and time.time() - t1 < 30.0:
        _wait(0.5)
    rec['pdf'] = os.path.basename(path) if os.path.exists(path) else None
    try:
        ddoc.save('Chapter 63: what the checks decided, the table')
    except Exception as e:
        log.append('drawing not saved: %s' % str(e).strip()[:120])
    try:
        info['doc'].close(False)                  # the design is saved; the drawing stays open
    except Exception:
        pass


# ---------- the build ----------
def _record(p):
    return dict(key=p['key'], number=p['number'], title=p['title'], machine=p['machine'], part=p['part'],
                saved=None, drawing=None, sheet=dict(size=[0, 0], views=[]), sheets=0, pdf=None,
                read=dict(dims=0, symbols=0), want=WANT.get(p['key'], {}), rows=[], changes={},
                model=dict(radii={}), volume=0.0, items=[], errors=[], log=[])


def build(part='all', folder=None):
    if part not in CHOICES:
        raise ValueError('No part called %s: the drawings are of %s.' % (part, ', '.join(CHOICES)))
    t0 = time.time()
    RSM = _load('RangeSweep')
    SB = _load('StopBracket')
    MF = _load('MaterialsFits')
    folder_obj, folder_name = _folder(folder if folder is not None else DD.FOLDER)
    specs = [p for p in DD.PARTS if part == 'all' or p['key'] == part]
    bracket_spec = [p for p in DD.PARTS if p['key'] == 'bracket'][0]
    recs = {p['key']: _record(p) for p in specs}
    machines = []
    for p in specs:
        if p['machine'] not in machines:
            machines.append(p['machine'])
    infos, notes = {}, {}
    for mk in machines:
        mspecs = [p for p in specs if p['machine'] == mk]
        mdocs = []
        try:
            ex, notes[mk] = _from_machine(mk, mspecs, RSM, SB, MF, bracket_spec, mdocs)
            for p in mspecs:
                rec = recs[p['key']]
                if isinstance(ex[p['key']], str):
                    rec['errors'].append(ex[p['key']])
                    continue
                try:
                    if p.get('assembly'):
                        info = _stop_design(p, ex[p['key']], SB, bracket_spec)
                    else:
                        info = _part_design(p, ex[p['key']], SB, mk)
                    rec['saved'] = _save(info['doc'], folder_obj, '%s %s' % (p['number'], p['title']))
                    for k in ('rows', 'changes', 'volume', 'model', 'frame'):
                        rec[k] = info.get(k)
                    for k in ('material', 'given', 'fits', 'carried', 'missed', 'items'):
                        if k in info:
                            rec[k] = info[k]
                    infos[p['key']] = info
                except Exception:
                    rec['errors'].append(traceback.format_exc())
        except Exception:
            for p in mspecs:
                recs[p['key']]['errors'].append('the %s: %s' % (mk, traceback.format_exc()))
        finally:
            for d in mdocs:                     # the machine, with its own joints, closed
                try:
                    d.close(False)
                except Exception:
                    pass
            adsk.doEvents()
    last_doc = None
    for p in specs:
        if p['key'] not in infos:
            continue
        try:
            _draw(p, infos[p['key']], recs[p['key']])
            last_doc = infos[p['key']].get('drawing_doc') or last_doc
        except Exception:
            recs[p['key']]['errors'].append(traceback.format_exc())
    if last_doc is not None:
        try:
            last_doc.activate()
        except Exception:
            pass
    names = _names(folder_obj, 2 * len(infos))
    parts = [recs[p['key']] for p in specs]
    r = dict(version=VERSION, part=part, folder=folder_name, parts=parts, names=names, machines=notes,
             seconds=round(time.time() - t0, 1))
    _last.clear()
    _last.update(r)
    try:
        os.makedirs(RESULTS, exist_ok=True)
        with open(os.path.join(RESULTS, 'part_drawings.json'), 'w', encoding='utf-8') as fh:
            json.dump(r, fh, indent=1, ensure_ascii=False, default=str)
    except Exception:
        pass
    lines = ['Part drawings (Chapter 63): %d of %d drawings made, in %s.' % (
        sum(1 for p in parts if p['pdf']), len(parts), folder_name)]
    for p in parts:
        w = p['want']
        views = len(p['sheet'].get('views', []))
        lines.append('%s %s: %d sheet(s), %d views on the first; %d dimensions and %d symbols added (asked %d and '
                     '%d); table of %d rows; PDF %s%s' % (
                         p['number'], p['title'], p['sheets'], views, p['read'].get('dims', 0),
                         p['read'].get('symbols', 0), w.get('dims', 0), w.get('datums', 0) + w.get('frames', 0),
                         len(p['rows']), p['pdf'] or 'none', '; FAILED: ' + p['errors'][-1].strip().split('\n')[-1]
                         if p['errors'] else ''))
    lines.append('Folder %s: %d files, %s.' % (folder_name, len(names),
                                               'each once' if len(names) == len(set(names)) else 'some twice'))
    lines.append('Results: design_office/results/part_drawings.json; PDFs in design_office/results/drawings.')
    lines.append('')
    lines.append('[%s]' % VERSION)
    return '\n'.join(lines)


# ---------- the command dialog ----------
_EVENT = 'mdfcPartDrawingsBuild'
_handlers = []
_queue = []


class _Build(adsk.core.CustomEventHandler):
    """The build, outside the command: Fusion does not create, save or close documents inside a command's events."""
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        while _queue:
            part = _queue.pop(0)
            try:
                ui.messageBox(build(part))
            except ValueError as e:
                ui.messageBox(str(e))
            except Exception:
                ui.messageBox('PartDrawings failed:\n%s' % traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            part = args.command.commandInputs.itemById('part').selectedItem.name
            _queue.append(part)
            if not app.fireCustomEvent(_EVENT, part):
                raise RuntimeError('the build could not be queued')
        except Exception:
            ui.messageBox('PartDrawings could not start:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('part', 'The part', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in CHOICES:
            dd.listItems.add(k, k == 'all')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        try:
            app.unregisterCustomEvent(_EVENT)
        except Exception:
            pass
        ev = app.registerCustomEvent(_EVENT)
        hb = _Build()
        ev.add(hb)
        _handlers.append(hb)
        cid = 'mdfcPartDrawings'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Part drawings', 'Chapter 63: the drawings of the five parts and the opening stop')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PartDrawings failed:\n%s' % traceback.format_exc())


def stop(_context):
    try:
        adsk.core.Application.get().unregisterCustomEvent(_EVENT)
    except Exception:
        pass
