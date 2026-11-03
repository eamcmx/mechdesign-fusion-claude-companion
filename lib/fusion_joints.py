# -*- coding: utf-8 -*-
"""fusion_joints.py: the joints of an assembly found on the model in Fusion (Chapter 51), for the census
(checks/joint_census.py) and for MaterialsFits.py. Runs inside Fusion only.

A seat is a cylindrical face of a bought part (a bearing's bore or outside, a dowel's or a key's round end) and a face
of a made part on the same axis, of the same diameter, facing the other way and touching; one within NEAR of the
diameter that does not touch is a seat with a gap. A fit is an outside cylinder of one made part in a bore of the same
diameter of another, on one axis, touching or within NEAR (the gap recorded). A key's seats are the made parts its body touches. Lengths in cm, as the
API gives them; the records in mm.
"""
import importlib
import importlib.util
import os

import adsk.core
import adsk.fusion

SEATED = ('bearings', 'pins and keys')

TOL = 1e-4          # cm: 0.001 mm
NEAR = 0.02         # cm: 0.2 mm


def cyl_faces(body):
    """The body's cylindrical faces: (face, radius, axis point, axis direction, outside?)."""
    out = []
    for f in body.faces:
        g = f.geometry
        if not isinstance(g, adsk.core.Cylinder):
            continue
        p = f.pointOnFace
        ok, n = f.evaluator.getNormalAtPoint(p)
        o, a = g.origin, g.axis
        v = adsk.core.Vector3D.create(p.x - o.x, p.y - o.y, p.z - o.z)
        t = v.dotProduct(a) / a.dotProduct(a)
        radial = adsk.core.Vector3D.create(v.x - t * a.x, v.y - t * a.y, v.z - t * a.z)
        out.append((f, g.radius, o, a, n.dotProduct(radial) > 0))
    return out


def coaxial(c1, c2):
    a1, a2 = c1[3].copy(), c2[3].copy()
    a1.normalize()
    a2.normalize()
    if abs(abs(a1.dotProduct(a2)) - 1.0) > 1e-6:
        return False
    w = adsk.core.Vector3D.create(c2[2].x - c1[2].x, c2[2].y - c1[2].y, c2[2].z - c1[2].z)
    return w.crossProduct(a1).length < TOL


def boxes_meet(b1, b2, pad=0.01):
    return (b1.minPoint.x - pad <= b2.maxPoint.x and b2.minPoint.x - pad <= b1.maxPoint.x and
            b1.minPoint.y - pad <= b2.maxPoint.y and b2.minPoint.y - pad <= b1.maxPoint.y and
            b1.minPoint.z - pad <= b2.maxPoint.z and b2.minPoint.z - pad <= b1.maxPoint.z)


def seats_of(bought, made, mm_):
    """[(bought part, its face, made part, the made face, touching?, gap cm)] for parts given as dicts with 'cyl' and
    'box' (see parts())."""
    out = []
    for b in bought:
        for m in made:
            if not boxes_meet(b['box'], m['box'], pad=0.05):
                continue
            for c1 in b['cyl']:
                for c2 in [x for x in m['cyl'] if x[4] != c1[4] and abs(x[1] - c1[1]) < NEAR]:
                    if not coaxial(c1, c2):
                        continue
                    gap = mm_.measureMinimumDistance(c1[0], c2[0]).value
                    if gap < NEAR:
                        out.append((b, c1, m, c2, gap < TOL, gap))
    return out


def fits(made, mm_):
    """[(shaft part, its face, bore part, its face, gap cm)] between made parts: touching, or drawn clear by up to NEAR
    on the radius (a bore larger than the shaft, the gap recorded)."""
    out = []
    for m1 in made:
        outs = [x for x in m1['cyl'] if x[4]]
        if not outs:
            continue
        for m2 in made:
            if m2 is m1 or not boxes_meet(m1['box'], m2['box'], pad=0.05):
                continue
            for c1 in outs:
                for c2 in [x for x in m2['cyl'] if not x[4] and -TOL < x[1] - c1[1] < NEAR]:
                    if not coaxial(c1, c2):
                        continue
                    gap = mm_.measureMinimumDistance(c1[0], c2[0]).value
                    if gap < NEAR:
                        out.append((m1, c1, m2, c2, gap))
    return out


def build(key, scripts):
    """Build one of the nine with its own script, the bag gripper as Chapter 50 detailed it (StandardParts.py)."""
    import detail_inventory as D
    name = 'StandardParts' if key == 'bag' else D.SCRIPTS[key]
    spec = importlib.util.spec_from_file_location('mdfc_joints_' + name, os.path.join(scripts, name, name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.build()


def parts(root, key):
    """The assembly's components with a body, each read as Chapter 49 reads it: kind, number, status, designation,
    group, its occurrence's path, its top-level group, bodies, material, box and cylinders. The bag gripper's numbers
    include the kinds Chapter 50's detail adds."""
    import detail_inventory as D
    nmap = D.number_map(key)
    if key == 'bag':
        import standard_parts as S
        nmap.update(S.detail_numbers())
    if key == 'wrist':
        import wrist_detail as WDET                 # the names Chapter 52's detail gives; absent from Chapter 40's model
        nmap.update(WDET.detail_numbers(nmap))
    inv = D.inventory(key)
    group_of = {dsg: g for g, dd in inv['groups'].items() for dsg in dd}
    out = []
    for o in root.allOccurrences:
        c = o.component
        if c.bRepBodies.count == 0:
            continue
        kind = D.kind_of(c.name, nmap)
        if kind is None:
            continue
        num, desc = nmap[kind]
        status = 'part of' if num is None else ('bought' if num.startswith('S-') else 'made')
        bodies = [b for b in o.bRepBodies]
        group = group_of.get(desc) or ('pins and keys' if desc.startswith(('DIN 6885', 'Key')) else None)
        out.append(dict(o=o, occ=o.fullPathName, name=c.name, kind=kind, status=status, designation=desc, group=group,
                        top=o.fullPathName.split('+')[0].split(':')[0], bodies=bodies,
                        material=bodies[0].material.name if bodies[0].material else None,
                        box=bodies[0].boundingBox, cyl=[x for b in bodies for x in cyl_faces(b)]))
    return out


def key_seats(keys, made, mm_):
    """[(key part, made part, the seat's faces)] for each made part a key's body touches, its shaft and its hub: the
    seat's faces are the made part's planes and its cylinders of the key's end radius that touch the key (the shaft's
    own seat touches the key only along the keyseat's rim, and is left out by its radius)."""
    out = []
    for k in keys:
        ends = {round(x[1], 4) for x in k['cyl']}
        for m in made:
            if not boxes_meet(k['box'], m['box']) or \
                    min(mm_.measureMinimumDistance(kb, mb).value for kb in k['bodies'] for mb in m['bodies']) >= TOL:
                continue
            faces = []
            for mb in m['bodies']:
                for f in mb.faces:
                    g = f.geometry
                    if not (isinstance(g, adsk.core.Plane) or
                            (isinstance(g, adsk.core.Cylinder) and round(g.radius, 4) in ends)):
                        continue
                    if min(mm_.measureMinimumDistance(f, kb).value for kb in k['bodies']) < TOL:
                        faces.append(f)
            out.append((k, m, faces))
    return out


def records(ps, mm_):
    """The census's records of one built assembly: (seats, fits), each record a dict in mm with the faces it was read
    from under '_face' or '_faces' (MaterialsFits colours them; the census drops them)."""
    made = [p for p in ps if p['status'] == 'made']
    bought = [p for p in ps if p['status'] == 'bought' and p['group'] in SEATED]
    keys = [p for p in bought if p['designation'].startswith(('DIN 6885', 'Key', 'Nut key'))]
    seats = []
    for b, c1, m, c2, touching, gap in seats_of([p for p in bought if p not in keys], made, mm_):
        seats.append(dict(bought=b['kind'], designation=b['designation'], group=b['group'], piece=b['name'], occ=b['occ'],
                          face='outside' if c1[4] else 'bore', made=m['kind'], made_name=m['name'], made_occ=m['occ'],
                          top=b['top'], made_top=m['top'], d=round(2 * c1[1] * 10, 3), gap=round(gap * 10, 4),
                          _face=c2[0]))
    for k, m, faces in key_seats(keys, made, mm_):
        seats.append(dict(bought=k['kind'], designation=k['designation'], group=k['group'], piece=k['name'], occ=k['occ'],
                          face='sides', made=m['kind'], made_name=m['name'], made_occ=m['occ'], top=k['top'],
                          made_top=m['top'], d=None, gap=0.0, n_faces=len(faces), _face=None, _faces=faces))
    fits_ = []
    for m1, c1, m2, c2, gap in fits(made, mm_):
        fits_.append(dict(shaft=m1['kind'], shaft_name=m1['name'], shaft_occ=m1['occ'], bore=m2['kind'],
                          bore_name=m2['name'], bore_occ=m2['occ'], shaft_top=m1['top'], bore_top=m2['top'],
                          d=round(2 * c2[1] * 10, 3), gap=round(gap * 10, 4), _faces=(c1[0], c2[0])))
    return seats, fits_


def links(root):
    """[(top group one, top group two, rigid?)] for every joint and as-built joint between two top-level groups;
    None stands for the root, the ground. A rigid group's tops are linked rigidly."""
    top = lambda o: o.fullPathName.split('+')[0].split(':')[0] if o else None
    out = []
    for coll in (root.allJoints, root.allAsBuiltJoints):
        for j in coll:
            t1, t2 = top(j.occurrenceOne), top(j.occurrenceTwo)
            if t1 != t2:
                out.append((t1, t2, j.jointMotion.jointType == adsk.fusion.JointTypes.RigidJointType))
    for g in root.allRigidGroups:
        ts = sorted({top(o) for o in g.occurrences})
        out += [(ts[0], t, True) for t in ts[1:]]
    return out
