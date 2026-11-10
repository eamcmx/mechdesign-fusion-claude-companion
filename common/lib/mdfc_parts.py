# =============================================================================
# mdfc_parts.py  -  the parts library of Volume 2 of "Mechanical Design in Autodesk Fusion with Claude": the machine
# elements of Volume 1 as functions that build a part inside a given component, so that a subsystem script can put
# real parts together. Every size is in millimetres and every position in world coordinates; the functions move
# the body into the component's own frame.
#
#   bodies       cyl, box, sphere, union, cut (TemporaryBRepManager); add_body puts one into a component
#   components   new_occ (a component inside another, at a translation), material, colour
#   shafts       stepped_shaft (sections along an axis), keyseat (cuts the shaft), parallel_key (DIN 6885, form A)
#   bearings     deep-groove bearing as inner ring, outer ring and balls (sizes of Table 11-2, balls as in Ch. 8)
#   gears        spur_teeth (the involute of Chapter 12, joined to a body and patterned)
#   fasteners    cap_screw (ISO 4762 proportions), nut, dowel
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math

import adsk.core
import adsk.fusion

CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString


# ---------- temporary bodies, world millimetres ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cyl(p0, p1, r):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def tube(p0, p1, r_out, r_in):
    b = cyl(p0, p1, r_out)
    d = [p1[i] - p0[i] for i in range(3)]
    L = math.sqrt(sum(v * v for v in d))
    e = [v / L for v in d]
    return cut(b, cyl([p0[i] - e[i] for i in range(3)], [p1[i] + e[i] for i in range(3)], r_in))


def box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2 * CM, (y0 + y1) / 2 * CM, (z0 + z1) / 2 * CM)
    return tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0), (x1 - x0) * CM,
                                                                   (y1 - y0) * CM, (z1 - z0) * CM))


def sphere(c, r):
    return tbm().createSphere(P3(*[v * CM for v in c]), r * CM)


def boolean(a, kind, *bs):
    for b in bs:
        if not tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def union(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.UnionBooleanType, *bs)


def cut(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.DifferenceBooleanType, *bs)


def intersect(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.IntersectionBooleanType, *bs)


# ---------- components ----------
def new_occ(parent, name, t=(0.0, 0.0, 0.0)):
    """A new component inside `parent` (a component), its origin at the translation t (mm). Inside a subassembly
    only the first component made is grounded to its parent; the others would stay behind when the subassembly
    turns on its joint, so each is grounded to its parent here."""
    mat = adsk.core.Matrix3D.create()
    mat.translation = V3(t[0] * CM, t[1] * CM, t[2] * CM)
    occ = parent.occurrences.addNewComponent(mat)
    occ.component.name = name
    if parent != parent.parentDesign.rootComponent:
        occ.isGroundToParent = True
    return occ


def world_matrix(occ):
    """The occurrence's transform to the world, through its parents."""
    m = occ.transform2.copy()
    p = occ.assemblyContext
    while p:
        m.transformBy(p.transform2)
        p = p.assemblyContext
    return m


def add_body(occ, tb, name, material=None):
    """A temporary body given in world coordinates into the occurrence's component."""
    m = occ.transform2.copy()
    m.invert()
    tbm().transform(tb, m)
    comp = occ.component
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    if material:
        set_material(body, material)
    return body


def set_material(body, name):
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    mat = lib.materials.itemByName(name)
    if mat:
        body.material = mat


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- shafts along y, their axis through (x, z) ----------
def stepped_shaft(sections, x=0.0, z=0.0):
    """A shaft of cylinders: sections = [(y0, y1, d), ...] in order along y."""
    b = None
    for y0, y1, d in sections:
        c = cyl((x, y0, z), (x, y1, z), d / 2.0)
        b = c if b is None else union(b, c)
    return b


def keyseat(shaft, d, y0, y1, kb, t1, x=0.0, z=0.0):
    """Cuts a form-A keyseat on top (+z) of a shaft of diameter d, from y0 to y1: kb wide, t1 deep, round ends."""
    r = kb / 2.0
    zt = z + d / 2.0 - t1
    slot = box(x - r, x + r, y0 + r, y1 - r, zt, z + d / 2.0 + 1.0)
    union(slot, cyl((x, y0 + r, zt), (x, y0 + r, z + d / 2.0 + 1.0), r), cyl((x, y1 - r, zt), (x, y1 - r, z + d / 2.0 + 1.0), r))
    return cut(shaft, slot)


def parallel_key(d, y0, y1, kb, h, t1, x=0.0, z=0.0):
    """A DIN 6885 form-A key, kb x h, from y0 to y1, sitting t1 deep in the keyseat of a shaft of diameter d."""
    r = kb / 2.0
    zb = z + d / 2.0 - t1
    k = box(x - r, x + r, y0 + r, y1 - r, zb, zb + h)
    return union(k, cyl((x, y0 + r, zb), (x, y0 + r, zb + h), r), cyl((x, y1 - r, zb), (x, y1 - r, zb + h), r))


# ---------- deep-groove ball bearings on a y axis ----------
def bearing_geometry(d, D):
    """Ball size, pitch diameter and count as in Volume 1, Chapter 8, and the rings' free diameters."""
    dm = (d + D) / 2.0
    Dw = 0.3 * (D - d)
    Z = int(math.floor(math.pi * dm / (1.25 * Dw)))
    return dict(dm=dm, Dw=Dw, Z=Z, in_od=dm - Dw - 0.1, out_id=dm + Dw + 0.1)


def bearing_bodies(d, D, B, y0, x=0.0, z=0.0):
    """Inner ring, outer ring and the ball set of a bearing whose faces are at y0 and y0 + B."""
    g = bearing_geometry(d, D)
    inner = tube((x, y0, z), (x, y0 + B, z), g['in_od'] / 2, d / 2)
    outer = tube((x, y0, z), (x, y0 + B, z), D / 2, g['out_id'] / 2)
    balls = None
    yc = y0 + B / 2.0
    for k in range(g['Z']):
        a = 2 * math.pi * k / g['Z']
        s = sphere((x + g['dm'] / 2 * math.cos(a), yc, z + g['dm'] / 2 * math.sin(a)), g['Dw'] / 2)
        balls = s if balls is None else union(balls, s)
    return inner, outer, balls, g


# ---------- spur gear teeth (Volume 1, Chapter 12) ----------
def inv(a):
    return math.tan(a) - a


def _rotate(pts, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(px * c - py * s, px * s + py * c) for px, py in pts]


def tooth_outline(m, z, alpha, backlash, phase, n=8):
    """One tooth centred on `phase`, closed just below the root circle (as GearPairOnPlate.py)."""
    r = m * z / 2.0
    rb = r * math.cos(alpha)
    ra = r + m
    rf = r - 1.25 * m
    r_lo = max(rb, rf)
    t_at = lambda rr: 0.0 if rr <= rb else math.sqrt((rr / rb) ** 2 - 1.0)
    half = (math.pi * m / 2.0 - backlash / 2.0) / (2.0 * r)
    rot = half + inv(alpha)
    right, left = [], []
    for i in range(n + 1):
        t = t_at(r_lo) + (t_at(ra) - t_at(r_lo)) * i / n
        px, py = rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t))
        right.append(_rotate([(px, -py)], rot)[0])
        left.append(_rotate([(px, py)], -rot)[0])
    a_l, a_r = math.atan2(left[0][1], left[0][0]), math.atan2(right[0][1], right[0][0])
    r_in = rf * 0.985
    pts = [(r_in * math.cos(a_r), r_in * math.sin(a_r)), (r_in * math.cos(a_l), r_in * math.sin(a_l))]
    if r_lo > r_in + 1e-9:
        pts.append((r_lo * math.cos(a_l), r_lo * math.sin(a_l)))
    pts.extend(left[1:] if r_lo <= r_in + 1e-9 else left)
    a0, a1 = math.atan2(left[-1][1], left[-1][0]), math.atan2(right[-1][1], right[-1][0])
    pts.extend((ra * math.cos(a0 + (a1 - a0) * j / 5), ra * math.sin(a0 + (a1 - a0) * j / 5)) for j in range(1, 5))
    pts.extend(reversed(right))
    clean = [pts[0]]
    for q in pts[1:]:
        if math.hypot(q[0] - clean[-1][0], q[1] - clean[-1][1]) > 1e-7:
            clean.append(q)
    return _rotate(clean, phase), rf


def xz_plane(comp, y):
    """A plane square to y at y (mm, in the component's own frame)."""
    if abs(y) < 1e-9:
        return comp.xZConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xZConstructionPlane, VI(y * CM))
    return comp.constructionPlanes.add(ci)


def spur_teeth(occ, body, m, z, alpha_deg, backlash, phase, y0, y1, n=8):
    """The teeth of a spur gear whose axis is the component's own y axis, joined to `body` from y0 to y1 (mm, in
    the component's frame): one tooth, then a circular pattern. The angle `phase` is measured in the xz plane from
    +x towards +z."""
    comp = occ.component
    pts, rf = tooth_outline(m, z, math.radians(alpha_deg), backlash, phase, n)
    plane = xz_plane(comp, y0)
    sk = comp.sketches.add(plane)
    lines = sk.sketchCurves.sketchLines
    model = [sk.modelToSketchSpace(P3(px * CM, y0 * CM, pz * CM)) for px, pz in pts]
    first = lines.addByTwoPoints(model[0], model[1])
    prev = first.endSketchPoint
    for q in model[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    normal = sk.xDirection.crossProduct(sk.yDirection)              # an extrusion goes along the sketch's normal
    inp.setDistanceExtent(False, VI((y1 - y0) * CM * (1 if normal.y > 0 else -1)))
    inp.participantBodies = [body]
    tooth = ext.add(inp)
    feats = adsk.core.ObjectCollection.create()
    feats.add(tooth)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.yConstructionAxis)
    pi.quantity = VI(z)
    pi.totalAngle = VS('360 deg')
    pats.add(pi)
    return rf


# ---------- fasteners ----------
def cap_screw(d, L, p, axis):
    """An ISO 4762 socket head cap screw, head d_k = 1.5 d and k = d, the head's underside at p, the shank of
    length L along the unit vector `axis` (into the part)."""
    ax = axis
    head = cyl(p, [p[i] - ax[i] * d for i in range(3)], 0.75 * d)
    cut(head, cyl([p[i] - ax[i] * d * 0.5 for i in range(3)], [p[i] - ax[i] * (d + 1) for i in range(3)], 0.43 * d))
    return union(head, cyl([p[i] - ax[i] * 0.001 for i in range(3)], [p[i] + ax[i] * L for i in range(3)], d / 2.0))


def nut(d, p, axis, m=None):
    """A nut, drawn round (across the flats 1.6 d), its face at p, m = 0.8 d thick along `axis`."""
    m = m or 0.8 * d
    return tube(p, [p[i] + axis[i] * m for i in range(3)], 0.8 * d, d / 2.0 + 0.001)


def dowel(d, p0, p1):
    return cyl(p0, p1, d / 2.0)


# ---------- tapered roller bearings on any axis (Volume 2, Chapter 29) ----------
def _along(p, e, t):
    return [p[i] + e[i] * t for i in range(3)]


def _perp(e):
    """A unit vector square to e."""
    a = (1.0, 0.0, 0.0) if abs(e[0]) < 0.9 else (0.0, 1.0, 0.0)
    v = (e[1] * a[2] - e[2] * a[1], e[2] * a[0] - e[0] * a[2], e[0] * a[1] - e[1] * a[0])
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def taper_geometry(d, D, T, B, C, alpha_deg=15.0):
    """Proportions for drawing a single row tapered roller bearing from its catalogue sizes: the cup's raceway at
    alpha to the axis, the cone's at alpha less the rollers' own taper. t is measured from the cone's back face (its
    large end) towards the small end; radii from the axis."""
    Dw = 0.24 * (D - d)                        # the rollers' mean diameter
    rm = (d + D) / 4.0                         # their pitch radius, at mid-width
    Lw = 0.62 * C                              # their length
    tm = T - C + C / 2.0                       # their middle, under the middle of the cup
    a_o = math.radians(alpha_deg)
    a_i = math.radians(alpha_deg - 4.0)
    a_m = (a_o + a_i) / 2.0
    g = dict(Dw=Dw, rm=rm, Lw=Lw, tm=tm, a_o=a_o, a_i=a_i, a_m=a_m)
    # the cup's raceway through (tm, rm + Dw/2), radius falling towards the small end at a_o
    g['cup_r'] = lambda t: rm + Dw / 2.0 + (tm - t) * math.tan(a_o)
    g['cone_r'] = lambda t: rm - Dw / 2.0 + (tm - t) * math.tan(a_i)
    g['n_rollers'] = int(math.floor(2 * math.pi * rm / (1.18 * Dw)))
    return g


def taper_bodies(d, D, T, B, C, p0, axis, alpha_deg=15.0):
    """The cone with its rollers (one body, as a cone assembly is bought) and the cup of a tapered roller bearing.
    p0: the centre of the cone's back face (its large end); axis: the unit vector from there towards the small end."""
    e = axis
    g = taper_geometry(d, D, T, B, C, alpha_deg)
    # the cone: a frustum on its raceway, a rib at the large end, the bore
    cone = tbm().createCylinderOrCone(P3(*[v * CM for v in _along(p0, e, 0.0)]), g['cone_r'](0.0) * CM,
                                      P3(*[v * CM for v in _along(p0, e, B)]), g['cone_r'](B) * CM)
    rib = cyl(_along(p0, e, 0.0), _along(p0, e, 0.12 * B), g['cone_r'](0.0) + 0.45 * g['Dw'])
    union(cone, rib)
    cut(cone, cyl(_along(p0, e, -1.0), _along(p0, e, B + 1.0), d / 2.0))
    # the rollers, between the raceways, their axes inclined at the mean of the two raceway angles
    u = _perp(e)
    w = (e[1] * u[2] - e[2] * u[1], e[2] * u[0] - e[0] * u[2], e[0] * u[1] - e[1] * u[0])
    t0, t1 = g['tm'] - g['Lw'] / 2.0, g['tm'] + g['Lw'] / 2.0
    for k in range(g['n_rollers']):
        ang = 2 * math.pi * k / g['n_rollers']
        rad = [math.cos(ang) * u[i] + math.sin(ang) * w[i] for i in range(3)]
        def at(t, r):
            return [p0[i] + e[i] * t + rad[i] * r for i in range(3)]
        rc0 = (g['cup_r'](t0) + g['cone_r'](t0)) / 2.0
        rc1 = (g['cup_r'](t1) + g['cone_r'](t1)) / 2.0
        r0 = (g['cup_r'](t0) - g['cone_r'](t0)) / 2.0 * 0.92      # clear of both raceways at the tilted ends
        r1 = (g['cup_r'](t1) - g['cone_r'](t1)) / 2.0 * 0.92
        roller = tbm().createCylinderOrCone(P3(*[v * CM for v in at(t0, rc0)]), r0 * CM,
                                            P3(*[v * CM for v in at(t1, rc1)]), r1 * CM)
        union(cone, roller)
    # the cup: its outside cylinder less a frustum on its raceway
    ta, tb = T - C, T
    cup = cyl(_along(p0, e, ta), _along(p0, e, tb), D / 2.0)
    race = tbm().createCylinderOrCone(P3(*[v * CM for v in _along(p0, e, ta - 1.0)]), g['cup_r'](ta - 1.0) * CM,
                                      P3(*[v * CM for v in _along(p0, e, tb + 1.0)]), g['cup_r'](tb + 1.0) * CM)
    cut(cup, race)
    return cone, cup, g


# ---------- straight-sided splines on any axis ----------
def spline_shaft(p0, p1, d_minor, d_major, n, w):
    """A shaft with n straight-sided splines, w wide, from its minor to its major diameter, between p0 and p1."""
    e = [p1[i] - p0[i] for i in range(3)]
    L = math.sqrt(sum(x * x for x in e))
    e = [x / L for x in e]
    s = cyl(p0, p1, d_minor / 2.0 + 0.01)
    for k in range(n):
        s = union(s, _spline_tooth(p0, e, L, k, n, d_minor, d_major, w))
    return s


def spline_hole(p0, p1, d_minor, d_major, n, w, gap=0.1):
    """The hole a splined hub needs: the minor diameter and n slots, `gap` larger all round."""
    e = [p1[i] - p0[i] for i in range(3)]
    L = math.sqrt(sum(x * x for x in e))
    e = [x / L for x in e]
    h = cyl(p0, p1, d_minor / 2.0 + gap)
    for k in range(n):
        h = union(h, _spline_tooth(p0, e, L, k, n, d_minor, d_major + 2 * gap, w + 2 * gap))
    return h


def _spline_tooth(p0, e, L, k, n, d_minor, d_major, w):
    u = _perp(e)
    v = (e[1] * u[2] - e[2] * u[1], e[2] * u[0] - e[0] * u[2], e[0] * u[1] - e[1] * u[0])
    ang = 2 * math.pi * k / n
    r = [math.cos(ang) * u[i] + math.sin(ang) * v[i] for i in range(3)]          # the tooth's radial direction
    s = [-math.sin(ang) * u[i] + math.cos(ang) * v[i] for i in range(3)]         # across the tooth
    rc = (d_minor / 2.0 * 0.9 + d_major / 2.0) / 2.0
    hgt = d_major / 2.0 - d_minor / 2.0 * 0.9
    c = [p0[i] + e[i] * L / 2.0 + r[i] * rc for i in range(3)]
    return tbm().createBox(adsk.core.OrientedBoundingBox3D.create(P3(*[x * CM for x in c]), V3(*e), V3(*r),
                                                                   L * CM, hgt * CM, w * CM))
