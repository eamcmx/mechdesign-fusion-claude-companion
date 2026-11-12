# -*- coding: utf-8 -*-
"""HelicalPair.py: a parametric pair of helical (or herringbone) gears on a base plate, assembled with As-built
Joints and a Motion Link so that the pair turns with Drive Joints.

Written from the printed prompt projects/helical_pair/HelicalPair/PROMPT.txt alone (prompt test).

Run it in Fusion: a dialog asks for the normal module, the two numbers of teeth, the helix angle, the normal
pressure angle, the face width and whether the pair is herringbone. OK opens a new design and builds
Base plate (grounded), Shaft 1, Shaft 2, Gear 1 and Gear 2, one body each. Lengths are typed and reported in mm and
angles in degrees; the API itself works in cm and radians.

Each tooth is its transverse outline (a true involute from the base circle, a radial line below it, the tip on the
tip circle) drawn as one closed polyline, swept along the gear's axis with a twist, then patterned round the axis.
Gear 1 is right-handed (its teeth turn anticlockwise, seen from above, as z rises) and gear 2 left-handed.
"""
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _common_folder():
    """The companion's common/ folder: walk up from this script to the first folder holding common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('No common/companion_paths.py in any folder above %s' % __file__)
        d = up


_COMMON = _common_folder()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402,F401  (puts every lib/ on sys.path; this script needs no library)

VERSION = 'HelicalPair v1.0 (prompt test, 11 Nov 2026)'

CM = 0.1                          # mm to cm
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

BACKLASH = 0.2                    # mm, on the pitch circle, transverse: each tooth is 0.1 mm thinner than pi m_t / 2
N_INV = 12                        # straight segments along each involute flank
N_TIP = 4                         # straight segments along the tip circle

# Sizes the prompt leaves open (my choices, all in mm)
SHAFT_D_PER_MN = 5.0              # shaft diameter = 5 m_n; fits inside the root circle of any gear the limits allow
BORE_CLEARANCE = 0.05             # gear bore = shaft diameter + this (diametral running clearance)
PLATE_GAP = 1.0                   # gears' bottom faces on z = 0 (gear 1 at the origin); plate top at z = -PLATE_GAP
PLATE_T = 6.0                     # plate thickness
PLATE_MARGIN = 10.0               # plate border beyond the tip circles
SHAFT_ABOVE = 3.0                 # shaft end above the gears' top faces
TOL = 1e-4                        # rad, when a measured twist or turn is compared with the wanted one

DEFAULTS = dict(mn=2.0, z1=20, z2=40, beta=20.0, alpha_n=20.0, b=12.0, herringbone=False)

_handlers = []
_last = {}                        # what the last build() made, for anyone who wants to look


# ----------------------------------------------------------------------------------------------------------------
# The design calculation (no Fusion needed)
# ----------------------------------------------------------------------------------------------------------------
def _inv(x):
    return math.tan(x) - x


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def helical(mn, z1, z2, beta, alpha_n, b, herringbone=False):
    """Everything derived from the inputs. mn and b in mm, beta and alpha_n in degrees. Returned lengths in mm,
    angles in radians. The face width b is first adjusted to b', the nearest width that makes gear 1's twist a whole
    number of tenths of a degree (of two tenths for a herringbone), and every value is worked out with b'."""
    be = math.radians(beta)
    an = math.radians(alpha_n)
    mt = mn / math.cos(be)                                      # transverse module
    at = math.atan(math.tan(an) / math.cos(be))                 # transverse pressure angle
    r1, r2 = mt * z1 / 2.0, mt * z2 / 2.0                       # pitch radii
    rb1, rb2 = r1 * math.cos(at), r2 * math.cos(at)             # base radii
    ra1, ra2 = r1 + mn, r2 + mn                                 # tip radii
    rf1, rf2 = r1 - 1.25 * mn, r2 - 1.25 * mn                   # root radii
    a = mt * (z1 + z2) / 2.0                                    # centre distance
    nsw = 2 if herringbone else 1                               # sweeps per tooth
    tenths = int(round(math.degrees(b * math.tan(be) / r1) * 10.0 / nsw)) * nsw
    bw = math.radians(tenths / 10.0) * r1 / math.tan(be)       # b'
    th1 = bw * math.tan(be) / r1                                # turn of a tooth over the face width, pitch circle
    th2 = bw * math.tan(be) / r2
    zmin = 2.0 * math.cos(be) / math.sin(at) ** 2               # fewest teeth without undercut
    eps_a = (math.sqrt(ra1 ** 2 - rb1 ** 2) + math.sqrt(ra2 ** 2 - rb2 ** 2) - a * math.sin(at)) \
        / (math.pi * mt * math.cos(at))                         # length of action over the transverse base pitch
    eps_b = bw * math.sin(be) / (math.pi * mn)                  # overlap ratio
    s_t = math.pi * mt / 2.0 - BACKLASH / 2.0                   # transverse tooth thickness on the pitch circle
    tw1 = tenths // nsw / 10.0                                  # gear 1's twist per sweep, deg, whole tenths
    tw2_exact = math.degrees(th2) / nsw                         # gear 2's twist per sweep, deg
    tw2 = round(tw2_exact * 10.0) / 10.0                        # ... as Fusion keeps it, in tenths
    tip2 = abs(math.radians(tw2 - tw2_exact)) * ra2             # how far that moves gear 2's tips, mm
    return dict(mn=mn, z1=z1, z2=z2, beta=be, alpha_n=an, b_asked=b, herringbone=bool(herringbone), nsw=nsw,
                mt=mt, at=at, r1=r1, r2=r2, rb1=rb1, rb2=rb2, ra1=ra1, ra2=ra2, rf1=rf1, rf2=rf2, a=a,
                b=bw, th1=th1, th2=th2, zmin=zmin, eps_a=eps_a, eps_b=eps_b, s_t=s_t,
                tw1=tw1, tw2=tw2, tw2_exact=tw2_exact, tip2=tip2)


def tip_half_angle(r, rb, ra, s_t):
    """Half the angle a tooth spans at the tip circle (rad); zero or less means a pointed tooth."""
    psi_b = s_t / (2.0 * r) + _inv(math.acos(rb / r))
    return psi_b - _inv(math.acos(rb / ra))


def tooth_outline(r, rb, ra, rf, s_t, mn, phase):
    """The transverse outline of one tooth, centred on the angle `phase` (rad), as a list of (x, y) in mm, closed
    from the last point back to the first. True involute flanks from the base circle to the tip circle; below the
    base circle a radial line down to just under the root circle, so that the tooth overlaps the gear's core."""
    psi_b = s_t / (2.0 * r) + _inv(math.acos(rb / r))           # half the tooth's angle at the base circle
    t_a = math.sqrt((ra / rb) ** 2 - 1.0)                       # roll angle at the tip
    flank = []                                                  # (radius, half angle), base circle to tip
    for i in range(N_INV + 1):
        t = t_a * i / N_INV
        flank.append((rb * math.sqrt(1.0 + t * t), psi_b - (t - math.atan(t))))
    flank[-1] = (ra, flank[-1][1])                              # exactly on the tip circle
    half_tip = flank[-1][1]
    r_in = rf - 0.25 * mn                                       # the outline closes this far inside the root circle
    pts = []
    if r_in < rb:
        pts.append((r_in, -psi_b))                              # radial line, right side
    pts += [(rho, -ang) for rho, ang in flank]                  # right flank, up
    pts += [(ra, -half_tip + 2.0 * half_tip * k / N_TIP) for k in range(1, N_TIP)]   # tip
    pts += [(rho, ang) for rho, ang in reversed(flank)]         # left flank, down
    if r_in < rb:
        pts.append((r_in, psi_b))                               # radial line, left side
    return [(rho * math.cos(phase + ang), rho * math.sin(phase + ang)) for rho, ang in pts]


def refusal(h):
    """The reason the pair cannot be built, or None."""
    beta = math.degrees(h['beta'])
    if not 5.0 - 1e-9 <= beta <= 45.0 + 1e-9:
        return 'helix angle beta = %g deg: it must be from 5 to 45 degrees' % beta
    if h['mn'] <= 0 or h['b_asked'] <= 0:
        return 'the normal module and the face width must be greater than zero'
    if not 0.0 < math.degrees(h['alpha_n']) < 45.0:
        return 'normal pressure angle alpha_n = %g deg: it must be between 0 and 45 degrees' % math.degrees(h['alpha_n'])
    for name, z in (('z1', h['z1']), ('z2', h['z2'])):
        if z < h['zmin']:
            return ('%s = %d teeth would be undercut: the smallest number of teeth is z_min = 2 cos(beta) / '
                    'sin^2(alpha_t) = %.2f' % (name, z, h['zmin']))
    if h['b'] <= 0:
        return ("face width b = %g mm is too small: gear 1's twist would be under one %s of a degree"
                % (h['b_asked'], 'tenth' if h['nsw'] == 1 else 'two tenths'))
    for name, r, rb, ra in (('gear 1', h['r1'], h['rb1'], h['ra1']), ('gear 2', h['r2'], h['rb2'], h['ra2'])):
        if tip_half_angle(r, rb, ra, h['s_t']) <= 0:
            return '%s: the teeth come to a point below the tip circle' % name
    if h['tip2'] > BACKLASH / 4.0 + 1e-12:
        return ("gear 2's twist per sweep, kept in tenths of a degree (%.1f deg instead of %.4f deg), moves its tips "
                'by %.4f mm, more than a quarter of the backlash (%.3f mm)'
                % (h['tw2'], h['tw2_exact'], h['tip2'], BACKLASH / 4.0))
    return None


# ----------------------------------------------------------------------------------------------------------------
# Fusion helpers (component frame, mm in, cm to the API)
# ----------------------------------------------------------------------------------------------------------------
def _new_occ(root, name, x=0.0):
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(x * CM, 0.0, 0.0)
    occ = root.occurrences.addNewComponent(m)
    occ.component.name = name
    return occ


def _xy_plane(comp, z):
    """A plane square to z at height z (mm, in the component's own frame)."""
    if abs(z) < 1e-12:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z * CM))
    return comp.constructionPlanes.add(ci)


def _extrude(comp, sk, profile, height, op=adsk.fusion.FeatureOperations.NewBodyFeatureOperation):
    """Extrude a profile `height` mm towards +z of the component."""
    normal = sk.xDirection.crossProduct(sk.yDirection)
    inp = comp.features.extrudeFeatures.createInput(profile, op)
    inp.setDistanceExtent(False, VI(height * CM * (1.0 if normal.z > 0 else -1.0)))
    return comp.features.extrudeFeatures.add(inp)


def _circles_body(comp, z0, z1, radii, name):
    """A cylinder (one radius) or a tube (two radii) on the component's z axis from z0 to z1 (mm)."""
    sk = comp.sketches.add(_xy_plane(comp, z0))
    c = sk.modelToSketchSpace(P3(0.0, 0.0, z0 * CM))
    for rr in radii:
        sk.sketchCurves.sketchCircles.addByCenterRadius(c, rr * CM)
    want_loops = len(radii)
    prof = None
    for p in sk.profiles:
        if p.profileLoops.count == want_loops:
            prof = p
    if prof is None:
        raise RuntimeError('%s: no profile with %d loops' % (name, want_loops))
    body = _extrude(comp, sk, prof, z1 - z0).bodies.item(0)
    body.name = name
    return body


def _box_body(comp, x0, x1, y0, y1, z0, z1, name):
    sk = comp.sketches.add(_xy_plane(comp, z0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(sk.modelToSketchSpace(P3(x0 * CM, y0 * CM, z0 * CM)),
                                                     sk.modelToSketchSpace(P3(x1 * CM, y1 * CM, z0 * CM)))
    body = _extrude(comp, sk, sk.profiles.item(0), z1 - z0).bodies.item(0)
    body.name = name
    return body


def _tooth_profile(comp, pts, z):
    """The tooth outline as ONE closed polyline of sketch lines sharing their points, on the plane z (mm)."""
    sk = comp.sketches.add(_xy_plane(comp, z))
    sk.isComputeDeferred = True
    lines = sk.sketchCurves.sketchLines
    q = [sk.modelToSketchSpace(P3(x * CM, y * CM, z * CM)) for x, y in pts]
    first = lines.addByTwoPoints(q[0], q[1])
    prev = first.endSketchPoint
    for p in q[2:]:
        prev = lines.addByTwoPoints(prev, p).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    if sk.profiles.count != 1:
        raise RuntimeError('the tooth outline gave %d profiles, not one' % sk.profiles.count)
    return sk.profiles.item(0)


def _sweep(comp, profile, z0, z1, twist_expr, body):
    """Sweep the profile along the gear's axis from z0 to z1 (mm) with a twist, joined to the gear's core."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    line = sk.sketchCurves.sketchLines.addByTwoPoints(sk.modelToSketchSpace(P3(0.0, 0.0, z0 * CM)),
                                                      sk.modelToSketchSpace(P3(0.0, 0.0, z1 * CM)))
    path = comp.features.createPath(line, False)
    sweeps = comp.features.sweepFeatures
    inp = sweeps.createInput(profile, path, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    inp.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    inp.twistAngle = VS(twist_expr)
    try:
        inp.participantBodies = [body]
    except Exception:
        pass
    return sweeps.add(inp)


def _tip_vertices(comp, ra, z):
    """Angles (rad) of the body vertices on the tip circle at height z (mm)."""
    out = []
    for body in comp.bRepBodies:
        for v in body.vertices:
            p = v.geometry
            if abs(math.hypot(p.x, p.y) - ra * CM) < 1e-5 and abs(p.z - z * CM) < 1e-5:
                out.append(math.atan2(p.y, p.x))
    return out


def _tip_angle(comp, ra, z):
    """The mean angle of the tip points at height z: the centre of the (one) tooth there."""
    angs = _tip_vertices(comp, ra, z)
    if not angs:
        raise RuntimeError('%s: no tooth tip found at z = %.3f mm' % (comp.name, z))
    return math.atan2(sum(math.sin(t) for t in angs), sum(math.cos(t) for t in angs))


def _settle_twist(sweep, comp, ra, z_top, start, want):
    """Measure the turn of the tooth from its bottom (at angle `start`) to z_top and make it `want` (rad,
    anticlockwise seen from above positive). Fusion's sign of a sweep's twist is read off the geometry, not assumed:
    if the tooth turned the wrong way the stored value is negated. Returns the twist Fusion stored (rad) and k,
    the sign that turns the stored value into the measured turn."""
    got = _wrap(_tip_angle(comp, ra, z_top) - start)
    if abs(_wrap(got - want)) > TOL and abs(want) > TOL and abs(_wrap(got + want)) <= TOL:
        sweep.twistAngle.value = -sweep.twistAngle.value
        got = _wrap(_tip_angle(comp, ra, z_top) - start)
    if abs(_wrap(got - want)) > TOL:
        raise RuntimeError('%s: the sweep turned the tooth %.4f deg, not %.4f deg'
                           % (comp.name, math.degrees(got), math.degrees(want)))
    stored = sweep.twistAngle.value
    k = -1.0 if stored * want < 0 else 1.0
    return stored, k


def _gear(occ, name, z, r, rb, ra, rf, mn, s_t, bw, phase, hand, tw, herringbone, bore):
    """One gear in its own component: the core (root circle, bored), one tooth swept with a twist, then the
    circular pattern. `tw` is the twist per sweep in degrees (whole tenths), `hand` +1 right, -1 left."""
    comp = occ.component
    body = _circles_body(comp, 0.0, bw, (rf, bore / 2.0), name)
    pts = tooth_outline(r, rb, ra, rf, s_t, mn, phase)
    prof = _tooth_profile(comp, pts, 0.0)
    want = hand * math.radians(tw)
    stored = []
    if not herringbone:
        sw = _sweep(comp, prof, 0.0, bw, '%.1f deg' % (hand * tw), body)
        s, k = _settle_twist(sw, comp, ra, bw, phase, want)
        stored.append(s)
        feats = [sw]
    else:
        half = bw / 2.0
        sw1 = _sweep(comp, prof, 0.0, half, '%.1f deg' % (hand * tw), body)
        s1, k = _settle_twist(sw1, comp, ra, half, phase, want)
        turned = k * s1                                         # the turn Fusion's stored twist gives
        prof2 = _tooth_profile(comp, tooth_outline(r, rb, ra, rf, s_t, mn, phase + turned), half)
        sw2 = _sweep(comp, prof2, half, bw, '%.1f deg' % round(-math.degrees(s1), 1), body)
        s2, _ = _settle_twist(sw2, comp, ra, bw, phase + turned, -turned)
        stored += [s1, s2]
        feats = [sw1, sw2]
    coll = adsk.core.ObjectCollection.create()
    for f in feats:
        coll.add(f)
    pats = comp.features.circularPatternFeatures
    pin = pats.createInput(coll, comp.zConstructionAxis)
    pin.quantity = VI(float(z))
    pin.totalAngle = VS('360 deg')
    pin.isSymmetric = False
    pat = pats.add(pin)
    n_tips = (N_TIP + 1) * z
    if len(_tip_vertices(comp, ra, bw)) != n_tips:
        pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
    if comp.bRepBodies.count != 1 or len(_tip_vertices(comp, ra, bw)) != n_tips:
        raise RuntimeError('%s: the pattern gave %d bodies and %d tip points at the top face (want 1 and %d)'
                           % (name, comp.bRepBodies.count, len(_tip_vertices(comp, ra, bw)), n_tips))
    comp.bRepBodies.item(0).name = name
    return stored


def _zangle(occ):
    m = occ.transform2
    return math.atan2(m.getCell(1, 0), m.getCell(0, 0))


def _drive(j, deg):
    j.jointMotion.rotationValue = math.radians(deg)
    adsk.doEvents()


def _revolute(root, gear, shaft, name):
    """An As-built revolute joint about z between the gear and its shaft, on the gear's axis. A positive joint value
    must turn the gear anticlockwise about +z; that is measured, and the joint is made the other way round if not."""
    for one, two in ((gear, shaft), (shaft, gear)):
        pt = gear.component.originConstructionPoint.createForAssemblyContext(gear)
        inp = root.asBuiltJoints.createInput(one, two, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(inp)
        j.name = name
        a0 = _zangle(gear)
        _drive(j, 10.0)
        turn = math.degrees(_wrap(_zangle(gear) - a0))
        _drive(j, 0.0)
        if abs(turn - 10.0) < 1e-4:
            return j
        j.deleteMe()
    raise RuntimeError('%s: no As-built revolute joint turns the gear anticlockwise about +z' % name)


def _rigid(root, plate, shaft, name):
    inp = root.asBuiltJoints.createInput(plate, shaft, None)
    inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def _motion_link(root, j1, j2, gear2, z1, z2):
    """Gear 2's joint turns the opposite way to gear 1's at the ratio of the teeth: z1/z2. Checked by driving."""
    want = -10.0 * z1 / z2
    rot = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
    for v1, v2, rev in ((z2, z1, True), (z2, z1, False), (z1, z2, True), (z1, z2, False)):
        inp = root.motionLinks.createInput(j1, j2)
        try:
            inp.motionOne = rot
            inp.motionTwo = rot
        except Exception:
            pass
        inp.valueOne = VS('%d deg' % v1)
        inp.valueTwo = VS('%d deg' % v2)
        inp.isReversed = rev
        ml = root.motionLinks.add(inp)
        a0 = _zangle(gear2)
        _drive(j1, 10.0)
        jv = math.degrees(j2.jointMotion.rotationValue)
        bt = math.degrees(_wrap(_zangle(gear2) - a0))
        _drive(j1, 0.0)
        if abs(jv - want) < 1e-4 and abs(bt - want) < 1e-4:
            ml.name = 'Gear 1 drives gear 2'
            return ml
        ml.deleteMe()
    raise RuntimeError('no Motion Link turns gear 2 by %.3f deg when gear 1 turns 10 deg' % want)


# ----------------------------------------------------------------------------------------------------------------
# The build
# ----------------------------------------------------------------------------------------------------------------
def build(mn=2.0, z1=20, z2=40, beta=20.0, alpha_n=20.0, b=12.0, herringbone=False):
    """Check the inputs (ValueError if refused), open a new Fusion design and build the pair in it.
    Returns the message that reports it."""
    if isinstance(herringbone, str):
        herringbone = herringbone.strip().lower() in ('yes', 'true', '1')
    if int(z1) != z1 or int(z2) != z2:
        raise ValueError('the numbers of teeth must be whole numbers')
    z1, z2 = int(z1), int(z2)
    if not (5.0 - 1e-9 <= float(beta) <= 45.0 + 1e-9):
        raise ValueError('helix angle beta = %g deg: it must be from 5 to 45 degrees' % float(beta))
    if float(mn) <= 0 or float(b) <= 0 or z1 < 1 or z2 < 1:
        raise ValueError('the normal module, the face width and the numbers of teeth must be greater than zero')
    h = helical(float(mn), z1, z2, float(beta), float(alpha_n), float(b), bool(herringbone))
    why = refusal(h)
    if why:
        raise ValueError(why)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    mn, a, bw = h['mn'], h['a'], h['b']
    shaft_d = SHAFT_D_PER_MN * mn
    bore = shaft_d + BORE_CLEARANCE

    plate = _new_occ(root, 'Base plate')
    shaft1 = _new_occ(root, 'Shaft 1')
    shaft2 = _new_occ(root, 'Shaft 2', a)
    gear1 = _new_occ(root, 'Gear 1')
    gear2 = _new_occ(root, 'Gear 2', a)
    plate.isGrounded = True
    for o in (shaft1, shaft2, gear1, gear2):
        o.isGrounded = False

    ymax = max(h['ra1'], h['ra2']) + PLATE_MARGIN
    _box_body(plate.component, -h['ra1'] - PLATE_MARGIN, a + h['ra2'] + PLATE_MARGIN, -ymax, ymax,
              -PLATE_GAP - PLATE_T, -PLATE_GAP, 'Base plate')
    for occ, name in ((shaft1, 'Shaft 1'), (shaft2, 'Shaft 2')):
        _circles_body(occ.component, -PLATE_GAP, bw + SHAFT_ABOVE, (shaft_d / 2.0,), name)

    # gear 1: a tooth centred on +x (on the line of centres) at the bottom face, right hand.
    # gear 2: a gap on the line of centres (at 180 deg) at the bottom face, so a tooth half a pitch from it; left hand.
    st1 = _gear(gear1, 'Gear 1', z1, h['r1'], h['rb1'], h['ra1'], h['rf1'], mn, h['s_t'], bw,
                0.0, +1, h['tw1'], h['herringbone'], bore)
    st2 = _gear(gear2, 'Gear 2', z2, h['r2'], h['rb2'], h['ra2'], h['rf2'], mn, h['s_t'], bw,
                math.pi - math.pi / z2, -1, h['tw2'], h['herringbone'], bore)

    _rigid(root, plate, shaft1, 'Shaft 1 fixed')
    _rigid(root, plate, shaft2, 'Shaft 2 fixed')
    j1 = _revolute(root, gear1, shaft1, 'Gear 1 turns')
    j2 = _revolute(root, gear2, shaft2, 'Gear 2 turns')
    _motion_link(root, j1, j2, gear2, z1, z2)

    try:
        app.activeViewport.fit()
    except Exception:
        pass

    tip2 = abs(abs(st2[0]) - math.radians(math.degrees(h['th2']) / h['nsw'])) * h['ra2']
    unit = 'tenth' if h['nsw'] == 1 else 'two tenths'
    lines = [
        '%s: %s gear pair built' % (VERSION, 'herringbone' if h['herringbone'] else 'helical'),
        'm_t = %.5f mm, alpha_t = %.3f deg, centre distance a = %.3f mm'
        % (h['mt'], math.degrees(h['at']), a),
        "face width b' = %.3f mm (asked %.3f mm; gear 1's twist made a whole number of %s of a degree)"
        % (bw, h['b_asked'], unit),
        'theta1 = %.3f deg (gear 1, right hand), theta2 = %.3f deg (gear 2, left hand)'
        % (math.degrees(h['th1']), math.degrees(h['th2'])),
        'twist Fusion stored per sweep: gear 1 %s deg, gear 2 %s deg (gear 2 tips off by %.4f mm, limit %.3f mm)'
        % (', '.join('%.1f' % math.degrees(s) for s in st1), ', '.join('%.1f' % math.degrees(s) for s in st2),
           tip2, BACKLASH / 4.0),
        'z_min = %.2f (z1 = %d, z2 = %d); transverse contact ratio %.3f, overlap ratio %.3f'
        % (h['zmin'], z1, z2, h['eps_a'], h['eps_b']),
        'm_n = %g mm, beta = %g deg, alpha_n = %g deg, backlash %.1f mm; herringbone: %s'
        % (mn, math.degrees(h['beta']), math.degrees(h['alpha_n']), BACKLASH, 'yes' if h['herringbone'] else 'no'),
    ]
    _last.clear()
    _last.update(h)
    _last.update(stored1=st1, stored2=st2)
    return '\n'.join(lines)


# ----------------------------------------------------------------------------------------------------------------
# The dialog
# ----------------------------------------------------------------------------------------------------------------
CMD_ID = 'mdfcHelicalPairPromptTest'


def _read_inputs(ins):
    def val(k):
        return ins.itemById(k).value
    return dict(mn=round(val('mn') / CM, 9), z1=int(val('z1')), z2=int(val('z2')),
                beta=round(math.degrees(val('beta')), 9), alpha_n=round(math.degrees(val('alpha_n')), 9),
                b=round(val('b') / CM, 9), herringbone=ins.itemById('herringbone').selectedItem.name == 'yes')


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            msg = build(**_read_inputs(args.command.commandInputs))
        except ValueError as e:
            ui.messageBox('Refused: %s\n\n%s' % (e, VERSION))
            return
        except Exception:
            ui.messageBox('Failed:\n%s\n\n%s' % (traceback.format_exc(), VERSION))
            return
        ui.messageBox(msg)


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            d = DEFAULTS
            ins.addValueInput('mn', 'Normal module m_n', 'mm', VS('%g mm' % d['mn']))
            ins.addIntegerSpinnerCommandInput('z1', 'Driver teeth z1', 3, 500, 1, d['z1'])
            ins.addIntegerSpinnerCommandInput('z2', 'Driven teeth z2', 3, 500, 1, d['z2'])
            ins.addValueInput('beta', 'Helix angle beta', 'deg', VS('%g deg' % d['beta']))
            ins.addValueInput('alpha_n', 'Normal pressure angle alpha_n', 'deg', VS('%g deg' % d['alpha_n']))
            ins.addValueInput('b', 'Face width b', 'mm', VS('%g mm' % d['b']))
            dd = ins.addDropDownCommandInput('herringbone', 'Herringbone', adsk.core.DropDownStyles.TextListDropDownStyle)
            dd.listItems.add('no', not d['herringbone'], '')
            dd.listItems.add('yes', bool(d['herringbone']), '')
            on_exec = _Execute()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('Failed:\n' + traceback.format_exc())


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Helical gear pair',
                                                       'A pair of helical or herringbone gears on a base plate')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('Failed:\n' + traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
