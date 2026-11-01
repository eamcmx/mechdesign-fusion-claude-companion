# =============================================================================
# RearAxleCarrier.py  -  Chapter 29 of "Mechanical Design in Autodesk Fusion with Claude": the rear axle's carrier,
# the subsystem that is set up on the bench and bolted into the banjo housing. A spiral bevel pinion of 12 teeth on
# two tapered roller bearings drives a crown wheel of 47 bolted to the differential's case; the case turns on two
# tapered roller bearings in the carrier's saddles under bearing caps; four spiders on a cross drive the two side
# gears. The sizing is lib/axle_design.py; the spiral teeth lib/spiral_bevel.py; the parts lib/mdfc_parts.py.
# Written from PROMPT.txt beside it; checked by checks/rear_axle_carrier_check.py.
#
#   INPUTS        the dialog: the final drive's module and face width, the differential's module and spiders
#   DERIVED       axle_design: the duty, the rating, the bearings, the differential; layout(): every position
#   HARD RULES    build() refuses a module whose pinion falls short in bending at the design torque, a crown wheel
#                 that would leave less than the revised ground clearance, a face over a third of the cone distance,
#                 and spiders that cannot mesh with both side gears
#   CONVENTIONS   the bevel pair's apex at the origin, the pinion along +X (forward), the axle along Y, the crown
#                 wheel on the -Y side; three subassemblies (pinion, differential, carrier), every part a component;
#                 gears in frames with their own z along their axis (Volume 1, Chapter 17)
#   ACCEPTANCE    the message; the ratio 47:12 measured; no interference at rest, through one pinion pitch and with
#                 the side gears turned in the case; every bearing on its seat; the parts list from Fusion
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

LIB = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import axle_design as ad        # noqa: E402
import spiral_bevel as sb       # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(ad)
importlib.reload(sb)
importlib.reload(mp)

VERSION = 'RearAxleCarrier 1.3 (2026-11-01)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
BACKLASH = 0.25                    # mm, shared by the two flanks of a mesh
CLEARANCE_MIN = 195.0              # the ground clearance after the change of Chapter 29 (mm)
R_STAT, WALL, GAP = 365.0, 8.0, 6.0
CASE_IN, CASE_OUT = 76.0, 84.0     # the differential case's spherical cavity and outside (mm)
GEAR_BACK = 74.0                   # the spiders' and side gears' spherical backs; washers from there to CASE_IN
HUB_Y = 74.0                       # the case's bearing shoulders at y = -74 and +74
LEG_Y = 82.0                       # the carrier's legs start here, clear of the crown wheel's bolt heads
CASE_BRG = '32213'
D_PIN = 22.0
CAP_BOLT_Z = 68.0                  # the bearing caps' bolts, clear of the cups (radius 60)
CW_BORE, CW_BOLT_R, FLANGE_R = 85.2, 93.0, 105.0   # the crown wheel's bore passes over the case (84); its bolts
HUB_R, HUB_BORE = 26.0, 26.2       # the side gears' hubs, in the case's hubs
SHAFT_SPLINE = (35.0, 40.0, 8, 7.0)   # the half shafts' splines: minor, major, number, width (Chapter 30)


# ---------- the design and the layout ----------
def design(m=6.5, F=52.0, m_d=5.5, N=4):
    b = ad.brief()
    d = ad.duty(b)
    u = ad.gear_duty(b)
    g = ad.bevel_geometry(m=m, F=F)
    pk = ad.rate(g, u['T1_peak'], u['n1_peak'], u['n_peak'], 1.0)
    cr = ad.rate(g, u['T1_cruise'], u['n1_cruise'], u['n_cruise'], ad.KA_TABLE[('light', 'light')])
    x = ad.differential(d, m=m_d, N=N, F=math.floor(0.5 * m_d * math.hypot(22, 15) / 3.0))
    return dict(b=b, d=d, u=u, g=g, peak=pk, cruise=cr, diff=x)


def refuse(D):
    g, pk, x = D['g'], D['peak'], D['diff']
    clr = R_STAT - (g['da2'] / 2.0 + WALL + GAP)
    if g['F'] > g['F_max'] + 1e-9:
        raise ValueError('Face width %.1f mm is more than a third of the cone distance, %.2f mm.' % (g['F'], g['F_max']))
    if pk['SF1'] < 1.0:
        raise ValueError('At module %.2f the pinion keeps a bending factor of %.3f at the design torque: under 1.'
                         % (g['m'], pk['SF1']))
    if clr < CLEARANCE_MIN:
        raise ValueError('At module %.2f the crown wheel (%.1f mm across) leaves %.1f mm of ground clearance, under %.0f.'
                         % (g['m'], g['da2'], clr, CLEARANCE_MIN))
    if abs(x['assembly'] - round(x['assembly'])) > 1e-9:
        raise ValueError('%d spiders cannot all mesh with side gears of 22 teeth: 2 z_s / N = %.2f.'
                         % (x['N'], x['assembly']))
    return clr


def layout(D):
    """Positions along the pinion's axis (x) and the axle (y), mm."""
    g = D['g']
    A, B = ad.SKF_TRB['32309'], ad.SKF_TRB['32308']
    C = ad.SKF_TRB[CASE_BRG]
    L = {}
    sd, cd = math.sin(math.radians(g['delta1'])), math.cos(math.radians(g['delta1']))
    L['heel_root'] = (g['Ae'] * sd - g['hf'] * cd, g['Ae'] * cd + g['hf'] * sd)      # pinion (r, x)
    L['collar'] = L['heel_root'][1] + 5.6        # the pinion's back face; the shim behind it
    L['shim'] = (L['collar'], L['collar'] + 1.0)
    L['xA'] = L['shim'][1]                        # bearing A's cone, back face, large end towards the pinion
    L['ppA'] = L['xA'] + (A[2] - A[3])            # its pressure point (SKF's a from the cup's wide face)
    L['xB'] = L['xA'] + A[2] + 63.3 + B[2]        # bearing B's cone back face, large end forward
    L['ppB'] = L['xB'] - (B[2] - B[3])
    L['spacer'] = (L['xA'] + A[2], L['xB'] - B[2])
    L['flange'] = (L['xB'], L['xB'] + 54.2)
    L['thread'] = (L['flange'][1], L['flange'][1] + 23.0)
    L['xm'] = g['Rm'] * cd                        # the pinion's mean point along x
    L['a'] = L['ppA'] - L['xm']
    L['s'] = L['ppB'] - L['ppA']
    # the case: bearing cones' large ends at y = -HUB_Y and +HUB_Y, facing inwards (face to face)
    L['ppC'] = HUB_Y + (C[2] - C[3])               # the pressure point, T - a out from the cone's back face
    L['ym'] = -g['Rm'] * math.cos(math.radians(g['delta2']))     # the crown wheel's mean point along y
    L['c'] = L['ppC'] + L['ym']                    # from bearing C1's pressure point to the crown wheel's mean point
    L['Lc'] = 2 * L['ppC']
    L['flange_y'] = (-69.0, -57.0)                 # the case's flange; the crown wheel's back face at -57
    return L


# ---------- frames and helpers ----------
def place(parent, name, x_axis=(1, 0, 0), z_axis=(0, 0, 1)):
    """A component inside `parent`, its origin at the apex, its own z along z_axis and x along x_axis."""
    zx, zy, zz = z_axis
    xx, xy, xz = x_axis
    y_axis = (zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx)
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(P3(0, 0, 0), V3(*x_axis), V3(*y_axis), V3(*z_axis))
    occ = parent.occurrences.addNewComponent(mat)
    occ.component.name = name
    if parent != parent.parentDesign.rootComponent:
        occ.isGroundToParent = True
    return occ


def body_local(occ, tb, name, material='Steel'):
    """A temporary body made in the component's own frame."""
    comp = occ.component
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    mp.set_material(body, material)
    return body


def cone_z(z0, z1, r0, r1):
    return mp.tbm().createCylinderOrCone(P3(0, 0, z0 * CM), r0 * CM, P3(0, 0, z1 * CM), r1 * CM)


def polyline(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def loft_tooth(occ, body, sections, z_teeth):
    """One tooth lofted through planar sections (lists of (x, y, z) in the component's frame, mm), joined to `body`,
    then patterned z_teeth times about the component's z axis."""
    comp = occ.component
    helper = comp.sketches.add(comp.xYConstructionPlane)
    helper.isVisible = False
    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.JoinFeatureOperation)
    for pts in sections:
        n = len(pts)
        sp = [helper.sketchPoints.add(P3(*[v * CM for v in pts[j]])) for j in (0, n // 3, (2 * n) // 3)]
        pin = comp.constructionPlanes.createInput()
        pin.setByThreePoints(sp[0], sp[1], sp[2])
        plane = comp.constructionPlanes.add(pin)
        plane.isLightBulbOn = False
        sk = comp.sketches.add(plane)
        back = sk.transform.copy()
        back.invert()
        sk.isComputeDeferred = True
        q = []
        for (x, y, z) in pts:
            p = P3(x * CM, y * CM, z * CM)
            p.transformBy(back)
            q.append((p.x, p.y))
        polyline(sk, q)
        sk.isComputeDeferred = False
        sk.isVisible = False
        li.loftSections.add(sk.profiles.item(0))
    li.isSolid = True
    li.participantBodies = [body]
    feats = adsk.core.ObjectCollection.create()
    feats.add(lofts.add(li))
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(z_teeth)
    pi.totalAngle = VS('360 deg')
    pats.add(pi)


def spiral_member(z, z_mate, g, pinion):
    return sb.member(z, z_mate, g['m'], g['F'], beta=g['beta'], alpha_n=g['alpha'], backlash=BACKLASH, pinion=pinion)


def shell_cap(axis, r_in, r_out, r_cyl, hole=0.0):
    """A spherical washer: the shell between r_in and r_out round the case's centre, within r_cyl of `axis`."""
    w = mp.cut(mp.sphere((0, 0, 0), r_out), mp.sphere((0, 0, 0), r_in))
    mp.intersect(w, mp.cyl([0.0] * 3, [(r_out + 2) * a for a in axis], r_cyl))
    if hole:
        mp.cut(w, mp.cyl([0.0] * 3, [(r_out + 2) * a for a in axis], hole))
    return w


# ---------- the pinion assembly ----------
def pinion_assembly(root, D, L):
    g = D['g']
    asm = mp.new_occ(root, 'Pinion assembly')
    A, B = ad.SKF_TRB['32309'], ad.SKF_TRB['32308']
    sp = spiral_member(g['z1'], g['z2'], g, True)
    frame = dict(x_axis=(0, -1, 0), z_axis=(1, 0, 0))
    o = place(asm.component, 'Pinion shaft z%d' % g['z1'], **frame)
    hr, k = L['heel_root'], sp['k']
    blank = cone_z(k * hr[1], hr[1], k * hr[0], hr[0])                          # the root cone, toe to heel
    back_r = lambda zz: (g['Ae'] - zz * math.cos(sp['delta'])) / math.sin(sp['delta'])
    mp.union(blank, cone_z(hr[1] - 0.01, L['collar'] - 1.6, hr[0], back_r(L['collar'] - 1.6)))   # the back cone
    secs = [(hr[1] - 0.5, L['collar'], 28.5),                                    # the collar behind the teeth
            (L['collar'] - 0.01, L['spacer'][0], A[0] / 2.0),                   # bearing A's seat, then the spacer's
            (L['spacer'][0], L['spacer'][1], A[0] / 2.0),
            (L['spacer'][1], L['xB'], B[0] / 2.0)]                              # bearing B's seat
    for z0, z1, r in secs:
        mp.union(blank, cone_z(z0, z1, r, r))
    mp.union(blank, cone_z(L['flange'][0] - 0.01, L['thread'][1], 15.0, 15.0))   # the thread's core under the splines
    spl = spline_z(L['flange'][0], L['flange'][1], 32.0, 38.0, 8, 6.0)
    mp.union(blank, spl)
    body = body_local(o, blank, 'Pinion shaft')
    loft_tooth(o, body, [sb.section(sp, s) for s in sb.scales(sp, 7)], g['z1'])
    parts = [o]
    o = place(asm.component, 'Pinion depth shim', **frame)
    body_local(o, tube_z(L['shim'][0], L['shim'][1], 28.0, A[0] / 2.0 + 0.05), 'Shim')
    parts.append(o)
    # the cones with their rollers: A's large end at xA facing the pinion, B's at xB facing forward
    for tag, (dd, DD, T, a_, *_), x0, sgn in (('A 32309', A, L['xA'], 1), ('B 32308', B, L['xB'], -1)):
        Bw, Cw = cone_widths(tag)
        cone, cup, _ = mp.taper_bodies(dd, DD, T, Bw, Cw, (x0, 0, 0), (sgn, 0, 0))
        o = mp.new_occ(asm.component, 'Bearing %s cone' % tag)
        mp.add_body(o, cone, 'Cone', 'Steel')
        parts.append(o)
    o = mp.new_occ(asm.component, 'Spacer')
    mp.add_body(o, mp.tube((L['spacer'][0], 0, 0), (L['spacer'][1], 0, 0), 28.0, A[0] / 2.0 + 0.05), 'Spacer', 'Steel')
    parts.append(o)
    # the companion flange: a splined hub the seal runs on, and a disc for the propeller shaft's bolts
    fl = mp.tube((L['flange'][0], 0, 0), (L['flange'][1], 0, 0), 30.0, 15.0)
    mp.union(fl, mp.tube((L['flange'][1] - 12.0, 0, 0), (L['flange'][1], 0, 0), 60.0, 15.0))
    mp.cut(fl, mp.spline_hole((L['flange'][0] - 1, 0, 0), (L['flange'][1] + 1, 0, 0), 32.0, 38.0, 8, 6.0))
    for kk in range(4):
        t = math.pi / 4 + kk * math.pi / 2
        mp.cut(fl, mp.cyl((L['flange'][1] - 13, 47 * math.cos(t), 47 * math.sin(t)),
                          (L['flange'][1] + 1, 47 * math.cos(t), 47 * math.sin(t)), 5.25))
    o = mp.new_occ(asm.component, 'Companion flange')
    mp.add_body(o, fl, 'Companion flange', 'Steel')
    parts.append(o)
    o = mp.new_occ(asm.component, 'Washer')
    mp.add_body(o, mp.tube((L['thread'][0], 0, 0), (L['thread'][0] + 3.0, 0, 0), 27.0, 15.2), 'Washer', 'Steel')
    parts.append(o)
    o = mp.new_occ(asm.component, 'Pinion nut M30')
    mp.add_body(o, mp.nut(30.0, (L['thread'][0] + 3.0, 0, 0), (1, 0, 0), 18.0), 'Nut', 'Steel')
    parts.append(o)
    return asm, parts


def cone_widths(tag):
    """Inner and outer ring widths (B, C) of the SKF bearings read on skf.com."""
    return {'A 32309': (36.0, 30.0), 'B 32308': (33.0, 27.0), 'C1 32212': (28.0, 24.0), 'C2 32212': (28.0, 24.0),
            'C1 32213': (31.0, 27.0), 'C2 32213': (31.0, 27.0)}[tag]


def tube_z(z0, z1, r_out, r_in):
    t = cone_z(z0, z1, r_out, r_out)
    return mp.cut(t, cone_z(z0 - 1, z1 + 1, r_in, r_in))


def spline_z(z0, z1, d_minor, d_major, n, w):
    """Splines on the component's z axis (the pinion's own frame)."""
    return mp.spline_shaft((0, 0, z0), (0, 0, z1), d_minor, d_major, n, w)


# ---------- the differential ----------
def differential_assembly(root, D, L):
    g, x = D['g'], D['diff']
    asm = mp.new_occ(root, 'Differential assembly')
    C = ad.SKF_TRB[CASE_BRG]
    parts = []
    # the case: a sphere with its cavity, the flange for the crown wheel, the spigot, two hubs, holes for the cross
    case = mp.intersect(mp.sphere((0, 0, 0), CASE_OUT), mp.box(-100.0, 100.0, -HUB_Y, HUB_Y, -100.0, 100.0))
    fy0, fy1 = L['flange_y']
    mp.union(case, mp.cyl((0, fy0, 0), (0, fy1, 0), FLANGE_R))                            # the flange
    mp.union(case, mp.cyl((0, fy1 - 0.01, 0), (0, -45.0, 0), CW_BORE - 0.2))              # the spigot
    for s in (-1, 1):
        mp.union(case, mp.cyl((0, s * 60.0, 0), (0, s * HUB_Y, 0), 40.0))                  # the bearing shoulder
        mp.union(case, mp.cyl((0, s * (HUB_Y - 0.01), 0), (0, s * (HUB_Y + C[2] + 1.0), 0), C[0] / 2.0))   # the seat
    mp.cut(case, mp.sphere((0, 0, 0), CASE_IN))                                           # the cavity
    for s in (-1, 1):
        mp.cut(case, mp.cyl((0, s * 40.0, 0), (0, s * (HUB_Y + C[2] + 2.0), 0), HUB_BORE))        # the side gears' hubs
    for ax in ((1, 0, 0), (0, 0, 1)):
        mp.cut(case, mp.cyl([-90 * a for a in ax], [90 * a for a in ax], D_PIN / 2.0 + 0.05))
    for t in (math.pi / 4, 5 * math.pi / 4):                                           # two windows
        c = (70 * math.cos(t), 0, 70 * math.sin(t))
        mp.cut(case, mp.cyl([0.6 * v for v in c], [1.4 * v for v in c], 45.0))
    for kk in range(10):                                                               # the crown wheel's bolt holes
        t = 2 * math.pi * kk / 10 + math.pi / 10
        mp.cut(case, mp.cyl((CW_BOLT_R * math.cos(t), fy0 - 1, CW_BOLT_R * math.sin(t)),
                            (CW_BOLT_R * math.cos(t), fy1 + 1, CW_BOLT_R * math.sin(t)), 6.6))
    o = mp.new_occ(asm.component, 'Differential case')
    mp.add_body(o, case, 'Case', 'Iron, Cast')
    parts.append(o)
    # the crown wheel, in its own frame (z along -Y)
    sc = spiral_member(g['z2'], g['z1'], g, False)
    cw = place(asm.component, 'Crown wheel z%d' % g['z2'], x_axis=(1, 0, 0), z_axis=(0, -1, 0))
    sd2, cd2 = math.sin(sc['delta']), math.cos(sc['delta'])
    heel_root = (g['Ae'] * sd2 - g['hf'] * cd2, g['Ae'] * cd2 + g['hf'] * sd2)
    k = sc['k']
    back_r = lambda zz: (g['Ae'] - zz * cd2) / sd2
    blank = cone_z(k * heel_root[1], heel_root[1], k * heel_root[0], heel_root[0])
    mp.union(blank, cone_z(heel_root[1] - 0.01, -fy1, back_r(heel_root[1]), back_r(-fy1)))
    mp.cut(blank, cone_z(0, 80, CW_BORE, CW_BORE))
    for kk in range(10):
        t = 2 * math.pi * kk / 10 + math.pi / 10
        mp.cut(blank, mp.tbm().createCylinderOrCone(P3(CW_BOLT_R * math.cos(t) * CM, CW_BOLT_R * math.sin(t) * CM, -fy1 * CM),
                                                    6.05 * CM, P3(CW_BOLT_R * math.cos(t) * CM, CW_BOLT_R * math.sin(t) * CM,
                                                                  (-fy1 - 24) * CM), 6.05 * CM))
    body = body_local(cw, blank, 'Crown wheel')
    loft_tooth(cw, body, [sb.section(sc, s, math.pi / g['z2']) for s in sb.scales(sc, 7)], g['z2'])
    parts.append(cw)
    for kk in range(10):                                                               # M12 bolts from the flange
        t = 2 * math.pi * kk / 10 + math.pi / 10
        o = mp.new_occ(asm.component, 'Crown wheel bolt M12x35 %d' % (kk + 1))
        mp.add_body(o, mp.cap_screw(12.0, 35.0, (CW_BOLT_R * math.cos(t), fy0, CW_BOLT_R * math.sin(t)), (0, 1, 0)),
                    'Bolt', 'Steel')
        parts.append(o)
    # the cross: two pins through each other, along X and Z
    cr = mp.cyl((-(CASE_OUT - 1), 0, 0), (CASE_OUT - 1, 0, 0), D_PIN / 2.0)
    mp.union(cr, mp.cyl((0, 0, -(CASE_OUT - 1)), (0, 0, CASE_OUT - 1), D_PIN / 2.0))
    o = mp.new_occ(asm.component, 'Cross')
    mp.add_body(o, cr, 'Cross', 'Steel')
    parts.append(o)
    # spiders and side gears, straight bevel teeth
    psi, sp_phase, right_phase = phases(x['z_s'], x['z_p'], x['N'])
    gsp = sb.member(x['z_p'], x['z_s'], x['m'], x['F'], beta=0.0, backlash=BACKLASH, pinion=True)
    gsd = sb.member(x['z_s'], x['z_p'], x['m'], x['F'], beta=0.0, backlash=BACKLASH, pinion=True)
    movers = {}
    for i, (p, ph) in enumerate(zip(psi, sp_phase)):
        s = place(asm.component, 'Spider %d' % (i + 1), x_axis=(0, 1, 0), z_axis=(math.sin(p), 0, math.cos(p)))
        s.isGroundToParent = False
        bevel_body(s, gsp, ph, bore=D_PIN + 0.2, hub=None)
        ax = (math.sin(p), 0, math.cos(p))
        movers['Spider %d' % (i + 1)] = (s, ax)
        o = mp.new_occ(asm.component, 'Spider washer %d' % (i + 1))
        mp.add_body(o, shell_cap(ax, GEAR_BACK + 0.1, CASE_IN - 0.05, 22.0, D_PIN / 2.0 + 0.2), 'Washer', 'Steel')
        parts += [s, o]
    for side, zax, xax, ph in (('L', (0, 1, 0), (0, 0, 1), 0.0), ('R', (0, -1, 0), (0, 0, 1), right_phase)):
        s = place(asm.component, 'Side gear %s' % side, x_axis=xax, z_axis=zax)
        s.isGroundToParent = False
        bevel_body(s, gsd, ph, bore=None, hub=(HUB_R, HUB_Y + C[2]))
        movers['Side gear %s' % side] = (s, zax)
        o = mp.new_occ(asm.component, 'Side gear washer %s' % side)
        mp.add_body(o, shell_cap(zax, GEAR_BACK + 0.1, CASE_IN - 0.05, 52.0, HUB_BORE), 'Washer', 'Steel')
        parts += [s, o]
    for tag, sgn in (('C1 ' + CASE_BRG, -1), ('C2 ' + CASE_BRG, 1)):
        Bw, Cw = cone_widths(tag)
        cone, _, _ = mp.taper_bodies(C[0], C[1], C[2], Bw, Cw, (0, sgn * HUB_Y, 0), (0, sgn, 0))
        o = mp.new_occ(asm.component, 'Bearing %s cone' % tag)
        mp.add_body(o, cone, 'Cone', 'Steel')
        parts.append(o)
    return asm, parts, movers, (psi, gsp, gsd)



def bevel_body(occ, g, phase, bore=None, hub=None):
    """A straight bevel gear of the differential: its root cone, a back face trimmed to the sphere the case's washers
    sit on, a bore or a splined hub, then the teeth."""
    sd, cd = math.sin(g['delta']), math.cos(g['delta'])
    heel_root = (g['Ae'] * sd - g['hf'] * cd, g['Ae'] * cd + g['hf'] * sd)
    k = g['k']
    z_back = heel_root[1] + 4.0
    blank = cone_z(k * heel_root[1], heel_root[1], k * heel_root[0], heel_root[0])
    back_r = (g['Ae'] - z_back * cd) / sd
    mp.union(blank, cone_z(heel_root[1] - 0.01, z_back, (g['Ae'] - heel_root[1] * cd) / sd, back_r))
    mp.intersect(blank, mp.sphere((0, 0, 0), GEAR_BACK))
    if hub:
        mp.union(blank, cone_z(z_back - 6.0, hub[1], hub[0], hub[0]))
        dmin, dmaj, n, w = SHAFT_SPLINE
        mp.cut(blank, mp.spline_hole((0, 0, k * heel_root[1] - 1), (0, 0, hub[1] + 1), dmin, dmaj, n, w))
    if bore:
        mp.cut(blank, cone_z(k * heel_root[1] - 1, z_back + 1, bore / 2.0, bore / 2.0))
    body = body_local(occ, blank, occ.component.name)
    loft_tooth(occ, body, [sb.section(g, s, phase) for s in sb.scales(g, 2)], g['z'])
    return body


def phases(z_s, z_p, N):
    """As in Chapter 27's CarDifferential: which way each spider and the right side gear must be turned."""
    def shows(a, phase, z):
        f = ((a - phase) * z / (2 * math.pi)) % 1.0
        if f < 1e-9 or f > 1 - 1e-9:
            return 'tooth'
        if abs(f - 0.5) < 1e-9:
            return 'gap'
        return None
    psi = [2 * math.pi * i / N for i in range(N)]
    sp = []
    for p in psi:
        left = shows(p, 0.0, z_s)
        if left is None:
            raise ValueError('spider at %.1f deg meets the left side gear between a tooth and a gap' % math.degrees(p))
        sp.append(math.pi / z_p if left == 'tooth' else 0.0)
    towards_right = [shows(math.pi, ph, z_p) for ph in sp]
    right = 0.0 if towards_right[0] == 'gap' else math.pi / z_s
    return psi, sp, right


# ---------- the carrier ----------
def carrier(root, D, L):
    asm = mp.new_occ(root, 'Carrier')
    A, B, C = ad.SKF_TRB['32309'], ad.SKF_TRB['32308'], ad.SKF_TRB[CASE_BRG]
    parts = []
    xA, xB = L['xA'], L['xB']
    cupA = (xA + A[2] - 30.0, xA + A[2])              # the cups' extents along x
    cupB = (xB - B[2], xB - B[2] + 27.0)
    # the casting: the mounting plate, the nose round the pinion's bearings, two legs to the case's bearings
    x_plate = (165.0, 181.0)
    body = mp.cyl((x_plate[0], 0, 0), (x_plate[1], 0, 0), 200.0)
    nose = mp.cyl((x_plate[0], 0, 0), (cupB[1] + 22.0, 0, 0), 62.0)
    mp.union(body, nose)
    yC = HUB_Y + C[2]                                  # the cups' outer faces
    for s in (-1, 1):
        y0, y1 = s * LEG_Y, s * (yC + 16.0)
        leg = mp.box(0.0, x_plate[0] + 0.01, min(y0, y1), max(y0, y1), -75.0, 75.0)
        mp.cut(leg, mp.cyl((0, y0 - s, 0), (0, y1 + s, 0), C[1] / 2.0 + 0.05))
        for zb in (-CAP_BOLT_Z, CAP_BOLT_Z):                                              # tapped for the caps' bolts
            mp.cut(leg, mp.cyl((-1.0, (y0 + y1) / 2.0, zb), (16.0, (y0 + y1) / 2.0, zb), 6.0))
        mp.union(body, leg)
    # the nose's bores: cup A, the shoulders, cup B, the seal
    mp.cut(body, mp.cyl((x_plate[0] - 1, 0, 0), (cupA[1], 0, 0), A[1] / 2.0 + 0.05))
    mp.cut(body, mp.cyl((cupA[1] - 0.01, 0, 0), (cupB[0] + 0.01, 0, 0), 46.5))
    mp.cut(body, mp.cyl((cupB[0], 0, 0), (cupB[1] + 12.0, 0, 0), B[1] / 2.0 + 0.05))
    mp.cut(body, mp.cyl((cupB[1] + 11.9, 0, 0), (cupB[1] + 23.0, 0, 0), 42.6))
    for kk in range(10):
        t = 2 * math.pi * kk / 10
        mp.cut(body, mp.cyl((x_plate[0] - 1, 185 * math.cos(t), 185 * math.sin(t)),
                            (x_plate[1] + 1, 185 * math.cos(t), 185 * math.sin(t)), 7.0))
    o = mp.new_occ(asm.component, 'Carrier casting')
    mp.add_body(o, body, 'Carrier casting', 'Iron, Cast')
    parts.append(o)
    # the bearing caps behind the case's bearings, two bolts each
    for s, tag in ((-1, 'C1'), (1, 'C2')):
        y0, y1 = s * LEG_Y, s * (yC + 16.0)
        cap = mp.box(-75.0, 0.0, min(y0, y1), max(y0, y1), -75.0, 75.0)
        mp.cut(cap, mp.cyl((0, y0 - s, 0), (0, y1 + s, 0), C[1] / 2.0 + 0.05))
        for zb in (-CAP_BOLT_Z, CAP_BOLT_Z):
            mp.cut(cap, mp.cyl((-76.0, (y0 + y1) / 2.0, zb), (1.0, (y0 + y1) / 2.0, zb), 6.6))
        o = mp.new_occ(asm.component, 'Bearing cap %s' % tag)
        mp.add_body(o, cap, 'Cap', 'Iron, Cast')
        parts.append(o)
        for zb in (-CAP_BOLT_Z, CAP_BOLT_Z):
            o = mp.new_occ(asm.component, 'Cap bolt M12x90 %s %s' % (tag, 'upper' if zb > 0 else 'lower'))
            mp.add_body(o, mp.cap_screw(12.0, 90.0, (-75.0, (y0 + y1) / 2.0, zb), (1, 0, 0)), 'Bolt', 'Steel')
            parts.append(o)
        o = mp.new_occ(asm.component, 'Adjuster %s' % tag)
        mp.add_body(o, mp.tube((0, s * yC, 0), (0, s * (yC + 16.0), 0), C[1] / 2.0, 36.0), 'Adjuster', 'Steel')
        parts.append(o)
    # the cups
    for tag, (dd, DD, T, a_, *_), x0, sgn in (('A 32309', A, xA, 1), ('B 32308', B, xB, -1)):
        Bw, Cw = cone_widths(tag)
        _, cup, _ = mp.taper_bodies(dd, DD, T, Bw, Cw, (x0, 0, 0), (sgn, 0, 0))
        o = mp.new_occ(asm.component, 'Bearing %s cup' % tag)
        mp.add_body(o, cup, 'Cup', 'Steel')
        parts.append(o)
    for tag, sgn in (('C1 ' + CASE_BRG, -1), ('C2 ' + CASE_BRG, 1)):
        Bw, Cw = cone_widths(tag)
        _, cup, _ = mp.taper_bodies(C[0], C[1], C[2], Bw, Cw, (0, sgn * HUB_Y, 0), (0, sgn, 0))
        o = mp.new_occ(asm.component, 'Bearing %s cup' % tag)
        mp.add_body(o, cup, 'Cup', 'Steel')
        parts.append(o)
    o = mp.new_occ(asm.component, 'Pinion seal 60x85x10')
    mp.add_body(o, mp.tube((cupB[1] + 12.0, 0, 0), (cupB[1] + 22.0, 0, 0), 42.5, 30.05), 'Seal', 'Rubber, Silicone')
    parts.append(o)
    return asm, parts


# ---------- joints ----------
def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def origin_point(occ, at=(0.0, 0.0, 0.0)):
    sk = occ.component.sketches.add(occ.component.xYConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(P3(*[v * CM for v in at])).createForAssemblyContext(occ)


def revolute_about(comp, moving, fixed, axis, name, at=(0.0, 0.0, 0.0)):
    """An As-built revolute joint in `comp` turning `moving` about `axis`; the joint directions are tried and the
    one that turns the part about the axis is kept (Volume 1, Chapter 17). Returns (joint, sign)."""
    J = adsk.fusion.JointDirections
    for direction in (J.ZAxisJointDirection, J.XAxisJointDirection, J.YAxisJointDirection):
        inp = comp.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(origin_point(moving, at)))
        inp.setAsRevoluteJointMotion(direction)
        j = comp.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.2
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        Dm = [[sum(r1[a][k] * r0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
        w = (Dm[2][1] - Dm[1][2], Dm[0][2] - Dm[2][0], Dm[1][0] - Dm[0][1])
        # the turn measured in the parent's frame; the subassemblies sit at the identity, so this is the world
        dot = sum(w[i] * axis[i] for i in range(3))
        if abs(dot) > 0.1:
            j.name = name
            return j, (1 if dot > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about %s' % (moving.name, axis))


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        w = o.component.name.split()
        if w[-1].isdigit():
            w = w[:-1]
        if w[-1] in ('upper', 'lower'):
            w = w[:-2]
        if w[-1] in ('L', 'R', 'C1', 'C2'):
            w = w[:-1]
        n = ' '.join(w)
        count[n] = count.get(n, 0) + 1
    return count


# ---------- the carrier assembly ----------
def build(m=6.5, F=52.0, m_d=5.5, N=4):
    D = design(m, F, m_d, int(round(N)))
    clr = refuse(D)
    L = layout(D)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design_ = adsk.fusion.Design.cast(app.activeProduct)
    design_.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design_.rootComponent
    car, _ = carrier(root, D, L)
    car.isGrounded = True
    pin, _ = pinion_assembly(root, D, L)
    dif, _, movers, _ = differential_assembly(root, D, L)
    jp, sp_ = revolute_about(root, pin, car, (1, 0, 0), 'Pinion turns')
    jc, sc_ = revolute_about(root, dif, car, (0, 1, 0), 'Case turns')
    g = D['g']
    k = math.gcd(g['z1'], g['z2'])
    li = root.motionLinks.createInput(jp, jc)
    li.valueOne = VS('%d deg' % (360 * g['z2'] // k))
    li.valueTwo = VS('%d deg' % (360 * g['z1'] // k))
    li.isReversed = (sp_ * sc_) < 0                    # the pinion's turn about +X drives the case about +Y
    root.motionLinks.add(li).name = 'Final drive %d:%d' % (g['z1'], g['z2'])
    case_occ = [o for o in dif.component.occurrences if o.component.name == 'Differential case'][0]
    case_px = case_occ.createForAssemblyContext(dif)
    for name, (occ, axis) in movers.items():
        # a joint made inside the subassembly did not move its part; made in the root between the parts' proxies, it
        # does, and the part still turns with the case when the case turns
        revolute_about(root, occ.createForAssemblyContext(dif), case_px, axis, name + ' turns in the case')
    for occ, c in ((car, 'Paint - Enamel Glossy (Grey)'), (pin, 'Paint - Enamel Glossy (Blue)'),
                   (dif, 'Paint - Enamel Glossy (Yellow)')):
        mp.colour(design_, occ, c)
    app.activeViewport.fit()
    pl = parts_list(root)
    pk, cr, x = D['peak'], D['cruise'], D['diff']
    return ('Rear axle carrier built: spiral bevel %d:%d (%.4f), module %.2f, face %.0f mm, crown wheel %.1f mm '
            'across, ground clearance %.1f mm.\n'
            'At the design torque: pinion bending factor %.3f, contact %.3f; at full power in sixth for the life: %.3f '
            'and %.3f.\n'
            'Pinion bearings 32309 and 32308, %.1f mm behind the mesh and %.1f mm apart; case bearings 32213, '
            '%.1f mm apart.\n'
            'Differential: %d spiders of %d teeth, side gears of %d, module %.1f, face %.0f: bending factors %.3f '
            'and %.3f.\n\nParts: %d components; %s.\n\n[%s]'
            % (g['z1'], g['z2'], g['z2'] / g['z1'], g['m'], g['F'], g['da2'], clr, pk['SF1'], pk['SH1'], cr['SF1'],
               cr['SH1'], L['a'], L['s'], L['Lc'], x['N'], x['z_p'], x['z_s'], x['m'], x['F'], x['SF_side'],
               x['SF_spider'], sum(pl.values()), ', '.join('%d %s' % (v, kk) for kk, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('m', 'Final-drive module at the heel (mm)', 6.5), ('F', 'Final-drive face width (mm)', 52.0),
          ('m_d', 'Differential module (mm)', 5.5), ('N', 'Spiders', 4.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            ui.messageBox(build(**{k: ins.itemById(k).value for k, _, _ in FIELDS}), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for key, label, dflt in FIELDS:
            ins.addValueInput(key, label, '', VI(dflt))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'RearAxleCarrierCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Rear axle carrier', 'The rear axle carrier')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('RearAxleCarrier could not start:\n' + traceback.format_exc())
