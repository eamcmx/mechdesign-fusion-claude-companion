# -*- coding: utf-8 -*-
# =============================================================================
# ScaraToolJoints.py  -  the SCARA's third and fourth joints at the end of its second arm (Chapter 37).
#
# A HIWIN FB ball screw spline: one shaft carrying a screw and a spline, the screw nut's flange on the arm's top
# plate (its rotor's pilot up, its pulley above), the spline nut's flange under the base plate (its rotor's pilot
# down, its pulley below), each rotor turning in its own support bearing inside its flange. A toothed pulley on each
# nut, located on the rotor's pilot and held by six screws in the pilot's tapped holes; two Gates GT3 belts to the
# J3 and J4 servo motors inside the arm's end; the shaft sliding (J3) and turning (J4) with a tool flange at its end.
#
# Every number of the design comes from projects/scara_robot/lib/scara_design.py: tool_joints() (the shaft as a
# cantilever, the depth rule, the screw's life, the spline's torque, the motors, the belts, the brakes), SP (HIWIN
# Tables 9.2 and 9.3), nut_layout() (every height), LAY37 (the layout) and MOTOR34 (the motors). What the library
# does not give (the motors' flange thickness and bolt circles, the plates' length, the belt's thickness, clearances
# for drawing) is marked "drawn as" below.
#
# Fusion: millimetres and degrees in this file, centimetres and radians at the API (CM, math.radians).
# =============================================================================
import importlib
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('common/companion_paths.py not found above ' + os.path.abspath(__file__))
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP          # puts every lib/ folder on sys.path
importlib.reload(CP)
import scara_design as sd
importlib.reload(sd)
import mdfc_parts as mp
importlib.reload(mp)

VERSION = 'ScaraToolJoints v1 [prompt test, 11 Nov 2026]'
CMD_ID = 'mdfcScaraToolJointsPT'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

# drawn as (not in the library): small gaps so that parts that only locate on each other never overlap
FIT = 0.01          # mm, radial: a pulley on its pilot or motor shaft, a nut's housing in its plate
TAP = 0.05          # mm, radial: a tapped hole drawn at the thread's major diameter plus this
BEARING_GAP = 0.5   # mm, radial: between a nut's rotor and its flange (the support bearing's place)
SHAFT_GAP = 1.0     # mm, radial: the pulleys' bore round the shaft
X0 = 30.0           # mm: where the arm's end begins (its plates run from here to the shaft's axis, ending round)
BELT_T = 1.27       # mm: a GT3 belt's band from its land to its back (2.41 overall less the 1.14 teeth)
BELT_GAP = 0.02     # mm: the band's inside clear of the pulley's outside diameter
GROOVE_R, GROOVE_DEPTH = 1.0, 1.14      # mm: the pulleys' grooves, drawn as round slots of the GT3 tooth's depth
MOUNT = {3: dict(t=8.0, pcd=70.0, bolt=5, hole=5.5, L=16.0),     # the motors' flanges: 60 and 40 mm frames
         4: dict(t=6.0, pcd=46.0, bolt=4, hole=4.5, L=16.0)}     # (thickness, bolt circle, bolt, hole, length)
MATERIALS = {'steel': ('Steel', 'Steel, Alloy', 'Steel, Carbon'),
             'al': ('Aluminum', 'Aluminum 6061', 'Aluminum, Alloy'),
             'rubber': ('Rubber, Silicone', 'Rubber', 'Neoprene', 'Nylon 6/6')}

_handlers = []
_state = {}
_last = {}          # the last build: its joints (joint, sign), occurrences, lead and design numbers


# ---------------------------------------------------------------------------------------------------------------
# the design numbers
# ---------------------------------------------------------------------------------------------------------------
def j1_life(size, b=None):
    """The first joint's life with this size's shaft at the tool, as the library's sp25_cost() does it for the SP25:
    the shaft's mass beyond the SP20's (which Chapter 36's sizing carried) at the tool's distance from J1 on the
    horizontal leg, added to M11; the torque grows in proportion and the life falls with its cube."""
    b = b or sd.brief()

    def m_shaft(k):
        lay = sd.nut_layout(k)
        return math.pi / 4 * sd.SP[k]['d'] ** 2 * (lay['shaft'][1] - lay['shaft'][0]) * sd.RHO_STEEL
    r = math.hypot(b['x0'], b['across'] / 2) / 1000
    p = sd.inertia(b, b['m_rated'])
    qa = sd.ik(b, b['x0'], -b['across'] / 2)
    M11 = sd.dynamics(p, qa[1], 0, 0, 0, 0)[2]['M11']
    dm = m_shaft(size) - m_shaft(20)
    dM = dm * r ** 2
    base = sd.modules(b)['j1'][40]['Lh']
    return dict(life=base / ((M11 + dM) / M11) ** 3, dm=dm, share=dM / M11, base=base)


def design(size=20):
    """Chapter 37's numbers for this size (tool_joints()) and the three refusals of the prompt."""
    size = int(round(float(size)))
    if size not in sd.SP:
        raise ValueError('SP%d is not a HIWIN FB size here: choose one of %s'
                         % (size, ', '.join('SP%d' % k for k in sorted(sd.SP))))
    b = sd.brief()
    o = sd.tool_joints(b, size)
    o['size'] = size
    o['j1'] = j1_life(size, b)
    why = []
    f = o['cant']['f']
    if f < 20.0:
        why.append('the shaft swings at %.1f Hz at the full stroke, %.1f Hz (%.0f %%) under 20 Hz'
                   % (f, 20.0 - f, (20.0 - f) / 20.0 * 100))
    n1 = o['depth']['n1']
    if n1 < 1.5:
        why.append('its fatigue factor with the tool 100 mm down is %.2f, %.2f under 1.5' % (n1, 1.5 - n1))
    life = o['j1']['life']
    if life < b['life_h']:
        why.append('its shaft puts %.3f kg more at the tool than the SP20, which cuts the first joint\'s life to '
                   '%s h, %s h (%.1f %%) short of %s h' % (o['j1']['dm'], _n(life), _n(b['life_h'] - life),
                                                       (b['life_h'] - life) / b['life_h'] * 100, _n(b['life_h'])))
    o['refusals'] = why
    if why:
        raise ValueError('SP%d refused: it falls short, as %s.' % (size, '; '.join(why)))
    return o


def _n(x):
    return '{:,.0f}'.format(x).replace(',', ' ')


# ---------------------------------------------------------------------------------------------------------------
# geometry helpers (world millimetres; every group and every single part at the identity)
# ---------------------------------------------------------------------------------------------------------------
_mat_cache = {}


def _mat(body, key):
    try:
        if key not in _mat_cache:
            app = adsk.core.Application.get()
            mls = app.materialLibraries
            libs = [mls.itemByName('Fusion Material Library')] + [mls.item(i) for i in range(mls.count)]
            found = None
            for name in MATERIALS[key]:
                for lib in libs:
                    if lib is None:
                        continue
                    m = lib.materials.itemByName(name)
                    if m:
                        found = m
                        break
                if found:
                    break
            _mat_cache[key] = found
        if _mat_cache[key]:
            body.material = _mat_cache[key]
    except Exception:
        pass


def _part(group, name, tb, mat):
    """A single part: its own component inside the group, the body given in world millimetres."""
    occ = mp.new_occ(group.component, name)
    body = mp.add_body(occ, tb, name)
    _mat(body, mat)
    return occ


def _frame(p, down=False):
    m = adsk.core.Matrix3D.create()
    o = P3(p[0] * CM, p[1] * CM, p[2] * CM)
    if down:
        m.setWithCoordinateSystem(o, V3(1, 0, 0), V3(0, -1, 0), V3(0, 0, -1))
    else:
        m.setWithCoordinateSystem(o, V3(1, 0, 0), V3(0, 1, 0), V3(0, 0, 1))
    return m


def _kind(name, tb_local, mat, places):
    """Several of one part: one component, an occurrence at each place. tb_local is drawn in the part's own frame;
    places = [(group occurrence, (x, y, z) mm, down)] put that frame's origin there, its z up or down."""
    comp, occs = None, []
    for grp, p, down in places:
        m = _frame(p, down)
        if comp is None:
            o = grp.component.occurrences.addNewComponent(m)
            comp = o.component
            comp.name = name
            bf = comp.features.baseFeatures.add()
            bf.startEdit()
            comp.bRepBodies.add(tb_local, bf)
            bf.finishEdit()
            body = bf.bodies.item(0)
            body.name = name
            _mat(body, mat)
        else:
            o = grp.component.occurrences.addExistingComponent(comp, m)
        o.isGroundToParent = True
        occs.append(o)
    return occs


def _ring(n, R, c, a0_deg=0.0):
    return [(c[0] + R * math.cos(math.radians(a0_deg) + 2 * math.pi * k / n),
             c[1] + R * math.sin(math.radians(a0_deg) + 2 * math.pi * k / n)) for k in range(n)]


def _holes(body, pts, r, z0, z1):
    for x, y in pts:
        mp.cut(body, mp.cyl((x, y, z0), (x, y, z1), r))
    return body


def _r_od(z):
    """A GT3 pulley's outside diameter's radius: the pitch diameter less 0.76 mm (as inertias34())."""
    return (sd.pd(z) - 0.76) / 2.0


def _pulley(c, z, rim, bores, hub=None, holes=None):
    """A toothed pulley on a z axis through c: the toothed rim from rim[0] to rim[1] with z round-bottomed grooves of
    the GT3 tooth's depth, a plain hub (z0, z1, r), the bores [(z0, z1, r)] and the screw holes (R, r, z0, z1, n)."""
    x, y = c
    r_od = _r_od(z)
    b = mp.cyl((x, y, rim[0]), (x, y, rim[1]), r_od)
    rc = r_od - GROOVE_DEPTH + GROOVE_R
    for k in range(z):
        a = 2 * math.pi * k / z
        gx, gy = x + rc * math.cos(a), y + rc * math.sin(a)
        mp.cut(b, mp.cyl((gx, gy, rim[0] - 1.0), (gx, gy, rim[1] + 1.0), GROOVE_R))
    if hub:
        mp.union(b, mp.cyl((x, y, hub[0]), (x, y, hub[1]), hub[2]))
    for z0, z1, r in bores:
        mp.cut(b, mp.cyl((x, y, z0), (x, y, z1), r))
    if holes:
        R, r, z0, z1, n = holes
        _holes(b, _ring(n, R, c), r, z0, z1)
    return b


def _belt(group, name, c1, r1, c2, r2, z0, w):
    """A belt's band, sketched (the temporary-body manager makes no belt): around the large pulley at c1 and the
    small one at c2, inside radii r1 and r2, the two external tangents and the two arcs, an inner and an outer loop
    BELT_T apart; the band between them extruded w up from z0."""
    occ = mp.new_occ(group.component, name)
    comp = occ.component
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, VI(z0 * CM))
    plane = comp.constructionPlanes.add(pin)
    sk = comp.sketches.add(plane)
    sk.name = name + ' band'
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    D = math.hypot(dx, dy)
    u = (dx / D, dy / D)
    nrm = (-u[1], u[0])

    def S(p):
        return sk.modelToSketchSpace(P3(p[0] * CM, p[1] * CM, z0 * CM))
    for a, bb in ((r1, r2), (r1 + BELT_T, r2 + BELT_T)):
        al = math.acos((a - bb) / D)                 # the tangent points' angle from the line of centres

        def on(c, r, ang):
            return (c[0] + r * (math.cos(ang) * u[0] + math.sin(ang) * nrm[0]),
                    c[1] + r * (math.cos(ang) * u[1] + math.sin(ang) * nrm[1]))
        lines = sk.sketchCurves.sketchLines
        l1 = lines.addByTwoPoints(S(on(c1, a, al)), S(on(c2, bb, al)))
        l2 = lines.addByTwoPoints(S(on(c1, a, -al)), S(on(c2, bb, -al)))
        arcs = sk.sketchCurves.sketchArcs
        arcs.addByThreePoints(l1.startSketchPoint, S(on(c1, a, math.pi)), l2.startSketchPoint)
        arcs.addByThreePoints(l1.endSketchPoint, S(on(c2, bb, 0.0)), l2.endSketchPoint)
    prof = None
    for pr in sk.profiles:
        if pr.profileLoops.count == 2:
            prof = pr
    if prof is None:
        raise RuntimeError('the band of %s gave no profile between its two loops' % name)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    normal = sk.xDirection.crossProduct(sk.yDirection)
    inp.setDistanceExtent(False, VI(w * CM * (1 if normal.z > 0 else -1)))
    feat = ext.add(inp)
    body = feat.bodies.item(0)
    body.name = name
    _mat(body, 'rubber')
    sk.isVisible = False
    plane.isLightBulbOn = False
    return occ


# ---------------------------------------------------------------------------------------------------------------
# joints
# ---------------------------------------------------------------------------------------------------------------
def _point(occ, x, y):
    """A sketch point on the group's own xy plane at (x, y) mm, in the group's assembly context."""
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Joint origin'
    p = sk.sketchPoints.add(P3(x * CM, y * CM, 0))
    return p.createForAssemblyContext(occ), sk


def _asbuilt(root, name, moving, anchor, x, y, slide=False):
    pt, sk = _point(moving, x, y)
    geo = adsk.fusion.JointGeometry.createByPoint(pt)
    inp = root.asBuiltJoints.createInput(moving, anchor, geo)
    zdir = adsk.fusion.JointDirections.ZAxisJointDirection
    if slide:
        inp.setAsSliderJointMotion(zdir)
    else:
        inp.setAsRevoluteJointMotion(zdir)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    sk.isVisible = False
    return j


def _link(root, name, ja, jb, a, b):
    """A Motion Link: when joint a turns by angle a (rad), joint b turns by b."""
    inp = root.motionLinks.createInput(ja, jb)
    rot = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
    try:
        inp.motionOne = rot
        inp.motionTwo = rot
    except Exception:
        pass
    inp.valueOne = VI(a)
    inp.valueTwo = VI(b)
    inp.isReversed = False
    ml = root.motionLinks.add(inp)
    try:
        ml.name = name
    except Exception:
        pass
    return ml


def _angle(occ):
    t = occ.transform2
    return math.atan2(t.getCell(1, 0), t.getCell(0, 0))


def _calibrate(J, occ, links):
    """Fusion chooses each joint's sense. Drive each joint a little and read the groups' turns about world z: the
    sign makes sign * value the turn about +z (the slide along +z); a belt's link that turned its motor against its
    nut is reversed (an open belt turns both pulleys the same way)."""
    notes = []
    sl = J['j3'][0]
    z0 = occ['carriage'].transform2.translation.z
    sl.jointMotion.slideValue = 1.0
    adsk.doEvents()
    dz = occ['carriage'].transform2.translation.z - z0
    sl.jointMotion.slideValue = 0.0
    adsk.doEvents()
    J['j3'] = (sl, 1 if dz >= 0 else -1)
    if abs(abs(dz) - 1.0) > 1e-6:
        notes.append('the slider moved %.4f cm for 1 cm' % dz)
    t = 0.3
    for drv, chain, ratios in (('j4', ('spline', 'm4'), (1.0, sd.PULLEYS[4][1] / sd.PULLEYS[4][0])),
                               ('screw', ('m3',), (sd.PULLEYS[3][1] / sd.PULLEYS[3][0],))):
        keys = (drv,) + chain
        ang = {}
        for _attempt in range(4):
            J[drv][0].jointMotion.rotationValue = t
            adsk.doEvents()
            ang = {k: _angle(occ[k]) for k in keys}
            flipped = False
            prev = drv
            for k in chain:
                if ang[k] * ang[prev] < 0:
                    links[k].isReversed = not links[k].isReversed
                    flipped = True
                    break
                prev = k
            if not flipped:
                break
        val = {k: J[k][0].jointMotion.rotationValue for k in keys}
        for k in keys:
            J[k] = (J[k][0], 1 if ang[k] * val[k] >= 0 else -1)
        prev = drv
        for k, ratio in zip(chain, ratios):
            if abs(ang[prev]) > 1e-9 and abs(ang[k] / ang[prev] - ratio) > 1e-3:
                notes.append('%s turned %.3f times %s, not %.3f' % (k, ang[k] / ang[prev], prev, ratio))
            prev = k
        J[drv][0].jointMotion.rotationValue = 0.0
        adsk.doEvents()
    return notes


def pose(j3, j4):
    """J3 (mm, down from the top) and J4 (deg): the carriage slides, the shaft turns (the spline nut and the J4
    motor follow by their Motion Links); the screw nut is set by the numbers to the shaft's turn less the travel
    over the lead (the J3 motor follows by its link)."""
    J = _last['joints']
    sl, s3 = J['j3']
    sl.jointMotion.slideValue = -j3 * CM * s3
    r4, s4 = J['j4']
    r4.jointMotion.rotationValue = math.radians(j4) * s4
    a = j4 - 360.0 * j3 / _last['lead']
    a = (a + 180.0) % 360.0 - 180.0          # the same place: the screw nut's angle kept within a turn
    rs, ss = J['screw']
    rs.jointMotion.rotationValue = math.radians(a) * ss
    adsk.doEvents()


def parts_list(root):
    """{component name: count} of every part (an occurrence whose component has a body)."""
    out = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count:
            out[o.component.name] = out.get(o.component.name, 0) + 1
    return out


# ---------------------------------------------------------------------------------------------------------------
# the build
# ---------------------------------------------------------------------------------------------------------------
def build(size=20):
    size = int(round(float(size)))
    o = design(size)                      # refuses before any document is made
    b = sd.brief()
    s, L, lay = sd.SP[size], sd.LAY37, o['lay']
    xs, hw, wall, H, tb, tt = L['x_s'], L['half_w'], L['wall'], L['H'], L['t_base'], L['t_top']
    c = (xs, 0.0)
    rimw = sd.BELT_W[4] + 3.0             # the toothed rim: the belt and 1.5 mm either side (nut_layout())
    z_scr_top = lay['scr']['rotor'][1]
    z_spl_bot = lay['spl']['rotor'][0]
    bel = o['belts']
    m3 = (bel[3]['x_m'], bel[3]['y_m'])
    m4 = (bel[4]['x_m'], bel[4]['y_m'])

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    root = des.rootComponent

    g = {}
    for key, name in (('arm', 'Arm end'), ('screw', 'Screw side'), ('spline', 'Spline side'),
                      ('m3', 'J3 motor side'), ('m4', 'J4 motor side'), ('carriage', 'Carriage'),
                      ('shaft', 'Tool shaft')):
        g[key] = mp.new_occ(root, name)
    g['arm'].isGrounded = True
    arm = g['arm']

    # ---- the arm's end: plates, walls -----------------------------------------------------------------------
    def plate(z0, z1):
        p = mp.box(X0, xs, -hw, hw, z0, z1)
        return mp.union(p, mp.cyl((xs, 0, z0), (xs, 0, z1), hw))
    fb_d = 4                                                        # the flanges' M4 bolts (C1 = C2 = 4.5)
    base = plate(-tb, 0.0)
    mp.cut(base, mp.cyl((xs, 0, -tb - 1), (xs, 0, 1), s['D1'] / 2 + FIT))
    _holes(base, _ring(4, s['W'] / 2, c, 45), fb_d / 2 + TAP, -tb - 1, 1)
    mount3, mount4 = MOUNT[3], MOUNT[4]
    bolts4 = _ring(4, mount4['pcd'] / 2, m4, 45)
    _holes(base, [m4], 6.0, -tb - 1, 1)
    _holes(base, bolts4, mount4['bolt'] / 2 + TAP, -tb - 1, 1)
    _part(arm, 'Base plate', base, 'al')
    top = plate(H, H + tt)
    mp.cut(top, mp.cyl((xs, 0, H - 1), (xs, 0, H + tt + 1), s['D2'] / 2 + FIT))
    _holes(top, _ring(4, s['W2'] / 2, c, 45), fb_d / 2 + TAP, H - 1, H + tt + 1)
    bolts3 = _ring(4, mount3['pcd'] / 2, m3, 45)
    _holes(top, [m3], 6.0, H - 1, H + tt + 1)
    _holes(top, bolts3, mount3['bolt'] / 2 + TAP, H - 1, H + tt + 1)
    _part(arm, 'Top plate', top, 'al')
    _part(arm, 'Side wall left', mp.box(X0, xs, hw - wall, hw, 0.0, H), 'al')
    _part(arm, 'Side wall right', mp.box(X0, xs, -hw, -hw + wall, 0.0, H), 'al')

    # ---- the nuts' flanges (each the housing of its rotor's support bearing), bolted to the plates -------------
    r_house = s['F2'] / 2 + BEARING_GAP
    zf = lay['scr']['flange']
    zb = lay['scr']['body']
    fl = mp.union(mp.tube((xs, 0, zb[0]), (xs, 0, zb[1]), s['D2'] / 2, r_house),
                  mp.tube((xs, 0, zf[0]), (xs, 0, zf[1]), s['A2'] / 2, r_house))
    _holes(fl, _ring(4, s['W2'] / 2, c, 45), s['C2'] / 2, zf[0] - 1, zf[1] + 1)
    _part(arm, 'Screw nut SP%d flange' % size, fl, 'steel')
    r_house1 = s['F1'] / 2 + BEARING_GAP
    zf1 = lay['spl']['flange']
    zb1 = lay['spl']['body']
    fl1 = mp.union(mp.tube((xs, 0, zf1[0]), (xs, 0, zf1[1]), s['A1'] / 2, r_house1),
                   mp.tube((xs, 0, zb1[0]), (xs, 0, zb1[1]), s['D1'] / 2, r_house1))
    _holes(fl1, _ring(4, s['W'] / 2, c, 45), s['C1'] / 2, zf1[0] - 1, zf1[1] + 1)
    _part(arm, 'Spline nut SP%d flange' % size, fl1, 'steel')
    L_fb = min(s['E2'] + tt, s['E1'] + tb) - 1.0                   # through the flange, short of the plate's far face
    _kind('Flange bolt M4', mp.cap_screw(fb_d, L_fb, (0, 0, 0), (0, 0, 1)), 'steel',
          [(arm, (x, y, zf[1]), True) for x, y in _ring(4, s['W2'] / 2, c, 45)] +
          [(arm, (x, y, zf1[0]), False) for x, y in _ring(4, s['W'] / 2, c, 45)])

    # ---- the servo motors (fixed in the arm's end), their bolts -------------------------------------------------
    sp3, sp4 = sd.MOTOR34[3], sd.MOTOR34[4]
    h3 = sp3['flange'] / 2
    mot3 = mp.union(mp.box(m3[0] - h3, m3[0] + h3, m3[1] - h3, m3[1] + h3, H - mount3['t'], H),
                    mp.cyl((m3[0], m3[1], H - sp3['L']), (m3[0], m3[1], H - mount3['t']), sp3['D'] / 2))
    _holes(mot3, bolts3, mount3['hole'] / 2, H - mount3['t'] - 1, H + 1)
    _part(arm, 'Servo motor J3', mot3, 'al')
    h4 = sp4['flange'] / 2
    mot4 = mp.union(mp.box(m4[0] - h4, m4[0] + h4, m4[1] - h4, m4[1] + h4, 0.0, mount4['t']),
                    mp.cyl((m4[0], m4[1], mount4['t']), (m4[0], m4[1], sp4['L']), sp4['D'] / 2))
    _holes(mot4, bolts4, mount4['hole'] / 2, -1, mount4['t'] + 1)
    _part(arm, 'Servo motor J4', mot4, 'al')
    _kind('Motor bolt M%d' % mount3['bolt'], mp.cap_screw(mount3['bolt'], mount3['L'], (0, 0, 0), (0, 0, 1)), 'steel',
          [(arm, (x, y, H - mount3['t']), False) for x, y in bolts3])
    _kind('Motor bolt M%d' % mount4['bolt'], mp.cap_screw(mount4['bolt'], mount4['L'], (0, 0, 0), (0, 0, 1)), 'steel',
          [(arm, (x, y, mount4['t']), True) for x, y in bolts4])

    # ---- the screw side: the screw nut's rotor (pilot up), its pulley above it, six screws ---------------------
    p3, p4 = lay['p3'], lay['p4']
    t3 = p3[1] - z_scr_top                                          # the pulley's web above the rotor's top face
    t4 = z_spl_bot - p4[0]                                          # and below the spline rotor's bottom face
    L_ps = max(t3, t4) + 10.0                                       # the screws: 10 mm into the tapped holes
    tap_depth = L_ps - min(t3, t4) + 2.0
    zr = lay['scr']['rotor']
    rot3 = mp.tube((xs, 0, zr[0]), (xs, 0, zr[1]), s['F2'] / 2, s['d'] / 2 + 0.3)
    _holes(rot3, _ring(6, s['W3'] / 2, c), s['m2'] / 2 + TAP, zr[1] - tap_depth, zr[1] + 1)
    _part(g['screw'], 'Screw nut SP%d rotor' % size, rot3, 'steel')
    zn3 = sd.PULLEYS[3][1]
    hub3 = (p3[0], p3[1] - rimw, min(_r_od(zn3) - 0.5, s['F2'] / 2 + 4.0))
    pul3 = _pulley(c, zn3, (p3[1] - rimw, p3[1]),
                   [(p3[0] - 1, z_scr_top, s['F2'] / 2 + FIT), (z_scr_top - 1, p3[1] + 1, s['d'] / 2 + SHAFT_GAP)],
                   hub=hub3, holes=(s['W3'] / 2, s['m2'] / 2 + 0.25, z_scr_top - 1, p3[1] + 1, 6))
    _part(g['screw'], 'Pulley %dT nut J3' % zn3, pul3, 'al')

    # ---- the spline side: the spline nut's rotor (pilot down), its pulley below it, six screws ------------------
    zr1 = lay['spl']['rotor']
    rot4 = mp.tube((xs, 0, zr1[0]), (xs, 0, zr1[1]), s['F1'] / 2, s['d'] / 2 + 0.3)
    _holes(rot4, _ring(6, s['W1'] / 2, c), s['m1'] / 2 + TAP, zr1[0] - 1, zr1[0] + tap_depth)
    _part(g['spline'], 'Spline nut SP%d rotor' % size, rot4, 'steel')
    zn4 = sd.PULLEYS[4][1]
    hub4 = (p4[0] + rimw, p4[1], min(_r_od(zn4) - 0.5, s['F1'] / 2 + 4.0))
    pul4 = _pulley(c, zn4, (p4[0], p4[0] + rimw),
                   [(z_spl_bot, p4[1] + 1, s['F1'] / 2 + FIT), (p4[0] - 1, z_spl_bot + 1, s['d'] / 2 + SHAFT_GAP)],
                   hub=hub4, holes=(s['W1'] / 2, s['m1'] / 2 + 0.25, p4[0] - 1, z_spl_bot + 1, 6))
    _part(g['spline'], 'Pulley %dT nut J4' % zn4, pul4, 'al')

    # the pulley screws, one kind for each thread (M5 on both nuts of the SP20)
    by_thread = {}
    for grp, R, m, zh, down in ((g['screw'], s['W3'] / 2, s['m2'], p3[1], True),
                                (g['spline'], s['W1'] / 2, s['m1'], p4[0], False)):
        by_thread.setdefault(m, []).extend((grp, (x, y, zh), down) for x, y in _ring(6, R, c))
    for m, places in by_thread.items():
        _kind('Pulley screw M%d' % m, mp.cap_screw(m, L_ps, (0, 0, 0), (0, 0, 1)), 'steel', places)

    # ---- the motors' sides: shaft and pulley -------------------------------------------------------------------
    r_ms = 4.0                                                      # the motors' shafts, 8 mm (inertias34())
    _part(g['m3'], 'Motor shaft J3', mp.cyl((m3[0], m3[1], H), (m3[0], m3[1], H + sp3['shaft']), r_ms), 'steel')
    zm3 = sd.PULLEYS[3][0]
    _part(g['m3'], 'Pulley %dT motor J3' % zm3,
          _pulley(m3, zm3, (p3[1] - rimw, p3[1]), [(p3[1] - rimw - 1, p3[1] + 1, r_ms + FIT)]), 'al')
    _part(g['m4'], 'Motor shaft J4', mp.cyl((m4[0], m4[1], 0.0), (m4[0], m4[1], -sp4['shaft']), r_ms), 'steel')
    zm4 = sd.PULLEYS[4][0]
    _part(g['m4'], 'Pulley %dT motor J4' % zm4,
          _pulley(m4, zm4, (p4[0], p4[0] + rimw), [(p4[0] - 1, p4[0] + rimw + 1, r_ms + FIT)]), 'al')

    # ---- the belts: bands sketched round the pulleys' outsides --------------------------------------------------
    for j, (zn, zm), mc, zb_ in ((3, sd.PULLEYS[3][::-1], m3, lay['belt3']), (4, sd.PULLEYS[4][::-1], m4, lay['belt4'])):
        _belt(arm, 'Belt 3MR-%d-%02d J%d' % (bel[j]['stock'], int(sd.BELT_W[j]), j),
              c, _r_od(zn) + BELT_GAP, mc, _r_od(zm) + BELT_GAP, zb_[0], sd.BELT_W[j])

    # ---- the tool's shaft: the ball screw spline's shaft and the tool flange -------------------------------------
    zs = lay['shaft']
    _part(g['shaft'], 'Ball screw spline shaft SP%d' % size, mp.cyl((xs, 0, zs[0]), (xs, 0, zs[1]), s['d'] / 2), 'steel')
    _part(g['shaft'], 'Tool flange', mp.cyl((xs, 0, zs[0] - L['flange_t']), (xs, 0, zs[0]), L['flange_d'] / 2), 'al')

    # ---- joints and Motion Links ---------------------------------------------------------------------------------
    J = {}
    J['j3'] = (_asbuilt(root, 'J3 slide', g['carriage'], arm, xs, 0.0, slide=True), 1)
    J['j4'] = (_asbuilt(root, 'J4 turn', g['shaft'], g['carriage'], xs, 0.0), 1)
    J['screw'] = (_asbuilt(root, 'Screw nut', g['screw'], arm, xs, 0.0), 1)
    J['spline'] = (_asbuilt(root, 'Spline nut', g['spline'], arm, xs, 0.0), 1)
    J['m3'] = (_asbuilt(root, 'J3 motor', g['m3'], arm, m3[0], m3[1]), 1)
    J['m4'] = (_asbuilt(root, 'J4 motor', g['m4'], arm, m4[0], m4[1]), 1)
    q = math.pi / 2
    R3 = sd.PULLEYS[3][1] / sd.PULLEYS[3][0]
    R4 = sd.PULLEYS[4][1] / sd.PULLEYS[4][0]
    links = {'spline': _link(root, 'Spline nut with the shaft', J['j4'][0], J['spline'][0], q, q),
             'm4': _link(root, 'J4 motor by its belt', J['spline'][0], J['m4'][0], q, q * R4),
             'm3': _link(root, 'J3 motor by its belt', J['screw'][0], J['m3'][0], q, q * R3)}
    occs = {'carriage': g['carriage'], 'j4': g['shaft'], 'screw': g['screw'], 'spline': g['spline'],
            'm3': g['m3'], 'm4': g['m4']}
    notes = _calibrate(J, occs, links)
    _last.clear()
    _last.update(joints=J, occ=g, links=links, lead=float(s['l']), size=size, o=o, notes=notes)
    pose(0.0, 0.0)
    try:
        app.activeViewport.fit()
    except Exception:
        pass

    return _message(o, parts_list(root), notes)


def _message(o, pl, notes):
    size = o['size']
    s = sd.SP[size]
    d, sc, mo, bl, br = o['depth'], o['screw'], o['mot'], o['belts'], o['brakes']
    lines = [
        VERSION,
        'HIWIN FB ball screw spline SP%d: lead %d mm, stroke 200 mm.' % (size, s['l']),
        'Shaft at the full stroke: swings at %.1f Hz (at least 20 Hz); fatigue factor %.2f with the tool 100 mm '
        'down and %.2f at the bottom, so below 100 mm the horizontal acceleration is scaled by %.3f (the horizontal '
        'leg in %.3f s).' % (o['cant']['f'], d['n1'], d['n2'], d['scale'], d['t_leg']),
        'Screw: cube-mean force %.1f N, life %s h (f_p %.1f); spline torque %.2f N m (C_T %.1f N m).'
        % (sc['Fm'], _n(sc['Lh']), sc['fp'], o['spline_T'], s['CT']),
        'Motors: J3 %.3f N m peak, %.3f N m RMS, %.0f rev/min; J4 %.3f N m peak, %.3f N m RMS, %.0f rev/min.'
        % (mo['m3']['peak'], mo['m3']['rms'], mo['m3']['nmax'], mo['m4']['peak'], mo['m4']['rms'], mo['m4']['nmax']),
    ]
    for j in (3, 4):
        x = bl[j]
        lines.append('Belt J%d: %dT on the motor, %dT on the nut, 3MR-%d-%02d, centre distance %.2f mm, motor at '
                     'x %.2f, y %+.0f mm; rated %.2f N m against %.2f N m (%.2f times), %d teeth in mesh.'
                     % (j, x['z'][0], x['z'][1], x['stock'], int(sd.BELT_W[j]), x['CD'], x['x_m'], x['y_m'],
                        x['rating'], x['design'], x['ratio'], x['tim']))
    lines.append('Brakes: J3 at least %.3f N m (%.2f N m stops a fall at 500 mm/s in %.1f mm), J4 at least %.3f N m.'
                 % (br['Tb3_min'], br['Tb3'], br['stop'], br['Tb4_min']))
    lines.append('First joint\'s life with this shaft: %s h.' % _n(o['j1']['life']))
    lines.append('Parts: %d components in %d kinds: %s.' % (sum(pl.values()), len(pl),
                                                            ', '.join('%s %d' % kv for kv in sorted(pl.items()))))
    if notes:
        lines.append('Joints: ' + '; '.join(notes) + '.')
    return '\n'.join(lines)


# ---------------------------------------------------------------------------------------------------------------
# the dialog
# ---------------------------------------------------------------------------------------------------------------
class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            item = args.command.commandInputs.itemById('size').selectedItem
            size = int(''.join(ch for ch in item.name if ch.isdigit()))
            msg = build(size=size)
            ui.messageBox(msg, 'ScaraToolJoints')
        except ValueError as e:
            ui.messageBox(str(e) + '\n' + VERSION, 'ScaraToolJoints')
        except Exception:
            ui.messageBox('ScaraToolJoints failed:\n' + traceback.format_exc() + '\n' + VERSION)


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _state.get('standalone'):
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            cmd = args.command
            inputs = cmd.commandInputs
            dd = inputs.addDropDownCommandInput('size', 'HIWIN FB ball screw spline',
                                                adsk.core.DropDownStyles.TextListDropDownStyle)
            for k in sorted(sd.SP):
                dd.listItems.add('SP %d' % k, k == 20, '')
            ex = _Execute()
            cmd.execute.add(ex)
            _handlers.append(ex)
            de = _Destroy()
            cmd.destroy.add(de)
            _handlers.append(de)
        except Exception:
            ui.messageBox('ScaraToolJoints dialog failed:\n' + traceback.format_exc())


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        _state['standalone'] = context is not None
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'SCARA J3 and J4',
                                                       'The ball screw spline at the end of the SCARA\'s second arm')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if context is not None:
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ScaraToolJoints failed to start:\n' + traceback.format_exc())


def stop(context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
