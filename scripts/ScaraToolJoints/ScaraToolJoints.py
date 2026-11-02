# =============================================================================
# ScaraToolJoints.py  -  Chapter 37 of "Mechanical Design in Autodesk Fusion with Claude": the SCARA's J3 and J4 at
# the second arm's end. A HIWIN SP20 FB ball screw spline: one shaft with a screw and a spline, the screw nut on the
# arm's top plate and the spline nut under its base plate, each turning in its own support bearing; a toothed pulley
# on each nut's rotor; two Gates GT3 3 mm belts to the J3 and J4 servo motors inside the arm's end, side by side.
# The shaft slides (J3) and turns (J4). Nuts, pulleys and motors are drawn at concept level from the catalogues'
# dimensions (HIWIN Tables 9.2 and 9.3; Gates' pitch diameters). The sizing is lib/scara_design.tool_joints().
# Written from PROMPT.txt beside it; checked by checks/scara_tool_check.py.
#
#   INPUTS        the dialog: the ball screw spline's size (16, 20, 25 or 32)
#   DERIVED       scara_design.tool_joints(): the shaft at the full stroke, the depth rule, the screw's life, the
#                 motors, the belts by Gates' procedure, the brakes; nut_layout(): every height
#   HARD RULES    build() refuses a size whose shaft swings under 20 Hz at the full stroke, whose fatigue factor
#                 at 100 mm is under 1.5, or that costs the first joint its life of 20 000 h
#   CONVENTIONS   the second arm's end: x along the arm from J2 (the shaft at x = 250), y across, z up from the
#                 base plate's top face; J3 down is positive; J4 about +z
#   ACCEPTANCE    the message; no interference at three poses; the shaft engaged at the full stroke; the belts'
#                 centre distances as the stock lengths give them
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

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import scara_design as sd       # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(sd)
importlib.reload(mp)

VERSION = 'ScaraToolJoints 1.0 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
SHAFT_D = {3: 14.0, 4: 8.0}                  # the motors' shafts (assumed)
HEAD = 4.0                                   # the nuts' flange bolts are M4: head height


def design(size=20):
    o = sd.tool_joints(size=int(size))
    o['size'] = int(size)
    return o


def refuse(o):
    size = o['size']
    c = o['sizes'][size]
    d = sd.depth_rule(size)
    short = []
    if c['f'] < 20.0:
        short.append('its shaft swings at %.1f Hz at the full stroke against 20' % c['f'])
    if d['n1'] < d['n_min']:
        short.append('its fatigue factor at %.0f mm is %.2f against %.1f' % (d['s_full'], d['n1'], d['n_min']))
    if size > 20:
        s25 = o['sp25']
        life = s25['life'] if size == 25 else None
        short.append('its mass at the tool costs the first joint its life%s' % (' (%.0f h against 20 000)' % life if life else ''))
    if short:
        raise ValueError('The SP%d falls short: %s.' % (size, '; '.join(short)))


# ---------- geometry (world mm) ----------
def zc(x, y, z0, z1, r, r_in=0.0):
    return mp.tube((x, y, z0), (x, y, z1), r, r_in) if r_in > 0 else mp.cyl((x, y, z0), (x, y, z1), r)


def part(parent, name, tb, material='Steel'):
    o = mp.new_occ(parent.component, name)
    mp.add_body(o, tb, name, material)
    return o


def circle_pts(xc, yc, r, n, phase=0.0):
    return [(xc + r * math.cos(phase + 2 * pi_ * k / n), yc + r * math.sin(phase + 2 * pi_ * k / n)) for k in range(n)]


pi_ = math.pi


def belt_band(parent, name, c1, r1, c2, r2, z0, z1):
    """An open belt round two pulleys in plan: the band between the loops at r and r + 1.2 mm round each centre,
    the straight spans on the external tangents; sketched and extruded in a component of its own."""
    o = mp.new_occ(parent.component, name)
    comp = o.component
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z0 * CM))
    sk = comp.sketches.add(comp.constructionPlanes.add(ci))
    sk.isVisible = False
    for rr1, rr2 in ((r1, r2), (r1 + 1.2, r2 + 1.2)):
        dx, dy = c2[0] - c1[0], c2[1] - c1[1]
        D = math.hypot(dx, dy)
        th = math.atan2(dy, dx)
        a = math.acos((rr1 - rr2) / D)          # the external tangents leave each circle at th +- a
        pts = []
        for sgn in (1, -1):
            pts.append(((c1[0] + rr1 * math.cos(th + sgn * a), c1[1] + rr1 * math.sin(th + sgn * a)),
                        (c2[0] + rr2 * math.cos(th + sgn * a), c2[1] + rr2 * math.sin(th + sgn * a))))
        S = lambda p: sk.modelToSketchSpace(P3(p[0] * CM, p[1] * CM, z0 * CM))
        lines = sk.sketchCurves.sketchLines
        arcs = sk.sketchCurves.sketchArcs
        l1 = lines.addByTwoPoints(S(pts[0][0]), S(pts[0][1]))
        l2 = lines.addByTwoPoints(S(pts[1][0]), S(pts[1][1]))
        # the arc round the second pulley from the first tangent's end to the second's, the short way outward
        mid2 = (c2[0] + rr2 * math.cos(th), c2[1] + rr2 * math.sin(th))
        mid1 = (c1[0] - rr1 * math.cos(th), c1[1] - rr1 * math.sin(th))
        arcs.addByThreePoints(l1.endSketchPoint, S(mid2), l2.endSketchPoint)
        arcs.addByThreePoints(l1.startSketchPoint, S(mid1), l2.startSketchPoint)
    prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, VI((z1 - z0) * CM))
    body = ext.add(inp).bodies.item(0)
    body.name = name
    mp.set_material(body, 'Rubber')
    return o


def build_parts(root, o):
    """Every part, in five groups that move apart: the arm's end (grounded), the screw side, the spline side, each
    motor's side, and the tool's shaft with its flange (on a body-less carriage that slides)."""
    size = o['size']
    s = sd.SP[size]
    L = sd.LAY37
    lay = o['lay']
    xs = L['x_s']
    b3, b4 = o['belts'][3], o['belts'][4]
    m3c, m4c = (b3['x_m'], b3['y_m']), (b4['x_m'], b4['y_m'])
    occ = {}
    for nm in ('Arm end', 'Screw side', 'Spline side', 'J3 motor side', 'J4 motor side', 'J3 carriage', 'Tool shaft'):
        occ[nm] = mp.new_occ(root, nm, (0.0, 0.0, 0.0))
    for nm in occ:
        if nm != 'Arm end':
            occ[nm].isGroundToParent = False
    st = occ['Arm end']
    x0, x1 = 40.0, 290.0
    hw = L['half_w']
    # the arm's end: base plate, top plate, two walls
    bp = mp.box(x0, x1, -hw, hw, -L['t_base'], 0.0)
    tp = mp.box(x0, x1, -hw, hw, L['H'], L['H'] + L['t_top'])
    for plate, z0, z1 in ((bp, -L['t_base'], 0.0), (tp, L['H'], L['H'] + L['t_top'])):
        mp.cut(plate, zc(xs, 0, z0 - 1, z1 + 1, s['D1'] / 2 + 0.1))
    for (mx, my), j in ((m3c, 3), (m4c, 4)):
        plate = tp if j == 3 else bp
        z0, z1 = (L['H'], L['H'] + L['t_top']) if j == 3 else (-L['t_base'], 0.0)
        mp.cut(plate, zc(mx, my, z0 - 1, z1 + 1, SHAFT_D[j] / 2 + 1.0))
        F = sd.MOTOR34[j]['flange']
        bd = 5.0 if j == 3 else 4.0
        for px, py in circle_pts(mx, my, (F / 2 - bd) * math.sqrt(2), 4, pi_ / 4):
            mp.cut(plate, zc(px, py, z0 - 1, z1 + 1, bd / 2 + 0.25))
    # the nuts' flange bolts: four M4 on W (spline, into the base plate) and W2 (screw, into the top plate)
    for (xx, yy) in circle_pts(xs, 0, s['W'] / 2, 4, pi_ / 4):
        mp.cut(bp, zc(xx, yy, -L['t_base'] - 1, -L['t_base'] + 8.0, 2.0))
    for (xx, yy) in circle_pts(xs, 0, s['W2'] / 2, 4, pi_ / 4):
        mp.cut(tp, zc(xx, yy, L['H'] + L['t_top'] - 8.0, L['H'] + L['t_top'] + 1, 2.0))
    part(st, 'Base plate', bp, 'Aluminum')
    part(st, 'Top plate', tp, 'Aluminum')
    for sg in (1, -1):
        part(st, 'Side wall %s' % ('left' if sg > 0 else 'right'),
             mp.box(x0, x1, sg * (hw - L['wall']), sg * hw, 0.0, L['H']) if sg > 0 else
             mp.box(x0, x1, -hw, -(hw - L['wall']), 0.0, L['H']), 'Aluminum')
    # the nuts' stators: flange and body, bored for the rotor
    spl, scr = lay['spl'], lay['scr']
    sn = zc(xs, 0, spl['flange'][0], spl['flange'][1], s['A1'] / 2, s['F1'] / 2 + 0.5)
    mp.union(sn, zc(xs, 0, spl['body'][0] - 0.01, spl['body'][1], s['D1'] / 2, s['F1'] / 2 + 0.5))
    for (xx, yy) in circle_pts(xs, 0, s['W'] / 2, 4, pi_ / 4):
        mp.cut(sn, zc(xx, yy, spl['flange'][0] - 1, spl['flange'][1] + 1, s['C1'] / 2))
    part(st, 'Spline nut SP%d flange' % size, sn)
    cn = zc(xs, 0, scr['flange'][0], scr['flange'][1], s['A2'] / 2, s['F2'] / 2 + 0.5)
    mp.union(cn, zc(xs, 0, scr['body'][0], scr['body'][1] + 0.01, s['D2'] / 2, s['F2'] / 2 + 0.5))
    for (xx, yy) in circle_pts(xs, 0, s['W2'] / 2, 4, pi_ / 4):
        mp.cut(cn, zc(xx, yy, scr['flange'][0] - 1, scr['flange'][1] + 1, s['C2'] / 2))
    part(st, 'Screw nut SP%d flange' % size, cn)
    for k, (xx, yy) in enumerate(circle_pts(xs, 0, s['W'] / 2, 4, pi_ / 4)):
        part(st, 'Flange bolt M4 %d' % (k + 1), mp.cap_screw(4.0, s['E1'] + 8.0, (xx, yy, spl['flange'][0]), (0, 0, 1)))
    for k, (xx, yy) in enumerate(circle_pts(xs, 0, s['W2'] / 2, 4, pi_ / 4)):
        part(st, 'Flange bolt M4 %d' % (k + 5), mp.cap_screw(4.0, s['E2'] + 8.0, (xx, yy, scr['flange'][1]), (0, 0, -1)))
    # the motors inside the box: J3 hanging from the top plate, J4 standing on the base plate
    for j, (mx, my) in ((3, m3c), (4, m4c)):
        m = sd.MOTOR34[j]
        F = m['flange']
        mo = mp.new_occ(st.component, 'Servo motor J%d' % j)
        if j == 3:
            zf0, zf1 = L['H'] - 8.0, L['H']
            zb0, zb1 = zf0 - m['L'], zf0
        else:
            zf0, zf1 = 0.0, 8.0
            zb0, zb1 = zf1, zf1 + m['L']
        fl = mp.box(mx - F / 2, mx + F / 2, my - F / 2, my + F / 2, zf0, zf1)
        bd = 5.0 if j == 3 else 4.0
        for px, py in circle_pts(mx, my, (F / 2 - bd) * math.sqrt(2), 4, pi_ / 4):
            mp.cut(fl, zc(px, py, zf0 - 1, zf1 + 1, bd / 2))
        mp.cut(fl, zc(mx, my, zf0 - 1, zf1 + 1, SHAFT_D[j] / 2 + 1.0))
        mp.add_body(mo, fl, 'Flange', 'Aluminum')
        split = 0.7
        if j == 3:
            mp.add_body(mo, zc(mx, my, zb1 - split * m['L'], zb1, m['D'] / 2), 'Motor', 'Aluminum')
            mp.add_body(mo, zc(mx, my, zb0, zb1 - split * m['L'], m['D'] / 2), 'Brake', 'Aluminum')
        else:
            mp.add_body(mo, zc(mx, my, zb0, zb0 + split * m['L'], m['D'] / 2), 'Motor', 'Aluminum')
            mp.add_body(mo, zc(mx, my, zb0 + split * m['L'], zb1, m['D'] / 2), 'Brake', 'Aluminum')
        for k, (px, py) in enumerate(circle_pts(mx, my, (F / 2 - bd) * math.sqrt(2), 4, pi_ / 4)):
            if j == 3:
                part(st, 'Motor bolt M5 %d' % (k + 1), mp.cap_screw(5.0, L['t_top'] + 6.0, (px, py, L['H'] + L['t_top']), (0, 0, -1)))
            else:
                part(st, 'Motor bolt M4 %d' % (k + 1), mp.cap_screw(4.0, L['t_base'] + 6.0, (px, py, -L['t_base']), (0, 0, 1)))
    # the belts, round the pulleys' outsides
    D3, d3 = b3['D'], b3['d']
    D4, d4 = b4['D'], b4['d']
    belt_band(st, 'Belt 3MR-%d-09 J3' % b3['stock'], (xs, 0.0), D3 / 2 - 0.381 + 0.1, m3c, d3 / 2 - 0.381 + 0.1,
              lay['belt3'][0], lay['belt3'][1])
    belt_band(st, 'Belt 3MR-%d-09 J4' % b4['stock'], (xs, 0.0), D4 / 2 - 0.381 + 0.1, m4c, d4 / 2 - 0.381 + 0.1,
              lay['belt4'][0], lay['belt4'][1])
    # the screw side: the screw nut's rotor, its pulley, six M5 screws
    sc = occ['Screw side']
    ro = zc(xs, 0, scr['rotor'][0], scr['rotor'][1], s['F2'] / 2, s['d'] / 2 + 0.3)
    p3a, p3b = lay['p3']
    for (xx, yy) in circle_pts(xs, 0, s['W3'] / 2, 6, pi_ / 6):
        mp.cut(ro, zc(xx, yy, scr['rotor'][1] - 8.0, scr['rotor'][1] + 1, 2.5))
    part(sc, 'Screw nut SP%d rotor' % size, ro)
    pu = zc(xs, 0, p3a, p3b, D3 / 2 - 0.381, s['d'] / 2 + 1.0)
    mp.cut(pu, zc(xs, 0, p3a - 1, scr['rotor'][1], s['F2'] / 2 + 0.1))
    for (xx, yy) in circle_pts(xs, 0, s['W3'] / 2, 6, pi_ / 6):
        mp.cut(pu, zc(xx, yy, p3a - 1, p3b + 1, 2.75))
    part(sc, 'Pulley %dT nut J3' % b3['z'][1], pu, 'Aluminum')
    for k, (xx, yy) in enumerate(circle_pts(xs, 0, s['W3'] / 2, 6, pi_ / 6)):
        part(sc, 'Pulley screw M5 %d' % (k + 1), mp.cap_screw(5.0, p3b - scr['rotor'][1] + 7.5, (xx, yy, p3b), (0, 0, -1)))
    # the spline side
    sp = occ['Spline side']
    ri = zc(xs, 0, spl['rotor'][0], spl['rotor'][1], s['F1'] / 2, s['d'] / 2 + 0.3)
    p4a, p4b = lay['p4']
    for (xx, yy) in circle_pts(xs, 0, s['W1'] / 2, 6, pi_ / 6):
        mp.cut(ri, zc(xx, yy, spl['rotor'][0] - 1, spl['rotor'][0] + 8.0, 2.5))
    part(sp, 'Spline nut SP%d rotor' % size, ri)
    pu = zc(xs, 0, p4a, p4b, D4 / 2 - 0.381, s['d'] / 2 + 1.0)
    mp.cut(pu, zc(xs, 0, spl['rotor'][0], p4b + 1, s['F1'] / 2 + 0.1))
    for (xx, yy) in circle_pts(xs, 0, s['W1'] / 2, 6, pi_ / 6):
        mp.cut(pu, zc(xx, yy, p4a - 1, p4b + 1, 2.75))
    part(sp, 'Pulley %dT nut J4' % b4['z'][1], pu, 'Aluminum')
    for k, (xx, yy) in enumerate(circle_pts(xs, 0, s['W1'] / 2, 6, pi_ / 6)):
        part(sp, 'Pulley screw M5 %d' % (k + 7), mp.cap_screw(5.0, spl['rotor'][0] - p4a + 7.5, (xx, yy, p4a), (0, 0, 1)))
    # each motor's side: its shaft and pulley
    for j, (mx, my), b_, zz in ((3, m3c, b3, lay['belt3']), (4, m4c, b4, lay['belt4'])):
        ms = occ['J%d motor side' % j]
        if j == 3:
            z0, z1 = L['H'], L['H'] + sd.MOTOR34[3]['shaft']
        else:
            z0, z1 = -sd.MOTOR34[4]['shaft'], 0.0
        part(ms, 'Motor shaft J%d' % j, zc(mx, my, z0, z1, SHAFT_D[j] / 2))
        pz0, pz1 = zz[0] - 1.5, zz[1] + 1.5
        part(ms, 'Pulley %dT motor J%d' % (b_['z'][0], j), zc(mx, my, pz0, pz1, b_['d'] / 2 - 0.381, SHAFT_D[j] / 2), 'Aluminum')
    # the tool's shaft with three spline grooves, and the tool flange
    ts = occ['Tool shaft']
    z_lo, z_hi = lay['shaft']
    shft = zc(xs, 0, z_lo, z_hi, s['d'] / 2)
    for k in range(3):
        a = 2 * pi_ * k / 3 + pi_ / 2
        mp.cut(shft, zc(xs + (s['d'] / 2 + 0.6) * math.cos(a), (s['d'] / 2 + 0.6) * math.sin(a), z_lo + 15.0, z_hi - 15.0, 1.6))
    part(ts, 'Ball screw spline shaft SP%d' % size, shft)
    part(ts, 'Tool flange', zc(xs, 0, z_lo - L['flange_t'], z_lo, L['flange_d'] / 2), 'Aluminum')
    return occ


# ---------- joints ----------
def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_z(root, moving, fixed, point, name):
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.ZAxisJointDirection, Jd.XAxisJointDirection, Jd.YAxisJointDirection):
        sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
        sk.isVisible = False
        pt = sk.sketchPoints.add(P3(*[v * CM for v in point])).createForAssemblyContext(moving)
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.05
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][k] * r0[c][k] for k in range(3)) for c in range(3)] for a in range(3)]
        wv = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        if abs(wv[2]) > 0.02 and abs(wv[0]) < 0.02 and abs(wv[1]) < 0.02:
            j.name = name
            return j, (1 if wv[2] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about z' % moving.name)


def slider_z(root, moving, fixed, point, name):
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.ZAxisJointDirection, Jd.XAxisJointDirection, Jd.YAxisJointDirection):
        sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
        sk.isVisible = False
        pt = sk.sketchPoints.add(P3(*[v * CM for v in point])).createForAssemblyContext(moving)
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsSliderJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        t0 = moving.transform2.translation
        j.jointMotion.slideValue = 1.0
        t1 = moving.transform2.translation
        j.jointMotion.slideValue = 0.0
        dv = (t1.x - t0.x, t1.y - t0.y, t1.z - t0.z)
        if abs(dv[2]) > 0.5 and abs(dv[0]) < 0.01 and abs(dv[1]) < 0.01:
            j.name = name
            return j, (1 if dv[2] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction slides %s along z' % moving.name)


def link(root, ja, sa, jb, sb, turns_a, name, same=True):
    """A Motion Link: ja turns turns_a times for one turn of jb; the two turn the same way in the world if `same`."""
    li = root.motionLinks.createInput(ja, jb)
    li.valueOne = VS('%g deg' % (360 * turns_a))
    li.valueTwo = VS('360 deg')
    li.isReversed = (sa * sb < 0) if same else (sa * sb > 0)
    root.motionLinks.add(li).name = name


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        wds = o.component.name.split()
        if wds[-1].isdigit():
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


_last = {}


def pose(j3, j4):
    """J3 down by j3 mm, J4 turned j4 deg about +z: the carriage slides, the shaft turns, the spline nut and its motor
    follow through their links; the screw nut turns with the shaft less j3 / lead turns (the screw is right-handed:
    a nut turned clockwise from above drives the shaft down, HIWIN Table 9.4), and its motor follows."""
    J = _last['joints']
    lead = _last['lead']
    js, ss = J['J3']
    js.jointMotion.slideValue = -ss * j3 * CM
    jr, sr = J['J4']
    jr.jointMotion.rotationValue = sr * math.radians(j4)
    jn, sn = J['screw']
    jn.jointMotion.rotationValue = sn * math.radians(j4 - 360.0 * j3 / lead)
    adsk.doEvents()


def build(size=20):
    o = design(size)
    refuse(o)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    occ = build_parts(root, o)
    occ['Arm end'].isGrounded = True
    xs = sd.LAY37['x_s']
    b3, b4 = o['belts'][3], o['belts'][4]
    st = occ['Arm end']
    J = {}
    J['J3'] = slider_z(root, occ['J3 carriage'], st, (xs, 0, 0), 'J3 slides')
    J['J4'] = revolute_z(root, occ['Tool shaft'], occ['J3 carriage'], (xs, 0, 0), 'J4 turns')
    J['screw'] = revolute_z(root, occ['Screw side'], st, (xs, 0, 0), 'Screw nut turns')
    J['spline'] = revolute_z(root, occ['Spline side'], st, (xs, 0, 0), 'Spline nut turns')
    J['m3'] = revolute_z(root, occ['J3 motor side'], st, (b3['x_m'], b3['y_m'], 0), 'J3 motor turns')
    J['m4'] = revolute_z(root, occ['J4 motor side'], st, (b4['x_m'], b4['y_m'], 0), 'J4 motor turns')
    link(root, J['spline'][0], J['spline'][1], J['J4'][0], J['J4'][1], 1, 'Spline nut with the shaft')
    link(root, J['m4'][0], J['m4'][1], J['spline'][0], J['spline'][1], o['mot']['R4'], 'J4 belt')
    link(root, J['m3'][0], J['m3'][1], J['screw'][0], J['screw'][1], o['mot']['R3'], 'J3 belt')
    _last['joints'] = J
    _last['lead'] = sd.SP[o['size']]['l']
    _last['occ'] = occ
    for nm, col in (('Arm end', 'Paint - Enamel Glossy (Grey)'), ('Screw side', 'Paint - Enamel Glossy (Yellow)'),
                    ('Spline side', 'Paint - Enamel Glossy (Yellow)'), ('Tool shaft', 'Paint - Enamel Glossy (Blue)'),
                    ('J3 motor side', 'Paint - Enamel Glossy (Yellow)'), ('J4 motor side', 'Paint - Enamel Glossy (Yellow)')):
        mp.colour(des, occ[nm], col)
    app.activeViewport.fit()
    c, d, sc, m = o['cant'], o['depth'], o['screw'], o['mot']
    pl = parts_list(root)
    return ('SCARA J3 and J4 built.\nSP%d: the shaft swings at %.1f Hz at the full stroke; fatigue factor %.2f at %.0f mm, '
            '%.2f at 200 mm, so the horizontal acceleration is scaled to %.3f there.\nScrew: cube mean %.1f N, life %.0f h. '
            'J3 motor %.3f N m peak, %.3f RMS, %.0f rev/min; J4 motor %.3f N m peak, %.3f RMS.\nBelts 3MR-%d and 3MR-%d, '
            'centre distances %.2f and %.2f mm.\n\nParts: %d components; %s.\n\n[%s]'
            % (o['size'], c['f'], d['n1'], d['s_full'], d['n2'], d['scale'], sc['Fm'], sc['Lh'], m['m3']['peak'],
               m['m3']['rms'], m['m3']['nmax'], m['m4']['peak'], m['m4']['rms'], b3['stock'], b4['stock'], b3['CD'],
               b4['CD'], sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('size', 'Ball screw spline size (SP)', 20.0)]


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
        cid = 'ScaraToolJointsCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'SCARA J3 and J4', 'The ball screw spline of Chapter 37')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ScaraToolJoints could not start:\n' + traceback.format_exc())
