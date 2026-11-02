# =============================================================================
# FlapActuator.py  -  Chapter 33 of "Mechanical Design in Autodesk Fusion with Claude": the electromechanical
# actuator of the flap actuation system of Chapter 32. A ball screw turned through a spur gear, an idler and a
# pinion by a motor unit (brushless motor, planetary gearhead and spring-applied brake, bought to a
# specification) drives a ball nut held in a rod; the rod ends in a rod end on the flap's horn. The screw turns in
# a back-to-back pair of angular contact thrust bearings in an aluminium housing that hangs from the rear spar on
# two trunnions; a second pinion on the screw's gear drives an absolute encoder. The sizing is
# lib/flap_design.actuator_design(); the parts lib/mdfc_parts.py; the ball groove the sweep of Chapter 21
# (BallScrewAxis.py). Written from PROMPT.txt beside it; checked by checks/flap_actuator_check.py.
#
#   INPUTS        the dialog: the screw's operating factor f_p, the gearhead's ratio
#   DERIVED       flap_design.actuator_design(): the nut's life, the torques, the gears, the motor and the brake;
#                 the positions of flap_design.LAY
#   HARD RULES    build() refuses a nut that does not last 60 000 strokes, a nut whose static rating falls under
#                 the ultimate force, a gear under 1.5 against yield at the ultimate or in fatigue at the limit, a
#                 rod that buckles under twice the ultimate force
#   CONVENTIONS   the actuator along +Y from the gimbal's axis (y = 0) to the rod end, Z up, X along the span;
#                 six subassemblies of real parts (Housing, Screw, Rod, Idler, Pinion, Encoder pinion), every part
#                 a component; As-built Joints and Motion Links: the screw turns, the rod slides 10 mm a turn
#   ACCEPTANCE    the message; no interference retracted, part way and extended; the rod end's centre 378.96 mm
#                 from the gimbal retracted and 461.04 mm extended; the parts list
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
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
import flap_design as fd        # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(fd)
importlib.reload(mp)


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, os.path.join(os.path.dirname(HERE), rel))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


BSA = _load('mdfc_ballscrewaxis', os.path.join('BallScrewAxis', 'BallScrewAxis.py'))

VERSION = 'FlapActuator 2.1 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
L = fd.LAY
G = L['gear']
R_G = 0.52 * 6.35                       # the groove's radius, 0.52 of the ball (Chapter 21)
D_M = 26.91 + 2 * R_G                   # the balls' pitch diameter, so that the groove's root is HIWIN's d_k
SENSOR = (60.0 * math.cos(math.radians(L['sensor_angle'])), 60.0 * math.sin(math.radians(L['sensor_angle'])))


def design(fp=3.0, i_gh=24.0):
    o = fd.actuator_design(fp=fp)
    if abs(i_gh - o['i_gh']) > 1e-9:
        o['i_gh'] = i_gh
        o['n_motor'] = o['n_pin'] * i_gh
        o['T_motor'] = o['T_pin'] / i_gh / o['eta_gh']
        o['P_motor'] = o['T_motor'] / 1000.0 * o['n_motor'] * 2 * math.pi / 60.0
        o['T_brake_need'] = -o['T_back'] / o['gears']['ratio'] / i_gh
        o['T_brake'] = 2.0 * o['T_brake_need']
    return o


def refuse(o):
    n, g = o['nut'], o['gears']
    short = []
    if n['life'] < 1.0:
        short.append('the nut %s lasts %.2f of the %d strokes at f_p %.2f'
                     % (n['name'], n['life'], fd.STROKES, o['fp']))
    if n['static'] < 1.0:
        short.append('the nut\'s static rating is %.2f of the ultimate force' % n['static'])
    if g['n_static'] < 1.5 or min(g['nf_P'], g['nf_G'], g['nf_I']) < 1.5:
        short.append('the gears keep %.2f at the ultimate and %.2f in fatigue'
                     % (g['n_static'], min(g['nf_P'], g['nf_G'], g['nf_I'])))
    if o['P_cr'] < 2 * o['act']['F_ult']:
        short.append('the rod buckles at %.0f N' % o['P_cr'])
    if o['n_motor'] > 6000.0:
        short.append('the motor would turn at %.0f rev/min' % o['n_motor'])
    if short:
        raise ValueError('The actuator falls short: %s.' % '; '.join(short))


# ---------- geometry helpers (world mm; the actuator's axis is +Y) ----------
def along_y(x, z, y0, y1, r, r_in=0.0):
    return mp.tube((x, y0, z), (x, y1, z), r, r_in) if r_in > 0 else mp.cyl((x, y0, z), (x, y1, z), r)


def along_x(y, z, x0, x1, r):
    return mp.cyl((x0, y, z), (x1, y, z), r)


def mesh_phase(theta_a, z_a, beta, z_b):
    """Gear b's tooth angle (deg, in the xz plane from +x towards +z) so that it meshes with gear a, whose teeth are
    at theta_a + k 360/z_a, gear b lying in the direction beta from gear a."""
    pa, pb = 360.0 / z_a, 360.0 / z_b
    e = (theta_a - beta) % pa
    if e > pa / 2:
        e -= pa
    return beta + 180.0 + pb / 2 - e * z_a / z_b


def groove_y(occ, body, y0, y1):
    """The ball groove cut into the screw from y0 to y1: Chapter 21's section, swept along +y with one turn per lead.
    The section's u is taken along +z and v along +x, so that its angle turns about +y."""
    comp = occ.component
    P = float(fd.HIWIN[fd.NUT][1])
    section = BSA.groove_section(D_M / 2, R_G, P, 2 * math.pi * y0 / P)
    sk = comp.sketches.add(mp.xz_plane(comp, y0))
    sk.isComputeDeferred = True
    pts = [sk.modelToSketchSpace(P3(v * CM, y0 * CM, u * CM)) for u, v in section]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    ps = comp.sketches.add(comp.xYConstructionPlane)
    ps.isVisible = False
    axis = ps.sketchCurves.sketchLines.addByTwoPoints(P3(0, y0 * CM, 0), P3(0, y1 * CM, 0))
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk.profiles.item(0), comp.features.createPath(axis, False),
                         adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.twistAngle = VI(2 * math.pi * (y1 - y0) / P)
    inp.participantBodies = [body]
    return sw.add(inp)


def part(parent, name, tb, material='Steel', t=(0.0, 0.0, 0.0)):
    o = mp.new_occ(parent.component, name, t)
    mp.add_body(o, tb, name, material)
    return o


def gear(parent, name, z, x, zc, y0, y1, face, bore, phase_deg, hub=None, web=None):
    """A spur gear of Chapter 12 on an axis parallel to y through (x, zc): its root disc from y0 to y1, the teeth on
    `face`, a bore; its component's origin on its axis, so the teeth are patterned about the component's y. With a
    web (r_rim, w), the disc is a rim from r_rim to the root, a web w thick in the middle and the hub."""
    m = G['m']
    rf = m * z / 2.0 - 1.25 * m
    if web:
        r_rim, w = web
        yc = 0.5 * (y0 + y1)
        blank = along_y(x, zc, y0, y1, rf, r_rim)
        mp.union(blank, along_y(x, zc, yc - w / 2, yc + w / 2, r_rim + 0.01, hub[2] - 0.01))
    else:
        blank = along_y(x, zc, y0, y1, rf, bore)
    if hub:
        mp.union(blank, along_y(x, zc, hub[0], hub[1], hub[2], bore))
    o = mp.new_occ(parent.component, name, (x, 0.0, zc))
    body = mp.add_body(o, blank, name, 'Steel')
    mp.spur_teeth(o, body, m, z, 20.0, G['backlash'], math.radians(phase_deg), face[0], face[1])
    return o


# ---------- the subassemblies ----------
def housing(root):
    asm = mp.new_occ(root, 'Housing')
    xs, zs = SENSOR
    lz, lx = L['lobe_z'], L['lobe_x']
    # the housing: the nose round the gear, the lobe down to the motor's pinion, the encoder's boss, the bearings'
    # boss, the tube, the gland, and the trunnions' bosses
    h = along_y(0, 0, L['nose'][0], L['nose'][1], L['r_nose'])
    mp.union(h, mp.box(-lx, lx, L['nose'][0], L['lobe_s'], lz, 0.0),
             along_y(xs, zs, L['nose'][0], L['lobe_s'], 26.0),
             along_y(0, 0, L['boss'][0], L['boss'][1], L['r_boss']),
             along_y(0, 0, L['tube'][0], L['tube'][1], L['r_tube']),
             along_y(0, 0, L['gland'][0], L['gland'][1], L['r_tube']))
    for s in (1, -1):
        mp.union(h, along_x(0, 0, s * (L['r_nose'] - 2.0), s * L['boss_x'][1], L['r_tboss']))
    c0 = L['nose'][0] - 0.01
    mp.cut(h, along_y(0, 0, c0, L['shoulder_ring'][0], L['r_cavity']),
           mp.box(-L['cav_x'], L['cav_x'], c0, L['shoulder_ring'][0], L['cav_z'], -20.0),
           along_y(xs, zs, c0, L['shoulder_ring'][0], 23.0),
           along_y(0, 0, L['shoulder_ring'][0] - 0.01, L['shoulder_ring'][1] + 0.01, L['r_shoulder_in']),
           along_y(0, 0, L['bearings'][0][0], L['bearings'][1][1], fd.BSD['D'] / 2),
           along_y(0, 0, L['ring_nut'][0], L['ring_nut'][1], L['r_ring']),
           along_y(0, 0, L['tube'][0] - 0.01, L['tube'][1], L['bore'] / 2),
           mp.box(-L['slot_w'] / 2, L['slot_w'] / 2, L['tube'][0] - 0.01, L['tube'][1], 0.0, L['slot_top']),
           along_y(0, 0, L['gland'][0] - 0.01, L['gland'][1] + 0.01, L['r_rod'] + 0.2),
           along_y(0, 0, 252.0, 258.0, 25.0),                                    # the rod seal's groove
           along_y(0, L['z_I'], L['shoulder_ring'][0] - 0.01, L['lobe_s'], 6.0),  # the idler's pin, pressed
           along_y(0, L['z_P'], L['shoulder_ring'][0] - 0.01, L['lobe_s'] + 0.01, 7.0),
           along_y(xs, zs, L['shoulder_ring'][0] - 0.01, L['lobe_s'] + 0.01, 5.0))
    for s in (1, -1):
        mp.cut(h, along_x(0, 0, s * (L['trunnion_x'][0] - 0.1), s * (L['boss_x'][1] + 0.01), L['r_trunnion']))
    for x, z in COVER_SCREWS:
        mp.cut(h, along_y(x, z, L['nose'][0] - 0.01, 10.0, 2.55))
    for x, z in MOTOR_SCREWS:
        mp.cut(h, along_y(x, z, 30.0, L['lobe_s'] + 0.01, 2.55))
    parts = [part(asm, 'Actuator housing', h, 'Aluminum')]
    # the cover over the gears, with six M5 cap screws
    cv = along_y(0, 0, L['cover'][0], L['cover'][1], L['r_nose'])
    mp.union(cv, mp.box(-lx, lx, L['cover'][0], L['cover'][1], lz, 0.0),
             along_y(xs, zs, L['cover'][0], L['cover'][1], 26.0))
    mp.cut(cv, along_y(0, L['z_I'], L['idler_pin'][0] - 0.01, L['cover'][1] + 0.01, 6.0))
    for s in (1, -1):                                                # clear of the trunnions' bosses
        mp.cut(cv, along_x(0, 0, s * (L['r_nose'] - 2.1), s * (L['boss_x'][1] + 0.1), L['r_tboss'] + 0.01))
    for x, z in COVER_SCREWS:
        mp.cut(cv, along_y(x, z, L['cover'][0] - 0.01, L['cover'][1] + 0.01, 2.75))
    parts.append(part(asm, 'Gear cover', cv, 'Aluminum'))
    for k, (x, z) in enumerate(COVER_SCREWS):
        parts.append(part(asm, 'Cover screw M5x16 %d' % (k + 1), mp.cap_screw(5.0, 16.0, (x, L['cover'][0], z), (0, 1, 0))))
    for s, tag in ((1, 'R'), (-1, 'L')):
        parts.append(part(asm, 'Trunnion %s' % tag, along_x(0, 0, s * L['trunnion_x'][0], s * L['trunnion_x'][1],
                                                              L['r_trunnion'])))
    for k, (y0, y1) in enumerate(L['bearings']):
        o = mp.new_occ(asm.component, 'Bearing BSD 2562 CGA %d' % (k + 1))
        inner, outer, balls, _ = mp.bearing_bodies(fd.BSD['d'], fd.BSD['D'], fd.BSD['B'], y0)
        for nm, body in (('inner ring', inner), ('outer ring', outer), ('balls', balls)):
            mp.add_body(o, body, nm, 'Steel')
        parts.append(o)
    parts.append(part(asm, 'Ring nut M66x1.5', along_y(0, 0, L['ring_nut'][0], L['ring_nut'][1], L['r_ring'],
                                                       L['r_shoulder_in'])))
    parts.append(part(asm, 'Idler pin', along_y(0, L['z_I'], L['idler_pin'][0], L['idler_pin'][1], 6.0)))
    for k, y0 in enumerate((L['idler_hub'][0], L['idler_hub'][0] + 10.0)):
        o = mp.new_occ(asm.component, 'Bearing 6201 %d' % (k + 1))
        inner, outer, balls, _ = mp.bearing_bodies(fd.B6201['d'], fd.B6201['D'], fd.B6201['B'], y0, 0.0, L['z_I'])
        for nm, body in (('inner ring', inner), ('outer ring', outer), ('balls', balls)):
            mp.add_body(o, body, nm, 'Steel')
        parts.append(o)
    # the motor unit, bought: flange, gearhead, motor, brake (sizes of the specification, assumed)
    m0, m1 = L['motor']
    zp = L['z_P']
    o = mp.new_occ(asm.component, 'Motor unit')
    fl = mp.box(-30.0, 30.0, m0, m0 + 6.0, zp - 30.0, zp + 30.0)
    mp.cut(fl, along_y(0, zp, m0 - 0.01, m0 + 6.01, 6.05))
    for x, z in MOTOR_SCREWS:
        mp.cut(fl, along_y(x, z, m0 - 0.01, m0 + 6.01, 2.75))
    mp.add_body(o, fl, 'Flange', 'Aluminum')
    for nm, a, b_ in (('Gearhead', m0 + 6.0, m0 + 66.0), ('Motor', m0 + 66.0, m1 - 30.0), ('Brake', m1 - 30.0, m1)):
        mp.add_body(o, along_y(0, zp, a, b_, L['r_motor']), nm, 'Aluminum')
    parts.append(o)
    for k, (x, z) in enumerate(MOTOR_SCREWS):
        parts.append(part(asm, 'Motor screw M5x12 %d' % (k + 1), mp.cap_screw(5.0, 12.0, (x, m0 + 6.0, z), (0, -1, 0))))
    parts.append(part(asm, 'Encoder', along_y(xs, zs, L['sensor'][0], L['sensor'][1], L['r_sensor']), 'Aluminum'))
    parts.append(part(asm, 'Rod seal', along_y(0, 0, 252.0, 258.0, 25.0, L['r_rod']), 'Steel'))
    return asm, parts


def screw(root):
    asm = mp.new_occ(root, 'Screw')
    sh = mp.stepped_shaft([(L['seat'][0], L['seat'][1], 25.0), (L['neck'][0], L['neck'][1], 30.0),
                           (L['thread'][0], L['thread'][1], 32.0), (L['journal_end'][0], L['journal_end'][1], 25.0)])
    mp.keyseat(sh, 25.0, L['key'][0], L['key'][1], 8.0, 4.0)
    mp.cut(sh, along_y(0, 0, L['seat'][0] - 0.01, L['washer'][1] + 20.0 - 3.0, 4.05),
           along_y(0, 0, L['journal_end'][1] + 3.0 - 16.0, L['journal_end'][1] + 0.01, 4.05))
    o = part(asm, 'Ball screw 32x10', sh)
    groove_y(o, o.component.bRepBodies.item(0), L['groove'][0], L['groove'][1])
    parts = [o]
    gz = G['z_G']
    th_G = -90.0                                                     # a tooth pointing at the idler
    parts.append(gear(asm, 'Screw gear z54', gz, 0.0, 0.0, L['gear_face'][0], L['gear_face'][1], L['gear_face'],
                      12.5, th_G, hub=(L['gear_hub'][0], L['gear_hub'][1], 18.0), web=(33.0, 6.0)))
    g = parts[-1]
    kw = mp.box(-4.0, 4.0, L['gear_hub'][0] - 0.01, L['gear_hub'][1] + 0.01, 12.5 - 4.0, 12.5 + 3.0 + 0.3)
    _cut_body(g, kw)
    parts.append(part(asm, 'Key 8x7x16', mp.parallel_key(25.0, L['key'][0], L['key'][1], 8.0, 7.0, 4.0)))
    parts.append(part(asm, 'Washer M8 1', along_y(0, 0, L['washer'][0], L['washer'][1], 15.0, 4.3)))
    parts.append(part(asm, 'End bolt M8x20', mp.cap_screw(8.0, 20.0, (0, L['washer'][0], 0), (0, 1, 0))))
    js = L['journal_end']
    parts.append(part(asm, 'End stop', along_y(0, 0, js[0], js[1], 18.0, 12.5)))
    parts.append(part(asm, 'Washer M8 2', along_y(0, 0, js[1], js[1] + 3.0, 15.0, 4.3)))
    parts.append(part(asm, 'Stop bolt M8x16', mp.cap_screw(8.0, 16.0, (0, js[1] + 3.0, 0), (0, -1, 0))))
    return asm, parts, th_G


def _cut_body(occ, tool):
    """Cut a temporary body, given in world mm, from the occurrence's (first) body, through a base feature."""
    m = occ.transform2.copy()
    m.invert()
    mp.tbm().transform(tool, m)
    comp = occ.component
    body = comp.bRepBodies.item(0)
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    tb = comp.bRepBodies.add(tool, bf)
    bf.finishEdit()
    tools = adsk.core.ObjectCollection.create()
    tools.add(bf.bodies.item(0))
    ci = comp.features.combineFeatures.createInput(body, tools)
    ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    comp.features.combineFeatures.add(ci)


def rod(root):
    asm = mp.new_occ(root, 'Rod')
    n0, n1 = L['nut']
    k = fd.RSI_KEY
    kc = 0.5 * (n0 + n1)
    nut = along_y(0, 0, n0, n1, 25.5, 16.6)
    mp.cut(nut, mp.box(-k['W'] / 2, k['W'] / 2, kc - k['K'] / 2, kc + k['K'] / 2, -25.6, -25.5 + k['H']))
    parts = [part(asm, 'Ball nut R32-10T4-RSI', nut)]
    parts.append(part(asm, 'Nut key 6x6.5x32', mp.box(-k['W'] / 2, k['W'] / 2, kc - k['K'] / 2, kc + k['K'] / 2,
                                                        -28.5, -25.5 + k['H'])))
    s0, s1 = L['sleeve']
    r = along_y(0, 0, s0, n1 + 0.01, L['r_sleeve'], 25.5)
    mp.union(r, along_y(0, 0, n1, s1, L['r_sleeve'], L['r_rod_in']),
             along_y(0, 0, s1 - 0.01, L['rod_tube'][1], L['r_rod'], L['r_rod_in']),
             along_y(0, 0, L['plug'][0] - 0.01, L['plug'][1], L['r_rod']),
             along_y(0, 0, L['stud'][0] - 0.01, L['stud'][1], 10.0))
    mp.cut(r, mp.box(-k['W'] / 2, k['W'] / 2, kc - k['K'] / 2, kc + k['K'] / 2, -28.5, -25.0),
           along_y(0, 0, 69.0, 71.0, 27.0),
           mp.box(-3.0, 3.0, 80.0, 140.0, 29.5, 31.0))
    parts.append(part(asm, 'Piston rod', r))
    parts.append(part(asm, 'Nut retaining ring', along_y(0, 0, 69.0, 71.0, 26.5, 22.0)))
    parts.append(part(asm, 'Guide key', mp.box(-3.0, 3.0, 80.0, 140.0, 29.5, 34.0)))
    parts.append(part(asm, 'Jam nut M20x1.5', mp.nut(20.0, (0, L['jam'][0], 0), (0, 1, 0), 10.0)))
    # the rod end, SKF SI 20 ES: its housing (shank, eye) and the spherical inner ring
    c = L['jam'][1] + fd.ROD_END['h']
    o = mp.new_occ(asm.component, 'Rod end SI 20 ES')
    hs = along_y(0, 0, L['jam'][1], c - 20.0, 16.0)
    mp.union(hs, along_x(c, 0, -fd.ROD_END['B_eye'] / 2, fd.ROD_END['B_eye'] / 2, fd.ROD_END['D_eye'] / 2))
    mp.cut(hs, along_y(0, 0, L['jam'][1] - 0.01, L['stud'][1] + 5.0, 10.0), mp.sphere((0, c, 0), 14.5))
    mp.add_body(o, hs, 'Housing', 'Steel')
    ball = mp.intersect(mp.sphere((0, c, 0), 14.49), mp.box(-8.0, 8.0, c - 15.0, c + 15.0, -15.0, 15.0))
    mp.cut(ball, along_x(c, 0, -9.0, 9.0, 10.0))
    mp.add_body(o, ball, 'Inner ring', 'Steel')
    parts.append(o)
    return asm, parts, c


def train(root, th_G):
    """The idler, the motor unit's pinion with its shaft, and the encoder's pinion with its shaft."""
    out = {}
    zI, zP = L['z_I'], L['z_P']
    th_I = mesh_phase(th_G, G['z_G'], -90.0, G['z_I'])
    a = mp.new_occ(root, 'Idler')
    out['Idler'] = (a, [gear(a, 'Idler gear z30', G['z_I'], 0.0, zI, L['idler_hub'][0], L['idler_hub'][1],
                             L['gear_face'], fd.B6201['D'] / 2, th_I)])
    th_P = mesh_phase(th_I, G['z_I'], -90.0, G['z_P'])
    a = mp.new_occ(root, 'Pinion')
    p = gear(a, 'Pinion z18', G['z_P'], 0.0, zP, L['gear_face'][0], L['gear_face'][1], L['gear_face'], 6.0, th_P)
    sh = part(a, 'Gearhead shaft', along_y(0, zP, L['gear_face'][0] - 1.0, L['motor'][0] + 6.0, 6.0))
    out['Pinion'] = (a, [p, sh])
    xs, zs = SENSOR
    th_S = mesh_phase(th_G, G['z_G'], L['sensor_angle'] - 360.0, G['z_S'])
    a = mp.new_occ(root, 'Encoder pinion')
    p = gear(a, 'Encoder pinion z26', G['z_S'], xs, zs, L['gear_face'][0], L['gear_face'][1], L['gear_face'], 4.0, th_S)
    sh = part(a, 'Encoder shaft', along_y(xs, zs, L['gear_face'][0] - 1.0, L['sensor'][0], 4.0))
    out['Encoder pinion'] = (a, [p, sh])
    return out


COVER_SCREWS = [(47.0 * math.cos(math.radians(a)), 47.0 * math.sin(math.radians(a))) for a in (30.0, 90.0, 150.0)] + \
               [(30.5, -80.0), (-30.5, -80.0), (0.0, -124.5)]
MOTOR_SCREWS = [(24.0, L['z_P'] + 24.0), (-24.0, L['z_P'] + 24.0), (24.0, L['z_P'] - 24.0), (-24.0, L['z_P'] - 24.0)]


# ---------- joints ----------
def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def _point(moving, point):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(P3(*[v * CM for v in point])).createForAssemblyContext(moving)


def revolute_y(root, moving, fixed, point, name):
    """An As-built revolute joint about the line through `point` parallel to y; the direction that really turns the
    part about y is kept, and its sign returned."""
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.YAxisJointDirection, Jd.ZAxisJointDirection, Jd.XAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(_point(moving, point)))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.05
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][k] * r0[c][k] for k in range(3)) for c in range(3)] for a in range(3)]
        wv = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        if abs(wv[1]) > 0.02 and abs(wv[0]) < 0.02 and abs(wv[2]) < 0.02:
            j.name = name
            return j, (1 if wv[1] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about y' % moving.name)


def slider_y(root, moving, fixed, name):
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.YAxisJointDirection, Jd.ZAxisJointDirection, Jd.XAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(_point(moving, (0, 0, 0))))
        inp.setAsSliderJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        y0 = moving.transform2.translation.y
        j.jointMotion.slideValue = 0.5
        dy = moving.transform2.translation.y - y0
        j.jointMotion.slideValue = 0.0
        if abs(abs(dy) - 0.5) < 1e-6:
            j.name = name
            return j, (1 if dy > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction slides %s along y' % moving.name)


def link(root, j1, s1, a1, j2, s2, a2, same_sense, name):
    """Joint one turns a1 degrees while joint two turns a2 degrees; same_sense says whether the parts turn the same
    way about +y; s1, s2 are the joints' own signs on +y."""
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%.6f deg' % a1)
    li.valueTwo = VS('%.6f deg' % a2)
    li.isReversed = ((1 if same_sense else -1) * s1 * s2) < 0
    m = root.motionLinks.add(li)
    m.name = name
    return m


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        wds = o.component.name.split()
        if wds[-1].isdigit():
            wds = wds[:-1]
        if wds[-1] in ('L', 'R'):
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


def build(fp=3.0, i_gh=24.0):
    o = design(fp, i_gh)
    refuse(o)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design_ = adsk.fusion.Design.cast(app.activeProduct)
    design_.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design_.rootComponent
    hs, _ = housing(root)
    hs.isGrounded = True
    sc, _, th_G = screw(root)
    rd, _, c = rod(root)
    tr = train(root, th_G)
    js, ss = revolute_y(root, sc, hs, (0, 0, 0), 'Screw turns')
    ji, si = revolute_y(root, tr['Idler'][0], hs, (0, 0, L['z_I']), 'Idler turns')
    jp, sp = revolute_y(root, tr['Pinion'][0], hs, (0, 0, L['z_P']), 'Pinion turns')
    je, se = revolute_y(root, tr['Encoder pinion'][0], hs, (SENSOR[0], 0, SENSOR[1]), 'Encoder pinion turns')
    jr, sr = slider_y(root, rd, hs, 'Rod slides')
    lim = jr.jointMotion.slideLimits
    lo, hi = sorted((0.0, sr * o['act']['stroke'] * CM))           # from retracted to extended, aft along +y
    lim.isMinimumValueEnabled, lim.minimumValue = True, lo
    lim.isMaximumValueEnabled, lim.maximumValue = True, hi
    zG = G['z_G']
    link(root, js, ss, 360.0, ji, si, 360.0 * zG / G['z_I'], False, 'Gear to idler 54:30')
    link(root, js, ss, 360.0, jp, sp, 360.0 * zG / G['z_P'], True, 'Gear to pinion 54:18')
    link(root, js, ss, 360.0, je, se, 360.0 * zG / G['z_S'], False, 'Gear to encoder pinion 54:26')
    li = root.motionLinks.createInput(js, jr)
    li.valueOne = VS('360 deg')                                     # one turn of the screw ...
    li.valueTwo = VI(10.0 * CM)                                     # ... moves the rod one lead
    li.isReversed = (ss * sr) > 0                                   # the rod goes aft (+y) for a turn about -y
    root.motionLinks.add(li).name = 'Lead 10 mm'
    for oc, col in ((hs, 'Paint - Enamel Glossy (Grey)'), (sc, 'Steel - Satin'), (rd, 'Paint - Enamel Glossy (Blue)'),
                    (tr['Idler'][0], 'Paint - Enamel Glossy (Yellow)'), (tr['Pinion'][0], 'Paint - Enamel Glossy (Red)'),
                    (tr['Encoder pinion'][0], 'Paint - Enamel Glossy (Yellow)')):
        mp.colour(design_, oc, col)
    app.activeViewport.fit()
    n, g = o['nut'], o['gears']
    pl = parts_list(root)
    return ('Flap actuator built: ball screw %s, lead %d mm; life %.2f of %d strokes at f_p %.2f (f_p up to %.2f '
            'passes); C0 %.2f of the ultimate force.\n'
            'Screw %.2f N m at the limit, %.2f N m backwards at the ultimate; gears %d:%d:%d, ratio %.1f, factors '
            '%.2f at the ultimate and %.2f in fatigue.\n'
            'Motor unit: %.0f rev/min, %.3f N m, %.0f W through a %.0f:1 gearhead; brake %.3f N m (needs %.3f).\n'
            'Rod end centre %.2f mm from the gimbal, retracted.\n\n'
            'Parts: %d components; %s.\n\n[%s]'
            % (n['name'], n['P'], n['life'], fd.STROKES, o['fp'], n['fp_max'], n['static'], o['T_lim'] / 1000,
               -o['T_back'] / 1000, G['z_G'], G['z_I'], G['z_P'], g['ratio'], g['n_static'],
               min(g['nf_P'], g['nf_G'], g['nf_I']), o['n_motor'], o['T_motor'] / 1000, o['P_motor'], o['i_gh'],
               o['T_brake'] / 1000, o['T_brake_need'] / 1000, c, sum(pl.values()),
               ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('fp', 'Screw operating factor f_p', 3.0), ('i_gh', 'Gearhead ratio', 24.0)]


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
        cid = 'FlapActuatorCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Flap actuator', 'The flap actuator of Chapter 33')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('FlapActuator could not start:\n' + traceback.format_exc())
