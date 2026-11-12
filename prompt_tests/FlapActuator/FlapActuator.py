# =============================================================================
# FlapActuator.py  -  the electromechanical actuator of one trailing-edge flap of a light twin (Volume 2, Chapter 33
# of "Mechanical Design in Autodesk Fusion with Claude"). A ball screw, turned through a spur gear, an idler and a
# pinion by a bought motor unit, pushes a rod out of an aluminium housing that hangs from the rear spar on two
# trunnions; the rod ends in a rod end on the flap's horn. Written from projects/flap_actuation/FlapActuator/PROMPT.txt.
#
#   INPUTS        the dialog: the screw's operating factor f_p (HIWIN F 3.8), the gearhead's ratio
#   DERIVED       flap_design.actuator_design(): forces, nut life and static rating, torques, buckling, critical
#                 speed, thrust bearings, gears, motor unit and brake, rod column, trunnions; flap_design.LAY: every
#                 position along the actuator's axis from the gimbal
#   HARD RULES    refuses a nut short of 60 000 strokes or of the ultimate force, gears under 1.5, a rod that buckles
#                 under twice the ultimate force, a motor faster than 6000 rev/min, and says by how much
#   CONVENTIONS   a new design; y along the actuator from the gimbal's axis, z up, x along the span; six
#                 subassemblies, every part a component; parts drawn in place in world millimetres; every face that
#                 would meet another stands GAP off it, so nothing touches; the ball groove swept with one turn of
#                 twist per lead with BallScrewAxis.groove_section(); As-built Joints and Motion Links
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


def _common():
    """common/ of the companion: the first folder above this script that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % __file__)
        d = up


_C = _common()
if _C not in sys.path:
    sys.path.insert(0, _C)
import companion_paths as CP                 # noqa: E402  (puts every lib/ on sys.path)
import flap_design as fd                     # noqa: E402
import mdfc_parts as MP                      # noqa: E402
importlib.reload(fd)                         # Fusion keeps modules between runs
importlib.reload(MP)


def _load_ballscrewaxis():
    """common/BallScrewAxis/BallScrewAxis.py, loaded as a module: its groove section and its groove proportions."""
    path = os.path.join(CP.ROOT, 'common', 'BallScrewAxis', 'BallScrewAxis.py')
    spec = importlib.util.spec_from_file_location('BallScrewAxis_for_FlapActuator', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


BSA = _load_ballscrewaxis()

VERSION = 'FlapActuator 1.0 (2026-11-12)'
CM = 0.1                                     # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
GAP = 0.05                                   # mm, every face that would meet another stands this far off it
SUBS = ('Housing', 'Screw', 'Rod', 'Idler', 'Pinion', 'Encoder pinion')
STATIONS = (0.0, 10.0, 20.0, 30.0, 40.0)     # deg of flap


# ---------- the numbers ----------
def design(fp=3.0, i_gh=24.0):
    """actuator_design() for this f_p; the motor unit's speed, torque and brake taken to the gearhead's ratio typed
    (actuator_design() sizes them for a ratio of its own; the power does not change with the ratio)."""
    if not fp > 0 or not i_gh > 0:
        raise ValueError('f_p and the gearhead ratio must be positive (f_p %s, ratio %s).' % (fp, i_gh))
    d = dict(fd.actuator_design(fp=float(fp)))
    k = d['i_gh'] / float(i_gh)
    d['n_motor'] = d['n_motor'] / k
    d['T_motor'] = d['T_motor'] * k
    d['T_brake_need'] = d['T_brake_need'] * k
    d['T_brake'] = d['T_brake'] * k
    d['i_gh'] = float(i_gh)
    return d


def refusals(d):
    """The hard rules: what fails, and by how much."""
    out = []
    n, act, g = d['nut'], d['act'], d['gears']
    if n['life'] < 1.0:
        lasts = n['life'] * fd.STROKES
        out.append('The nut %s at f_p %.2f lasts %.0f strokes, %.0f strokes (%.1f %%) short of the %d strokes '
                   'asked; f_p up to %.2f passes.' % (n['name'], d['fp'], lasts, fd.STROKES - lasts,
                                                     100 * (1 - n['life']), fd.STROKES, n['fp_max']))
    if n['static'] < 1.0:
        out.append('The nut\'s static rating C0 %.0f N is %.0f N below the ultimate force %.0f N.'
                   % (n['C0'], act['F_ult'] - n['C0'], act['F_ult']))
    if g['n_static'] < 1.5:
        out.append('The gears stand %.2f against yield at the ultimate force, %.2f short of 1.5.'
                   % (g['n_static'], 1.5 - g['n_static']))
    nf = min(g['nf_P'], g['nf_G'], g['nf_I'])
    if nf < 1.5:
        out.append('The weakest gear stands %.2f in fatigue at the limit force, %.2f short of 1.5.' % (nf, 1.5 - nf))
    if d['P_cr'] < 2 * act['F_ult']:
        out.append('The rod buckles at %.0f N, %.0f N short of twice the ultimate force (%.0f N).'
                   % (d['P_cr'], 2 * act['F_ult'] - d['P_cr'], 2 * act['F_ult']))
    if d['n_motor'] > 6000.0:
        out.append('The motor would turn at %.0f rev/min, %.0f rev/min faster than 6000.'
                   % (d['n_motor'], d['n_motor'] - 6000.0))
    return out


# ---------- solids as data: world millimetres ----------
def cylY(y0, y1, r, x=0.0, z=0.0):
    return ('cyl', (x, y0, z), (x, y1, z), r)


def cylX(x0, x1, r, y=0.0, z=0.0):
    return ('cyl', (x0, y, z), (x1, y, z), r)


def cylZ(z0, z1, r, x=0.0, y=0.0):
    return ('cyl', (x, y, z0), (x, y, z1), r)


def box(x0, x1, y0, y1, z0, z1):
    return ('box', x0, x1, y0, y1, z0, z1)


def sph(c, r):
    return ('sph', tuple(c), r)


def add(*a):
    return ('add',) + a


def sub(a, *b):
    return ('sub', a) + b


def both(*a):
    return ('and',) + a


def tubeY(y0, y1, ro, ri, x=0.0, z=0.0):
    return sub(cylY(y0, y1, ro, x, z), cylY(y0 - 1.0, y1 + 1.0, ri, x, z))


def slot_z(x, y0, y1, w, z0, z1):
    """A round-ended slot or feather key, w wide in x, from y0 to y1, from z0 to z1."""
    r = w / 2.0
    return add(box(x - r, x + r, y0 + r, y1 - r, z0, z1), cylZ(z0, z1, r, x, y0 + r), cylZ(z0, z1, r, x, y1 - r))


def cap_screw(d, L, p, ax):
    """An ISO 4762 socket head cap screw in mdfc_parts' proportions: head 1.5 d across and d high, the head's
    underside at p, the shank L long along the unit vector ax."""
    at = lambda t: tuple(p[i] + ax[i] * t for i in range(3))
    head = sub(('cyl', p, at(-d), 0.75 * d), ('cyl', at(-0.5 * d), at(-(d + 1.0)), 0.43 * d))
    return add(head, ('cyl', at(-0.001), at(L), d / 2.0))


def bearing(d, D, B, y0, x=0.0, z=0.0):
    """A ball bearing drawn as mdfc_parts draws one (inner ring, outer ring, balls), every face GAP inside the
    catalogue's envelope so that it touches neither its seat nor its neighbours."""
    g = MP.bearing_geometry(d + 2 * GAP, D - 2 * GAP)
    ya, yb = y0 + GAP, y0 + B - GAP
    balls = add(*[sph((x + g['dm'] / 2 * math.cos(2 * math.pi * i / g['Z']), (ya + yb) / 2,
                       z + g['dm'] / 2 * math.sin(2 * math.pi * i / g['Z'])), g['Dw'] / 2) for i in range(g['Z'])])
    return [('Inner ring', tubeY(ya, yb, g['in_od'] / 2, d / 2 + GAP, x, z), 'steel'),
            ('Outer ring', tubeY(ya, yb, D / 2 - GAP, g['out_id'] / 2, x, z), 'steel'),
            ('Balls', balls, 'steel')]


def mesh_phase(ph_a, z_a, c_a, z_b, c_b):
    """The angle of a tooth of gear b (rad, in the xz plane from +x towards +z) that meshes tooth-on-gap with gear a,
    whose tooth is at ph_a: a gap of b on the line of centres where a has a tooth, turned back by the ratio."""
    psi = math.atan2(c_b[1] - c_a[1], c_b[0] - c_a[0])
    pitch = 2 * math.pi / z_a
    off = (ph_a - psi) % pitch                              # a's nearest tooth at psi + off
    if off > pitch / 2:
        off -= pitch
    gap_b = psi + math.pi - (z_a / float(z_b)) * off
    return gap_b + math.pi / z_b


# ---------- the layout: every part, where it is, what it is made of ----------
def layout(d):
    L = fd.LAY
    G = d['gears']
    g = L['gear']
    m = g['m']
    nut = d['nut']
    ds, P = float(nut['ds']), float(nut['P'])
    Dw = fd.HIWIN[fd.NUT][2]
    k = BSA.ballscrew(d_s=ds, P=P, D_w=Dw)                  # the groove's proportions as BallScrewAxis cuts them
    k = dict(P=P, d_m=k['d_m'], R_g=k['R_g'], r_land=k['r_land'], r_bore=k['r_bore'], per_turn=k['per_turn'],
             D_w=Dw)
    rad = lambda z: m * z / 2.0
    rf = lambda z: rad(z) - 1.25 * m                        # the root, as mdfc_parts' tooth_outline
    ra = lambda z: rad(z) + m
    zG, zI, zP, zS = g['z_G'], g['z_I'], g['z_P'], g['z_S']
    cG, cI, cP = (0.0, 0.0), (0.0, L['z_I']), (0.0, L['z_P'])
    a = math.radians(L['sensor_angle'])
    cS = (G['a_GS'] * math.cos(a), G['a_GS'] * math.sin(a))
    xe, ze = cS
    phG = math.atan2(cI[1] - cG[1], cI[0] - cG[0])          # a tooth of the screw's gear points at the idler
    phI = mesh_phase(phG, zG, cG, zI, cI)
    phP = mesh_phase(phI, zI, cI, zP, cP)
    phS = mesh_phase(phG, zG, cG, zS, cS)
    f0, f1 = L['gear_face']
    y_n0, y_n1 = L['nose']
    y_sh = L['shoulder_ring'][0]
    r_pocket = ra(zS) + 2.0                                 # the encoder pinion's pocket in the gear case
    r_elobe = r_pocket + 6.0
    r_gland, r_gland_bore, seal, r_seal = 32.0, L['r_rod'] + 0.2, (250.0, 258.0), 27.0
    parts = []

    def part(sub_, name, bodies, origin=(0.0, 0.0, 0.0), copies=(), gear=None, groove=None, balls=None,
             colour=None):
        parts.append(dict(sub=sub_, name=name, bodies=bodies, origin=origin, copies=list(copies), gear=gear,
                          groove=groove, balls=balls, colour=colour))

    # --- the fixed side ---
    cover_pts = [((L['r_cavity'] + L['r_nose']) / 2 * math.cos(math.radians(t)),
                  (L['r_cavity'] + L['r_nose']) / 2 * math.sin(math.radians(t))) for t in (45.0, 135.0)]
    cover_pts += [((L['cav_x'] + L['lobe_x']) / 2, -60.0),
                  (15.0, (L['cav_z'] + L['lobe_z']) / 2), (-15.0, (L['cav_z'] + L['lobe_z']) / 2),
                  (xe + (r_pocket + r_elobe) / 2 * math.cos(a), ze + (r_pocket + r_elobe) / 2 * math.sin(a))]
    motor_pts = [(sx * 24.0, L['z_P'] + sz * 24.0) for sx in (1, -1) for sz in (1, -1)]
    tb_in = 40.0                                            # the trunnions' bosses start inside the nose's wall
    outer = add(cylY(y_n0, y_n1, L['r_nose']),
                box(-L['lobe_x'], L['lobe_x'], y_n0, L['lobe_s'], L['lobe_z'], 0.0),
                cylY(y_n0, L['lobe_s'], r_elobe, xe, ze),
                cylY(L['boss'][0] - 1.0, L['boss'][1], L['r_boss']),
                cylY(L['tube'][0] - 1.0, L['tube'][1], L['r_tube']),
                cylY(L['gland'][0] - 1.0, L['gland'][1], r_gland),
                cylX(tb_in, L['boss_x'][1], L['r_tboss']), cylX(-L['boss_x'][1], -tb_in, L['r_tboss']))
    holes = [cylY(y_n0 - 0.5, y_sh, L['r_cavity']),
             box(-L['cav_x'], L['cav_x'], y_n0 - 0.5, y_sh, L['cav_z'], 0.0),
             cylY(y_n0 - 0.5, y_sh, r_pocket, xe, ze),
             cylY(y_sh - 0.5, L['bearings'][0][0] + 0.5, L['r_shoulder_in']),
             cylY(L['bearings'][0][0], L['tube'][1], L['bore'] / 2),
             cylY(L['ring_nut'][0] - GAP, L['ring_nut'][1] + GAP, L['r_ring'] + GAP),
             cylY(L['tube'][1] - 0.5, L['gland'][1] + 0.5, r_gland_bore),
             cylY(seal[0], seal[1], r_seal),
             box(-L['slot_w'] / 2, L['slot_w'] / 2, L['tube'][0], L['tube'][1], L['bore'] / 2 - 1.0, L['slot_top']),
             cylX(L['trunnion_x'][0] - GAP, L['boss_x'][1] + 1.0, L['r_trunnion'] + GAP),
             cylX(-L['boss_x'][1] - 1.0, -L['trunnion_x'][0] + GAP, L['r_trunnion'] + GAP),
             cylY(y_sh - 0.5, L['lobe_s'] + 0.5, 6.0 + GAP, 0.0, L['z_I']),        # the idler's pin
             cylY(y_sh - 0.5, L['lobe_s'] + 0.5, 8.0, 0.0, L['z_P']),             # the gearhead's shaft
             cylY(y_sh - 0.5, L['lobe_s'] + 0.5, 4.0, xe, ze)]                    # the encoder's shaft
    holes += [cylY(y_n0 - 0.5, 8.1, 2.5 + GAP, x, z) for x, z in cover_pts]       # M5, 10 deep
    holes += [cylY(L['lobe_s'] - 6.1, L['lobe_s'] + 0.5, 2.5 + GAP, x, z) for x, z in motor_pts]
    housing = sub(outer, box(-L['boss_x'][0], L['boss_x'][0], y_n0 - 20.0, y_n0, -L['r_tboss'] - 5, L['r_tboss'] + 5),
                  *holes)
    part('Housing', 'Actuator housing', [('Actuator housing', housing, 'al')], colour='Aluminum - Satin')
    c0, c1 = L['cover'][0], L['cover'][1] - GAP
    cover = sub(add(cylY(c0, c1, L['r_nose']), box(-L['lobe_x'], L['lobe_x'], c0, c1, L['lobe_z'], 0.0),
                    cylY(c0, c1, r_elobe, xe, ze)),
                box(47.0, 70.0, c0 - 1, c1 + 1, -L['r_tboss'] - 1, L['r_tboss'] + 1),     # clear of the trunnions
                box(-70.0, -47.0, c0 - 1, c1 + 1, -L['r_tboss'] - 1, L['r_tboss'] + 1),
                cylY(L['idler_pin'][0] - 0.1, c1 + 0.5, 6.0 + GAP, 0.0, L['z_I']),
                *[cylY(c0 - 0.5, c1 + 0.5, 2.75, x, z) for x, z in cover_pts])
    part('Housing', 'Gear cover', [('Gear cover', cover, 'al')], colour='Aluminum - Satin')
    x0, z0 = cover_pts[0]
    part('Housing', 'Cover screw M5x16', [('Cover screw M5x16', cap_screw(5.0, 16.0, (x0, c0 - GAP, z0), (0, 1, 0)),
                                           'steel')],
         copies=[('t', (x - x0, 0.0, z - z0)) for x, z in cover_pts[1:]], colour='Paint - Enamel Glossy (Black)')
    part('Housing', 'Trunnion', [('Trunnion', cylX(L['trunnion_x'][0], L['trunnion_x'][1], L['r_trunnion']), 'steel')],
         copies=[('rz', (0.0, 0.0, 0.0))], colour='Steel - Satin')
    part('Housing', 'Ring nut M66x1.5', [('Ring nut M66x1.5', tubeY(L['ring_nut'][0] + GAP, L['ring_nut'][1] - GAP,
                                                                    L['r_ring'], 24.0), 'steel')],
         colour='Steel - Satin')
    (b0, b1), (b2, b3) = L['bearings']
    part('Housing', 'Bearing BSD 2562 CGA', bearing(fd.BSD['d'], fd.BSD['D'], fd.BSD['B'], b0),
         copies=[('rz', (0.0, b0 + b3, 0.0))], colour='Steel - Satin')          # the pair back to back
    part('Housing', 'Rod seal', [('Rod seal', tubeY(seal[0] + GAP, seal[1] - GAP, r_seal - GAP, L['r_rod'] + 0.1),
                                  'rubber')], colour='Paint - Enamel Glossy (Black)')
    mo0, mo1 = L['motor']
    zp = L['z_P']
    motor = sub(add(box(-31.0, 31.0, mo0 + GAP, mo0 + 6.0, zp - 31.0, zp + 31.0),
                    cylY(mo0 + 5.0, mo1 - 2.0, 20.0, 0.0, zp),
                    cylY(mo0 + 6.0, 106.0, L['r_motor'], 0.0, zp),           # the planetary gearhead
                    cylY(107.0, 186.0, L['r_motor'], 0.0, zp),               # the brushless motor
                    cylY(187.0, mo1 - 2.0, L['r_motor'] - 2.0, 0.0, zp),     # the spring-applied brake
                    cylY(mo1 - 2.5, mo1, 12.0, 0.0, zp)),
                cylY(mo0 - 0.5, 52.2, 6.2, 0.0, zp),
                *[cylY(mo0 - 0.5, mo0 + 6.5, 2.75, x, z) for x, z in motor_pts])
    part('Housing', 'Motor unit', [('Motor unit', motor, 'al')], colour='Paint - Enamel Glossy (Black)')
    xm, zm = motor_pts[0]
    part('Housing', 'Motor screw M5x12', [('Motor screw M5x12', cap_screw(5.0, 12.0, (xm, mo0 + 6.0 + GAP, zm),
                                                                          (0, -1, 0)), 'steel')],
         copies=[('t', (x - xm, 0.0, z - zm)) for x, z in motor_pts[1:]], colour='Paint - Enamel Glossy (Black)')
    se0, se1 = L['sensor']
    part('Housing', 'Encoder', [('Encoder', sub(cylY(se0 + GAP, se1, L['r_sensor'], xe, ze),
                                                cylY(se0 - 0.5, 52.2, 3.2, xe, ze)), 'al')],
         colour='Paint - Enamel Glossy (Blue)')
    ip0, ip1 = L['idler_pin']
    ih0, ih1 = L['idler_hub']
    part('Housing', 'Idler pin', [('Idler pin', add(cylY(ip0, ip1, 6.0 - GAP, 0.0, L['z_I']),
                                                    cylY(y_n0 + GAP, ih0 - 0.1, 7.5, 0.0, L['z_I'])), 'steel')],
         colour='Steel - Satin')
    part('Housing', 'Bearing 6201', bearing(fd.B6201['d'], fd.B6201['D'], fd.B6201['B'], ih0, 0.0, L['z_I']),
         copies=[('t', (0.0, fd.B6201['B'], 0.0))], colour='Steel - Satin')

    # --- the screw ---
    s0, s1 = L['seat']
    th0, th1 = L['thread']
    j0, j1 = L['journal_end']
    k0, k1 = L['key']
    screw = sub(add(cylY(s0, s1 + 0.5, fd.BSD['d'] / 2 - GAP), cylY(L['neck'][0], L['neck'][1] + 0.5, 15.0),
                    cylY(th0, th1, k['r_land']), cylY(j0 - 0.5, j1, 10.0)),
                cylY(s0 - 0.5, s0 + 18.1, 4.0 + GAP),                      # the end bolt's thread, M8
                cylY(j1 - 13.1, j1 + 0.5, 4.0 + GAP),                      # the stop bolt's thread, M8
                slot_z(0.0, k0 - GAP, k1 + GAP, 8.0 + 2 * GAP, fd.BSD['d'] / 2 - 4.0 - GAP, fd.BSD['d'] / 2 + 1.0))
    part('Screw', 'Ball screw 32x10', [('Ball screw 32x10', screw, 'steel')], groove=(L['groove'][0], L['groove'][1]),
         colour='Steel - Satin')
    h0, h1 = L['gear_hub']
    blankG = sub(add(sub(cylY(f0, f1, rf(zG)), cylY(f0 - 0.5, f1 + 0.5, 33.0)), cylY(17.0, 23.0, 33.2),
                     cylY(h0 + GAP, h1 - GAP, 20.0)),
                 cylY(h0 - 0.5, h1 + 0.5, fd.BSD['d'] / 2 + GAP),
                 box(-4.0 - GAP, 4.0 + GAP, h0 - 0.5, h1 + 0.5, 0.0, fd.BSD['d'] / 2 + 3.3))
    part('Screw', 'Screw gear z54', [('Screw gear z54', blankG, 'steel')], gear=(zG, phG, f0, f1),
         colour='Paint - Enamel Glossy (Grey)')
    part('Screw', 'Key 8x7x16', [('Key 8x7x16', slot_z(0.0, k0, k1, 8.0, fd.BSD['d'] / 2 - 4.0,
                                                       fd.BSD['d'] / 2 + 3.0), 'steel')], colour='Steel - Satin')
    w0, w1 = L['washer']
    part('Screw', 'Washer M8', [('Washer M8', tubeY(w0 + GAP, w1 - GAP, 16.0, 4.3), 'steel')],
         copies=[('t', (0.0, j1 - w0, 0.0))], colour='Steel - Satin')
    part('Screw', 'End bolt M8x20', [('End bolt M8x20', cap_screw(8.0, 20.0, (0.0, w0, 0.0), (0, 1, 0)), 'steel')],
         colour='Paint - Enamel Glossy (Black)')
    st0, st1 = L['stop']
    part('Screw', 'End stop', [('End stop', tubeY(st0 + GAP, st1 - GAP, 18.0, 10.1), 'steel')], colour='Steel - Satin')
    part('Screw', 'Stop bolt M8x16', [('Stop bolt M8x16', cap_screw(8.0, 16.0, (0.0, j1 + (w1 - w0), 0.0), (0, -1, 0)),
                                       'steel')], colour='Paint - Enamel Glossy (Black)')

    # --- the rod ---
    sl0, sl1 = L['sleeve']
    n0, n1 = L['nut']
    rt0, rt1 = L['rod_tube']
    r_nut = nut['D'] / 2.0
    r_sb = r_nut + 0.1                                       # the sleeve's bore round the nut
    kc = 0.5 * (n0 + n1)                                     # the nut's middle
    K, W, H = fd.RSI_KEY['K'], fd.RSI_KEY['W'], fd.RSI_KEY['H']
    rod = sub(add(cylY(sl0, sl1, L['r_sleeve']), cylY(sl1 - 1.0, L['plug'][1], L['r_rod']),
                  cylY(L['stud'][0] - 1.0, L['stud'][1], 10.0)),
              cylY(sl0 - 0.5, n1 + GAP, r_sb), cylY(n1, rt1, L['r_rod_in']),
              cylY(69.0, 71.0, r_sb + 1.9),                                        # the retaining ring's groove
              slot_z(0.0, kc - K / 2 - 0.1, kc + K / 2 + 0.1, W + 0.2, -(r_nut + 3.0 + 0.1), -(r_sb - 0.6)),
              box(-3.1, 3.1, 80.0 - GAP, 140.0 + GAP, 29.5 - GAP, L['r_sleeve'] + 1.0))  # the guide key's seat
    part('Rod', 'Piston rod', [('Piston rod', rod, 'steel')], colour='Steel - Satin')
    nutb = sub(tubeY(n0, n1, r_nut, k['r_bore']),
               slot_z(0.0, kc - K / 2 - 0.1, kc + K / 2 + 0.1, W + 0.2, -(r_nut + 1.0), -(r_nut - H - 0.1)))
    cen = []
    for i in range(4 * k['per_turn']):                       # T4: four turns of balls centred on the nut's middle
        y = kc - 2 * P + P * (i + 0.5) / k['per_turn']
        t = 2 * math.pi * y / P
        cen.append((k['d_m'] / 2 * math.sin(t), y, k['d_m'] / 2 * math.cos(t)))
    part('Rod', 'Ball nut R32-10T4-RSI', [('Ball nut R32-10T4-RSI', nutb, 'steel')], groove=(n0, n1),
         balls=add(*[sph(c, Dw / 2) for c in cen]), colour='Paint - Enamel Glossy (Blue)')
    part('Rod', 'Nut key 6x6.5x32', [('Nut key 6x6.5x32', slot_z(0.0, kc - K / 2, kc + K / 2, W, -(r_nut + 3.0),
                                                                 -(r_nut - H)), 'steel')], colour='Steel - Satin')
    ring = sub(tubeY(69.0 + 0.1, 71.0 - 0.1, r_sb + 1.8, 23.0), box(-1.5, 1.5, 68.0, 72.0, 20.0, 30.0))
    part('Rod', 'Nut retaining ring', [('Nut retaining ring', ring, 'steel')], colour='Paint - Enamel Glossy (Black)')
    part('Rod', 'Guide key', [('Guide key', box(-3.0, 3.0, 80.0, 140.0, 29.5, 34.0), 'steel')], colour='Steel - Satin')
    jn0, jn1 = L['jam']
    part('Rod', 'Jam nut M20x1.5', [('Jam nut M20x1.5', tubeY(jn0 + GAP, jn1 - GAP, 15.0, 10.1), 'steel')],
         colour='Steel - Satin')
    RE = fd.ROD_END
    yc = jn1 + RE['h']                                       # the rod end's centre, retracted
    ball = sub(both(sph((0.0, yc, 0.0), 14.5), box(-RE['B'] / 2, RE['B'] / 2, yc - 20, yc + 20, -20, 20)),
               cylX(-RE['B'], RE['B'], RE['d'] / 2, yc, 0.0))
    eye = sub(add(cylX(-RE['B_eye'] / 2, RE['B_eye'] / 2, RE['D_eye'] / 2, yc, 0.0),
                  cylY(jn1 + GAP, yc - 20.0, 14.0)),
              sph((0.0, yc, 0.0), 14.5 + GAP), cylY(jn1 - 0.5, L['stud'][1] + 4.0, 10.1))
    part('Rod', 'Rod end SI 20 ES', [('Inner ring', ball, 'steel'), ('Eye and shank', eye, 'steel')],
         colour='Steel - Satin')

    # --- the gears on their own axes ---
    blankI = sub(add(cylY(f0, f1, rf(zI), 0.0, L['z_I']), cylY(ih0 + GAP, f0 + 0.5, 19.0, 0.0, L['z_I'])),
                 cylY(ih0 - 0.5, ih1 + 0.5, fd.B6201['D'] / 2 + GAP, 0.0, L['z_I']))
    part('Idler', 'Idler gear z30', [('Idler gear z30', blankI, 'steel')], origin=(0.0, 0.0, L['z_I']),
         gear=(zI, phI, f0, f1), colour='Paint - Enamel Glossy (Grey)')
    part('Pinion', 'Pinion z18', [('Pinion z18', sub(cylY(f0, f1, rf(zP), 0.0, zp), cylY(f0 - 0.5, f1 + 0.5,
                                                                                          6.0 + GAP, 0.0, zp)),
                                   'steel')], origin=(0.0, 0.0, zp), gear=(zP, phP, f0, f1),
         colour='Paint - Enamel Glossy (Grey)')
    part('Pinion', 'Gearhead shaft', [('Gearhead shaft', cylY(f0 - 2.0, 52.0, 6.0, 0.0, zp), 'steel')],
         colour='Steel - Satin')
    part('Encoder pinion', 'Encoder pinion z26', [('Encoder pinion z26', sub(cylY(f0, f1, rf(zS), xe, ze),
                                                                             cylY(f0 - 0.5, f1 + 0.5, 3.0 + GAP, xe, ze)),
                                                   'steel')], origin=(xe, 0.0, ze), gear=(zS, phS, f0, f1),
         colour='Paint - Enamel Glossy (Grey)')
    part('Encoder pinion', 'Encoder shaft', [('Encoder shaft', cylY(f0 - 2.0, 52.0, 3.0, xe, ze), 'steel')],
         colour='Steel - Satin')

    joints = dict(Screw=(0.0, 100.0, 0.0), Rod=(0.0, 100.0, 0.0), Idler=(0.0, f0, L['z_I']),
                  Pinion=(0.0, f0, zp), **{'Encoder pinion': (xe, f0, ze)})
    tips = {'Screw gear z54': (cG, ra(zG)), 'Idler gear z30': (cI, ra(zI)), 'Pinion z18': (cP, ra(zP)),
            'Encoder pinion z26': (cS, ra(zS))}
    return dict(parts=parts, k=k, joints=joints, tips=tips, rod_end_y=yc, gear_c=dict(G=cG, I=cI, P=cP, S=cS),
                phases=dict(G=phG, I=phI, P=phP, S=phS), z=dict(G=zG, I=zI, P=zP, S=zS))


# ---------- what the layout gives before anything is built ----------
def _bbox(n):
    t = n[0]
    if t == 'cyl':
        p0, p1, r = n[1], n[2], n[3]
        e = [p1[i] - p0[i] for i in range(3)]
        Ln = math.sqrt(sum(v * v for v in e))
        ext = [r * math.sqrt(max(0.0, 1 - (e[i] / Ln) ** 2)) for i in range(3)]
        return ([min(p0[i], p1[i]) - ext[i] for i in range(3)], [max(p0[i], p1[i]) + ext[i] for i in range(3)])
    if t == 'box':
        return ([n[1], n[3], n[5]], [n[2], n[4], n[6]])
    if t == 'sph':
        return ([n[1][i] - n[2] for i in range(3)], [n[1][i] + n[2] for i in range(3)])
    bs = [_bbox(c) for c in n[1:]]
    if t == 'add':
        return ([min(b[0][i] for b in bs) for i in range(3)], [max(b[1][i] for b in bs) for i in range(3)])
    if t == 'and':
        return ([max(b[0][i] for b in bs) for i in range(3)], [min(b[1][i] for b in bs) for i in range(3)])
    return bs[0]


def _apply(c, p):
    kind, v = c
    if kind == 't':
        return [p[0] + v[0], p[1] + v[1], p[2] + v[2]]
    return [-p[0] + v[0], -p[1] + v[1], p[2] + v[2]]        # 'rz': half a turn about z, then v


def _pieces(n):
    """The boxes of the pieces a solid is built up from (what is cut away left in): finer than its own box."""
    if n[0] in ('cyl', 'box', 'sph', 'and'):
        return [_bbox(n)]
    if n[0] == 'add':
        return [b for c in n[1:] for b in _pieces(c)]
    return _pieces(n[1])


def instance_boxes(lay):
    """(sub, name, lo, hi) of the pieces of every component's occurrence at rest, the gears out to their tips."""
    out = []
    for p in lay['parts']:
        boxes = [b for _, n, _ in p['bodies'] for b in _pieces(n)]
        if p['name'] in lay['tips']:
            (cx, cz), r = lay['tips'][p['name']]
            y0, y1 = p['gear'][2], p['gear'][3]
            boxes.append(([cx - r, y0, cz - r], [cx + r, y1, cz + r]))
        for lo, hi in boxes:
            out.append((p['sub'], p['name'], lo, hi))
            for c in p['copies']:
                cs = [_apply(c, [x, y, z]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
                out.append((p['sub'], p['name'], [min(q[i] for q in cs) for i in range(3)],
                            [max(q[i] for q in cs) for i in range(3)]))
    return out


def room(lay, d):
    """The least room to the skins above and below with the flap at 0 to 40 deg (each part's box carried into the
    wing's section along the actuator's line from the gimbal), and how far forward of the gimbal's axis it reaches."""
    A = fd.LINK['A']
    L0 = d['act']['L_retracted']
    bx = instance_boxes(lay)
    up = low = 1e9
    for delta in range(0, 41, 2):
        kk = fd.linkage(float(delta))
        u = kk['u']
        n = (-u[1], u[0])
        slide = kk['length'] - L0
        for s, _, lo, hi in bx:
            for y in (lo[1], hi[1]):
                yy = y + (slide if s == 'Rod' else 0.0)
                px, pz = A[0] + yy * u[0] + hi[2] * n[0], A[1] + yy * u[1] + hi[2] * n[1]
                up = min(up, fd.skin_up(px) - pz)
                px, pz = A[0] + yy * u[0] + lo[2] * n[0], A[1] + yy * u[1] + lo[2] * n[1]
                low = min(low, pz - fd.skin_low(px))
    return up, low, -min(lo[1] for _, _, lo, _ in bx)


def parts_list(root):
    """{component name: how many} of the parts, the subassemblies left out."""
    out = {}
    for o in root.allOccurrences:
        if o.childOccurrences.count == 0:
            out[o.component.name] = out.get(o.component.name, 0) + 1
    return out


# ---------- Fusion ----------
def solid(n):
    """A temporary body from the data of a solid."""
    t = n[0]
    if t == 'cyl':
        return MP.cyl(list(n[1]), list(n[2]), n[3])
    if t == 'box':
        return MP.box(*n[1:])
    if t == 'sph':
        return MP.sphere(list(n[1]), n[2])
    a = solid(n[1])
    rest = [solid(c) for c in n[2:]]
    if t == 'add':
        return MP.union(a, *rest)
    if t == 'sub':
        return MP.cut(a, *rest)
    return MP.intersect(a, *rest)


class _Materials:
    WANT = dict(al=(('Aluminum 7075', 'Aluminum 7075-T6', 'Aluminum 6061', 'Aluminum'), 'Aluminum'),
                steel=(('Steel', 'Steel, Alloy'), 'Steel'),
                rubber=(('Rubber', 'Rubber, Nitrile', 'Rubber, Silicone'), 'Rubber'))

    def __init__(self, app):
        self.lib = None
        for nm in ('Fusion Material Library', 'Fusion 360 Material Library'):
            self.lib = app.materialLibraries.itemByName(nm)
            if self.lib:
                break
        self.found = {}

    def get(self, kind):
        if kind not in self.found:
            mat = None
            if self.lib:
                names, word = self.WANT[kind]
                for nm in names:
                    mat = self.lib.materials.itemByName(nm)
                    if mat:
                        break
                if not mat:
                    for i in range(self.lib.materials.count):
                        if self.lib.materials.item(i).name.startswith(word):
                            mat = self.lib.materials.item(i)
                            break
            self.found[kind] = mat
        return self.found[kind]

    def set(self, body, kind):
        mat = self.get(kind)
        if mat:
            body.material = mat


def _groove(comp, body, y0, y1, k, sense):
    """The ball groove cut into `body` from y0 to y1 on the component's y axis: BallScrewAxis' section of the tube
    round the helix (its u along +z, v along +x, so its angle turns positively about +y) swept along the axis with
    one turn of twist per lead, `sense` the sign of the twist."""
    sec = BSA.groove_section(k['d_m'] / 2, k['R_g'], k['P'], 2 * math.pi * y0 / k['P'])
    sk = comp.sketches.add(MP.xz_plane(comp, y0))
    sk.isComputeDeferred = True
    pts = [sk.modelToSketchSpace(P3(v * CM, y0 * CM, u * CM)) for u, v in sec]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    ps = comp.sketches.add(comp.xYConstructionPlane)
    axis = ps.sketchCurves.sketchLines.addByTwoPoints(ps.modelToSketchSpace(P3(0, y0 * CM, 0)),
                                                      ps.modelToSketchSpace(P3(0, y1 * CM, 0)))
    ps.isVisible = False
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk.profiles.item(0), comp.features.createPath(axis, False), CUT)
    inp.twistAngle = VI(sense * 2 * math.pi * (y1 - y0) / k['P'])
    inp.participantBodies = [body]
    return sw.add(inp)


def _right_handed(body, y0, k):
    """True if the groove cut in `body` is right-handed (its angle 2 pi y / P about +y): a point 1 mm inside the
    helix's centre on that angle is out of the steel, the point half a turn round from it in the steel."""
    y = y0 + 2.25 * k['P']
    th = 2 * math.pi * y / k['P']
    r = k['d_m'] / 2 - 1.0
    PC = adsk.fusion.PointContainment
    a = body.pointContainment(P3(r * math.sin(th) * CM, y * CM, r * math.cos(th) * CM))
    b = body.pointContainment(P3(-r * math.sin(th) * CM, y * CM, -r * math.cos(th) * CM))
    if a == PC.PointOutsidePointContainment and b == PC.PointInsidePointContainment:
        return True
    if a == PC.PointInsidePointContainment and b == PC.PointOutsidePointContainment:
        return False
    return None


def _matrix(c, origin):
    m = adsk.core.Matrix3D.create()
    if c[0] == 'rz':
        m.setCell(0, 0, -1.0)
        m.setCell(1, 1, -1.0)
        t = [-origin[0] + c[1][0], -origin[1] + c[1][1], origin[2] + c[1][2]]
    else:
        t = [origin[i] + c[1][i] for i in range(3)]
    m.translation = V3(t[0] * CM, t[1] * CM, t[2] * CM)
    return m


def _point(occ, p):
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(sk.modelToSketchSpace(P3(p[0] * CM, p[1] * CM, p[2] * CM)))
    return adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(occ))


def _rot(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def _revolute(root, moving, fixed, p, name):
    """An As-built revolute joint about the world y axis through p; the joint's direction tried until the part
    really turns about y. Returns (joint, sign): sign +1 when a positive value turns it positively about +y."""
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.YAxisJointDirection, Jd.ZAxisJointDirection, Jd.XAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, _point(moving, p))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = _rot(moving)
        j.jointMotion.rotationValue = 0.2
        r1 = _rot(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][c] * r0[b][c] for c in range(3)) for b in range(3)] for a in range(3)]
        w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        if abs(w[1]) > 0.3 and abs(w[0]) < 0.05 and abs(w[2]) < 0.05:
            j.name = name
            return j, (1 if w[1] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about y' % moving.name)


def _slider(root, moving, fixed, p, name):
    """An As-built slider along the world y axis. Returns (joint, sign): +1 when a positive value moves it to +y."""
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.YAxisJointDirection, Jd.XAxisJointDirection, Jd.ZAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, _point(moving, p))
        inp.setAsSliderJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        t0 = moving.transform2.translation
        j.jointMotion.slideValue = 0.5
        t1 = moving.transform2.translation
        j.jointMotion.slideValue = 0.0
        dy, dx, dz = t1.y - t0.y, t1.x - t0.x, t1.z - t0.z
        if abs(dy) > 0.4 and abs(dx) < 0.01 and abs(dz) < 0.01:
            j.name = name
            return j, (1 if dy > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction slides %s along y' % moving.name)


def _link(root, j1, j2, v1, v2, reverse, name):
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = v1                                       # when joint one moves v1 ...
    li.valueTwo = v2                                       # ... joint two moves v2
    li.isReversed = reverse
    ml = root.motionLinks.add(li)
    ml.name = name
    return ml


def build(fp=3.0, i_gh=24.0):
    d = design(fp, i_gh)
    bad = refusals(d)
    if bad:
        raise ValueError('Refused (f_p %.2f, gearhead %.1f:1):\n%s\n\n[%s]' % (fp, i_gh, '\n'.join(bad), VERSION))
    lay = layout(d)
    k = lay['k']
    up, low, fwd = room(lay, d)
    act = d['act']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    subs = {}
    for s in SUBS:
        o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        o.component.name = s
        subs[s] = o
    subs['Housing'].isGrounded = True
    mats = _Materials(app)
    sense, handed = 1, None
    for p in lay['parts']:
        occ = MP.new_occ(subs[p['sub']].component, p['name'], p['origin'])
        comp = occ.component
        first = None
        for bname, n, mat in p['bodies']:
            body = MP.add_body(occ, solid(n), bname)
            mats.set(body, mat)
            if first is None:
                first = body
        if p['gear']:
            z, ph, y0, y1 = p['gear']
            o = p['origin']
            MP.spur_teeth(occ, first, fd.LAY['gear']['m'], z, 20.0, fd.LAY['gear']['backlash'], ph, y0 - o[1], y1 - o[1])
        if p['groove']:
            y0, y1 = p['groove']
            if handed is None:                              # the screw: find which way a positive twist turns
                feat = _groove(comp, first, y0, y1, k, sense)
                handed = _right_handed(comp.bRepBodies.item(0), y0, k)
                if handed is False:
                    feat.deleteMe()
                    sense = -1
                    _groove(comp, comp.bRepBodies.item(0), y0, y1, k, sense)
                    handed = _right_handed(comp.bRepBodies.item(0), y0, k)
                handed = bool(handed)
            else:
                _groove(comp, first, y0, y1, k, sense)
        if p['balls'] is not None and handed:
            mats.set(MP.add_body(occ, solid(p['balls']), 'Balls'), 'steel')
        occs = [occ]
        for c in p['copies']:
            o2 = subs[p['sub']].component.occurrences.addExistingComponent(comp, _matrix(c, p['origin']))
            o2.isGroundToParent = True
            occs.append(o2)
        for o2 in occs:
            if p['colour']:
                MP.colour(des, o2, p['colour'])

    J = lay['joints']
    H = subs['Housing']
    js, ss = _revolute(root, subs['Screw'], H, J['Screw'], 'Screw turns')
    jr, sr = _slider(root, subs['Rod'], H, J['Rod'], 'Rod slides')
    ji, si = _revolute(root, subs['Idler'], H, J['Idler'], 'Idler turns')
    jp, sp = _revolute(root, subs['Pinion'], H, J['Pinion'], 'Pinion turns')
    je, se = _revolute(root, subs['Encoder pinion'], H, J['Encoder pinion'], 'Encoder pinion turns')
    lim = jr.jointMotion.slideLimits
    s = act['stroke'] * CM
    if sr > 0:
        lim.isMinimumValueEnabled, lim.minimumValue = True, 0.0
        lim.isMaximumValueEnabled, lim.maximumValue = True, s
    else:
        lim.isMaximumValueEnabled, lim.maximumValue = True, 0.0
        lim.isMinimumValueEnabled, lim.minimumValue = True, -s
    z = lay['z']
    # a right-hand screw turned positively about +y draws a nut that cannot turn towards -y: the rod goes aft (+y)
    # one lead for each turn the other way
    _link(root, js, jr, VS('360 deg'), VI(k['P'] * CM), ss * sr > 0, 'Lead %.0f mm' % k['P'])
    _link(root, js, ji, VS('%d deg' % z['I']), VS('%d deg' % z['G']), ss * si > 0, '%d:%d idler' % (z['G'], z['I']))
    _link(root, js, je, VS('%d deg' % z['S']), VS('%d deg' % z['G']), ss * se > 0, '%d:%d encoder' % (z['G'], z['S']))
    _link(root, js, jp, VS('%d deg' % z['P']), VS('%d deg' % z['G']), ss * sp < 0, '%d:%d pinion' % (z['G'], z['P']))
    try:
        app.activeViewport.fit()
    except Exception:
        pass

    pl = parts_list(root)
    mass = sum(o.physicalProperties.mass for o in root.allOccurrences
               if o.childOccurrences.count == 0 and o.component.name != 'Motor unit') + fd.LAY['motor_mass']
    return report(d, lay, pl, mass, handed, (up, low, fwd))


def report(d, lay, pl, mass, handed, rooms):
    """The message: the nut, the torques, the gears, the motor unit and brake, the rod end, the parts, the version."""
    act, nut, g = d['act'], d['nut'], d['gears']
    z = lay['z']
    up, low, fwd = rooms
    nf = min(g['nf_P'], g['nf_G'], g['nf_I'])
    return ('Flap actuator built: a 32 x 10 ball screw turned through gears of %d, %d and %d teeth (and %d to the '
            'encoder) by a motor unit with a %.0f:1 gearhead; the rod end on the flap\'s horn.\n\n'
            'Nut HIWIN %s at f_p %.2f: life %.2f of %d strokes (f_p up to %.2f passes); C0 %.2f of the ultimate '
            'force %.0f N.\n'
            'Screw: %.2f N m at the limit force %.0f N, %.2f N m backwards at the ultimate (it back-drives: the brake '
            'holds it); buckling 0.5 F_k %.0f N, critical speed 0.8 n_k %.0f rev/min against %.1f rev/min.\n'
            'Thrust bearings 2 x SKF BSD 2562 CGA back to back: life %.2f, static %.2f. Idler bearings 2 x 6201.\n'
            'Gears m %.1f, 20 deg, %.0f mm wide: factors %.2f against yield at the ultimate and %.2f in fatigue at '
            'the limit; contact %.2f.\n'
            'Motor unit (brushless motor, %.0f:1 planetary gearhead, spring-applied brake): %.0f rev/min, %.3f N m, '
            '%.1f W; brake %.3f N m (twice the %.3f N m the back-driving screw puts on it).\n'
            'Rod as a column: %.0f N against twice the ultimate %.0f N; trunnions %.2f against yield.\n'
            'Rod end SI 20 ES centre %.2f mm from the gimbal\'s axis retracted, %.2f mm extended: stroke %.2f mm, '
            '%.4f turns of the screw (ball groove %s).\n'
            'Room to the skins at 0 to 40 deg: %.2f mm above, %.2f mm below; %.2f mm forward of the gimbal\'s axis. '
            'Mass %.2f kg with the motor unit at %.1f kg.\n\n'
            'Parts: %d components in %d kinds: %s.\n\n[%s]'
            % (z['G'], z['I'], z['P'], z['S'], d['i_gh'], nut['name'], d['fp'], nut['life'], fd.STROKES, nut['fp_max'],
               nut['static'], act['F_ult'], d['T_lim'] / 1000, act['F_limit'], -d['T_back'] / 1000, d['F_kmax'],
               d['n_kmax'], d['n_screw'], d['brg_life'], d['brg_static'], fd.LAY['gear']['m'], fd.LAY['gear']['F'],
               g['n_static'], nf, g['nc'], d['i_gh'], d['n_motor'], d['T_motor'] / 1000, d['P_motor'],
               d['T_brake'] / 1000, d['T_brake_need'] / 1000, d['P_cr'], 2 * act['F_ult'], d['trunnion_n'],
               lay['rod_end_y'], lay['rod_end_y'] + act['stroke'], act['stroke'], d['turns_stroke'],
               'right-handed, balls in the nut' if handed else 'of unproven hand, the nut drawn without balls',
               up, low, fwd, mass, fd.LAY['motor_mass'], sum(pl.values()), len(pl),
               ', '.join('%s %d' % (n, c) for n, c in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            ui.messageBox(build(fp=ins.itemById('fp').value, i_gh=ins.itemById('i_gh').value), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('fp', 'Screw operating factor f_p (HIWIN F 3.8)', '', VI(3.0))
        ins.addValueInput('i_gh', 'Gearhead ratio', '', VI(24.0))
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
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Flap actuator', 'The trailing-edge flap\'s actuator')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Flap actuator could not start:\n' + traceback.format_exc())
