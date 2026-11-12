# =============================================================================
# AxleHousing.py  -  Chapter 30 of "Mechanical Design in Autodesk Fusion with Claude": the housing of the light truck's
# live rear axle and its two semi-floating wheel ends, as real parts. A banjo housing holds the carrier of
# RearAxleCarrier.py and carries the truck's rear from its spring seats to its wheel bearings; in each tube's end a
# half shaft turns on a deep-groove bearing, the shaft's flange carrying the wheel's studs.
#
#   INPUTS        the dialog: the half shaft's body, the tube's outside and wall, the wheel bearing's bore
#   DERIVED       axle_design.wheel_end() and housing_tube(): the six load cases, the shaft's three factors, the
#                 bearing's life and its load in a turn, the tube's stress in the pothole; layout(): every position
#   HARD RULES    build() refuses a shaft under 1.5 against yield in the hardest turn or in fatigue, a bearing that
#                 does not last 127.32 million turns, a tube under 1.5 in the pothole, and sizes that cannot go together
#   CONVENTIONS   the axle along Y (+Y the left wheel), forward +X, up +Z, the axle's middle at the origin; three
#                 subassemblies (Housing grounded, Wheel end L, Wheel end R), every part a component grounded to its
#                 parent, one body each; each wheel end on an As-built revolute joint about Y on the housing
#   ACCEPTANCE    the message; 57 components in 16 kinds; no interference at rest or turned; the bearings on their
#                 seats; each wheel end able to go together; the housing no lower than 168.07 mm below the axle
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
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import axle_design as ad        # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(ad)
importlib.reload(mp)

VERSION = 'AxleHousing 1.0 (2026-11-12)'
CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal

# ---------- given by the prompt ----------
N_MIN = 1.5                        # the least factor of the shaft (yield, fatigue) and of the tube (pothole)
B_OFF, Y_INNER = 65.0, 100.0       # the bearing inboard of the wheel's middle plane; the side gear's hold on the shaft
Y_SEAT = 575.0                     # the spring seats, from the axle's middle
JOINT_X = 165.0                    # the face the carrier's plate is bolted to
FLANGE_D, STUD_PCD, N_WSTUD, WSTUD = 190.0, 139.7, 6, 14.0   # the shaft's flange and the wheel's studs M14
SEAL_D, SEAL_W = 100.0, 10.0       # the oil seal's outside and width; its bore is the bearing's shoulder d_S
N_SCREW, SCREW_D, SCREW_L = 10, 8.0, 20.0                  # the cover's screws M8x20
PLUG_D = 22.0                      # the filler and drain plugs M22
CSTUD = 12.0                       # the carrier's studs and nuts M12

# ---------- chosen here (the prompt leaves them open) ----------
DRUM_IN = 130.0                    # the drum's inside from the middle along Y, clear of the carrier's legs
FLANGE_R, FLANGE_T = 200.0, 20.0   # the front flange: the carrier's plate outline (radius 200), 20 thick
PLATE_T = 16.0                     # the carrier's plate, x 165 to 181 in RearAxleCarrier.carrier()
CSTUD_IN = 18.0                    # the carrier's studs screwed 18 mm into the flange
REAR_NECK_R, REAR_FLANGE_R, REAR_FLANGE_T = 120.0, 150.0, 14.0
COVER_T, DISH_R, DISH_D = 6.0, 118.0, 14.0               # the rear cover: a plate and a dish behind it
SCREW_R = 136.0                    # the cover's screws on a circle of 272 mm
FILLER_Z, DRAIN_Z = -20.0, -90.0   # the plugs in the cover's dish
BREATHER_Y = 200.0                 # the breather on top of the left tube
SEAT_L, SEAT_W = 120.0, 75.0       # the spring seat's pad under the tube: along X (the spring) and along Y
U_TIP, U_SPLINE = 40.0, 125.0      # the half shaft's splines from its tip to the body (the side gear's hub ends at 106.75)
FL_T, STUD_OUT = 14.0, 40.0        # the shaft's flange thickness; the studs out past it


# ---------- the design ----------
def design(d_body=48.0, D=80.0, t=8.0, d_seat=65.0):
    """The wheel end and the tube at the spring seat, worked by axle_design (Budynas and Nisbett 2015)."""
    bore = int(round(d_seat))
    if abs(bore - d_seat) > 1e-9 or bore not in ad.TABLE_11_2:
        raise ValueError('No 02-series deep-groove bearing of %g mm bore in Table 11-2 (%s mm).'
                         % (d_seat, ', '.join(str(k) for k in sorted(ad.TABLE_11_2))))
    b = ad.brief()
    w = ad.wheel_end(b, b_off=B_OFF, y_inner=Y_INNER, d_seat=bore, d_body=float(d_body))
    h = ad.housing_tube(b, w, D=float(D), t=float(t), y_seat=Y_SEAT)
    return dict(b=b, d=ad.duty(b), w=w, h=h, d_body=float(d_body), D=float(D), t=float(t))


def designation(d):
    return '62%02d' % (d // 5)


def refuse(Dz):
    """The hard rules of strength, in the order of the chapter: the shaft, the bearing, the tube."""
    w, h = Dz['w'], Dz['h']
    short = []
    if w['n_yield'] < N_MIN:
        short.append('%.2f against yield in the hardest turn' % w['n_yield'])
    if w['n_fatigue'] < N_MIN:
        short.append('%.2f in fatigue (DE-Goodman)' % w['n_fatigue'])
    if short:
        raise ValueError('A half shaft of %g mm keeps %s, under %.1f.' % (Dz['d_body'], ' and '.join(short), N_MIN))
    if w['L10_rev'] < w['rev_needed']:
        raise ValueError('The wheel bearing %s (%d mm bore) lasts %.1f million turns, under the %.2f million turns of '
                         'the life.' % (designation(w['bearing']['d']), w['bearing']['d'], w['L10_rev'] / 1e6,
                                        w['rev_needed'] / 1e6))
    if h['n'] < N_MIN:
        raise ValueError('A tube %g x %g keeps %.2f in the pothole (%.1f MPa at the spring seat), under %.1f.'
                         % (h['D'], h['t'], h['n'], h['sigma'], N_MIN))


def carrier():
    """The carrier this housing holds: RearAxleCarrier.py, loaded (not copied) for its sizes."""
    path = CP.script('RearAxleCarrier')
    spec = importlib.util.spec_from_file_location('RearAxleCarrier_held_by_housing', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def layout(Dz):
    """Every position, mm. Along the axle: u, the distance out from the middle (y = u on the left, -u on the right)."""
    w = Dz['w']
    brg = w['bearing']
    RC = carrier()
    g = RC.design()['g']
    C = ad.SKF_TRB[RC.CASE_BRG]
    L = dict(d=float(brg['d']), D_b=float(brg['D']), B=float(brg['B']), dS=float(brg['dS']), dH=float(brg['dH']))
    # the drum round the crown wheel: its tip, the carrier's gap and wall
    L['r_tip'] = g['da2'] / 2.0
    L['wall'] = RC.WALL
    L['R_in'] = L['r_tip'] + RC.GAP
    L['R_out'] = L['R_in'] + RC.WALL
    L['clearance'] = RC.R_STAT - L['R_out']
    L['clear_min'] = RC.CLEARANCE_MIN
    L['reach'] = RC.HUB_Y + C[2] + 16.0                  # the carrier's legs and adjusters, 16 past the case's cups
    L['hub_end'] = RC.HUB_Y + C[2]                       # the side gear's splined hub
    L['spline'] = RC.SHAFT_SPLINE                        # minor, major, number, width
    L['y_in'], L['y_out'] = DRUM_IN, DRUM_IN + RC.WALL
    L['JB'] = ad.JOINT_BOTTOM
    # the wheel end, out from the middle
    yb = w['y_b']
    L['yb0'], L['yb1'] = yb - L['B'] / 2.0, yb + L['B'] / 2.0          # the bearing's faces
    L['u_sh0'] = L['yb0'] - 4.0                          # the housing's shoulder (d_H) under the outer ring
    L['u_seal1'] = L['u_sh0']
    L['u_seal0'] = L['u_seal1'] - SEAL_W                 # the oil seal's bore
    L['u_rel0'] = L['u_seal0'] - 5.5                     # a relief round the retaining ring's inner end
    L['u_ring0'] = L['u_rel0'] + 2.0                     # the retaining ring, from here to the bearing
    L['u_bh0'] = L['u_rel0'] - 4.0                       # the bearing's housing at the tube's end
    L['u_cone0'] = L['u_bh0'] - 30.0                     # the tube widens to it
    L['u_bf0'] = L['yb1'] - 12.0                         # the brake's flange, its face flush with the bearing's
    L['u_rp1'] = L['yb1'] + 6.0                          # the retainer plate over the outer ring
    L['u_fl0'] = L['yb1'] + 19.5                         # the shaft's flange
    L['u_fl1'] = L['u_fl0'] + FL_T
    L['u_tube0'], L['u_bore0'] = DRUM_IN - 10.0, DRUM_IN - 5.0
    # radii
    D, t = Dz['D'], Dz['t']
    L['r_t'], L['r_tb'] = D / 2.0, (D - 2 * t) / 2.0
    L['r_rel'] = L['dS'] / 2.0 + 3.0
    L['r_seal'] = SEAL_D / 2.0
    L['r_sh'] = L['dH'] / 2.0
    L['r_b'] = L['D_b'] / 2.0
    L['r_bh'] = L['r_b'] + 15.0
    L['r_bf'] = L['r_bh'] + 30.0
    L['r_bfh'] = L['r_bh'] + 15.0                        # the brake flange's and the retainer plate's holes
    L['r_rp'] = L['dS'] / 2.0 + 3.0                      # the retainer plate's bore
    L['bg'] = mp.bearing_geometry(L['d'], L['D_b'])
    # the rear: the flange's front face just behind the drum
    L['x_rf1'] = -(math.ceil(L['R_out']) + 2.0)
    L['x_rf0'] = L['x_rf1'] - REAR_FLANGE_T
    L['x_cv0'] = L['x_rf0'] - COVER_T
    L['x_dish0'] = L['x_cv0'] - DISH_D
    # what cannot go together
    db, dmin, dmaj = Dz['d_body'], L['spline'][0], L['spline'][1]
    if L['clearance'] < RC.CLEARANCE_MIN - 1e-9:
        raise ValueError('The drum round the crown wheel leaves %.2f mm of ground clearance, under %.0f.'
                         % (L['clearance'], RC.CLEARANCE_MIN))
    if L['reach'] + 5.0 > DRUM_IN:
        raise ValueError('The carrier reaches %.2f mm from the middle: the drum is too narrow.' % L['reach'])
    if db >= L['d']:
        raise ValueError('A body of %g mm is not narrower than the bearing\'s %g mm bore: the bearing could not be '
                         'pressed on from the shaft\'s inner end.' % (db, L['d']))
    if db <= dmaj:
        raise ValueError('A body of %g mm is not wider than the splines\' %g mm.' % (db, dmaj))
    if t <= 0 or 2 * L['r_tb'] < db + 8.0:
        raise ValueError('A tube %g x %g leaves a bore of %g mm: less than 4 mm round the %g mm body.'
                         % (D, t, D - 2 * t, db))
    if L['r_tb'] > L['r_seal'] - 3.0 or L['r_t'] > L['r_bh'] - 5.0:
        raise ValueError('A tube %g x %g does not fit the bearing\'s housing and its %g mm seal.' % (D, t, SEAL_D))
    if not (L['dS'] / 2.0 + 3.0 < L['r_seal'] < L['r_sh']):
        raise ValueError('An oil seal %gx%gx%g does not fit between the retaining ring and the bearing\'s shoulder.'
                         % (L['dS'], SEAL_D, SEAL_W))
    if L['r_rp'] < L['bg']['in_od'] / 2.0 + 1.0:
        raise ValueError('The retainer plate would rub the bearing\'s inner ring.')
    return L


# ---------- helpers, world millimetres ----------
def ycyl(s, u0, u1, r, x=0.0, z=0.0):
    """A cylinder along the axle from u0 to u1 out from the middle on side s (+1 left, -1 right)."""
    return mp.cyl((x, s * u0, z), (x, s * u1, z), r)


def ycone(s, u0, u1, r0, r1):
    return mp.tbm().createCylinderOrCone(P3(0, s * u0 * CM, 0), r0 * CM, P3(0, s * u1 * CM, 0), r1 * CM)


def xcyl(x0, x1, r, y=0.0, z=0.0):
    return mp.cyl((x0, y, z), (x1, y, z), r)


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def on_circle(r, n, phase):
    return [(r * math.cos(phase + 2 * math.pi * k / n), r * math.sin(phase + 2 * math.pi * k / n)) for k in range(n)]


def part(parent, name, tb, material='Steel'):
    o = mp.new_occ(parent.component, name)
    mp.add_body(o, tb, name, material)
    return o


# ---------- the housing ----------
def banjo(L):
    R_out, R_in, wall, JB = L['R_out'], L['R_in'], L['wall'], L['JB']
    yo, yi = L['y_out'], L['y_in']
    body = mp.cyl((0, -yo, 0), (0, yo, 0), R_out)                                    # the drum
    neck = mp.intersect(xcyl(0.0, JOINT_X, R_out), mp.box(-1.0, JOINT_X + 1, -yo, yo, JB, R_out + 1))
    flange = mp.intersect(xcyl(JOINT_X - FLANGE_T, JOINT_X, FLANGE_R),
                          mp.box(JOINT_X - FLANGE_T - 1, JOINT_X + 1, -FLANGE_R - 1, FLANGE_R + 1, JB, FLANGE_R + 1))
    mp.union(body, neck, flange)
    mp.union(body, xcyl(-100.0, L['x_rf1'] - 1.0, REAR_NECK_R), xcyl(L['x_rf0'], L['x_rf1'], REAR_FLANGE_R))
    for s in (1, -1):
        mp.union(body, ycyl(s, L['u_tube0'], L['u_cone0'], L['r_t']))                 # the tube
        mp.union(body, ycone(s, L['u_cone0'], L['u_bh0'], L['r_t'], L['r_bh']))
        mp.union(body, ycyl(s, L['u_bh0'], L['yb1'], L['r_bh']))                      # the bearing's housing
        mp.union(body, ycyl(s, L['u_bf0'], L['yb1'], L['r_bf']))                      # the brake's flange
        y0, y1 = s * (Y_SEAT - SEAT_W / 2.0), s * (Y_SEAT + SEAT_W / 2.0)
        mp.union(body, mp.box(-SEAT_L / 2.0, SEAT_L / 2.0, min(y0, y1), max(y0, y1),
                              -(L['r_t'] + 8.0), -(L['r_t'] - 20.0)))                  # the spring seat, under
    mp.union(body, zcyl(L['r_t'] - 10.0, L['r_t'] + 6.0, 9.0, y=BREATHER_Y))           # the breather's boss
    # the hollows
    mp.cut(body, mp.cyl((0, -yi, 0), (0, yi, 0), R_in))
    mp.cut(body, mp.intersect(xcyl(0.0, JOINT_X + 1, R_in), mp.box(-1.0, JOINT_X + 2, -yi, yi, JB + wall, R_in + 1)))
    mp.cut(body, mp.box(JOINT_X, JOINT_X + 60.0, -260.0, 260.0, -260.0, 260.0))      # nothing past the joint face
    mp.cut(body, xcyl(-100.0, L['x_rf0'] - 1.0, REAR_NECK_R - wall))
    for s in (1, -1):
        mp.cut(body, ycyl(s, L['u_bore0'], L['u_rel0'] + 0.5, L['r_tb']))
        mp.cut(body, ycyl(s, L['u_rel0'], L['u_seal0'] + 0.5, L['r_rel']))
        mp.cut(body, ycyl(s, L['u_seal0'], L['u_seal1'] + 0.5, L['r_seal']))           # the seal's bore
        mp.cut(body, ycyl(s, L['u_sh0'], L['yb0'] + 0.5, L['r_sh']))                   # the bearing's shoulder
        mp.cut(body, ycyl(s, L['yb0'], L['yb1'] + 1.0, L['r_b']))                      # the bearing's bore
        for x, z in on_circle(L['r_bfh'], 4, math.pi / 4):
            mp.cut(body, ycyl(s, L['u_bf0'] - 1.0, L['yb1'] + 1.0, 6.5, x, z))
        mp.cut(body, zcyl(-(L['r_t'] + 9.0), -(L['r_t'] + 1.0), 6.5, y=s * Y_SEAT))  # the spring's centre bolt
    for y, z in ad.joint_bolts():
        mp.cut(body, xcyl(JOINT_X - CSTUD_IN, JOINT_X + 1.0, CSTUD / 2.0, y, z))
    for y, z in on_circle(SCREW_R, N_SCREW, math.pi / N_SCREW):
        mp.cut(body, xcyl(L['x_rf0'] - 1.0, L['x_rf1'] + 1.0, SCREW_D / 2.0, y, z))
    mp.cut(body, zcyl(L['r_t'] - 15.0, L['r_t'] + 7.0, 4.0, y=BREATHER_Y))
    return body


def rear_cover(L):
    x0, x1, xd = L['x_cv0'], L['x_rf0'], L['x_dish0']
    c = xcyl(x0, x1, REAR_FLANGE_R)
    mp.union(c, xcyl(xd, x0 + 0.5, DISH_R))
    mp.cut(c, xcyl(xd + COVER_T, x1 + 1.0, REAR_NECK_R - L['wall']))
    for z in (FILLER_Z, DRAIN_Z):
        mp.union(c, xcyl(xd - 3.0, xd + 0.5, 15.0, 0.0, z))
        mp.cut(c, xcyl(xd - 4.0, xd + COVER_T + 1.0, PLUG_D / 2.0, 0.0, z))
    for y, z in on_circle(SCREW_R, N_SCREW, math.pi / N_SCREW):
        mp.cut(c, xcyl(x0 - 1.0, x1 + 1.0, SCREW_D / 2.0 + 0.5, y, z))
    return c


def plug(L, z):
    xd = L['x_dish0']
    p = xcyl(xd - 13.0, xd - 3.0, 14.0, 0.0, z)                                        # the head, drawn round
    return mp.union(p, xcyl(xd - 3.01, xd + COVER_T - 1.0, PLUG_D / 2.0, 0.0, z))     # the thread, short of the inside


def breather(L):
    r = L['r_t']
    b = zcyl(r - 4.0, r + 6.01, 4.0, y=BREATHER_Y)                                     # the thread, into the hex
    mp.union(b, zcyl(r + 6.0, r + 12.0, 7.0, y=BREATHER_Y), zcyl(r + 11.99, r + 18.0, 6.0, y=BREATHER_Y))
    mp.cut(b, zcyl(r - 5.0, r + 15.0, 1.5, y=BREATHER_Y))                             # the vent
    mp.cut(b, mp.cyl((-8.0, BREATHER_Y, r + 15.0), (8.0, BREATHER_Y, r + 15.0), 1.5))
    return b


def housing_parts(H, L):
    part(H, 'Banjo housing', banjo(L))
    part(H, 'Rear cover', rear_cover(L))
    for k, (y, z) in enumerate(on_circle(SCREW_R, N_SCREW, math.pi / N_SCREW)):
        part(H, 'Cover screw M%dx%d %d' % (SCREW_D, SCREW_L, k + 1),
             mp.cap_screw(SCREW_D, SCREW_L, (L['x_cv0'], y, z), (1, 0, 0)))
    part(H, 'Filler plug M%d' % PLUG_D, plug(L, FILLER_Z))
    part(H, 'Drain plug M%d' % PLUG_D, plug(L, DRAIN_Z))
    part(H, 'Breather', breather(L), 'Brass')
    x_nut = JOINT_X + PLATE_T
    m_nut = 0.8 * CSTUD
    for k, (y, z) in enumerate(ad.joint_bolts()):
        part(H, 'Carrier stud M%d %d' % (CSTUD, k + 1),
             xcyl(JOINT_X - CSTUD_IN, x_nut + m_nut + 2.4, CSTUD / 2.0, y, z))
        part(H, 'Carrier nut M%d %d' % (CSTUD, k + 1), mp.nut(CSTUD, (x_nut, y, z), (1, 0, 0), m_nut))
    for s, tag in ((1, 'L'), (-1, 'R')):
        _, outer, _, _ = mp.bearing_bodies(L['d'], L['D_b'], L['B'], L['yb0'] if s > 0 else -L['yb1'])
        part(H, 'Bearing %s outer ring %s' % (designation(int(L['d'])), tag), outer)
        part(H, 'Oil seal %gx%gx%g %s' % (L['dS'], SEAL_D, SEAL_W, tag),
             mp.tube((0, s * L['u_seal0'], 0), (0, s * L['u_seal1'], 0), L['r_seal'], L['dS'] / 2.0), 'Rubber, Silicone')
        rp = mp.tube((0, s * L['yb1'], 0), (0, s * L['u_rp1'], 0), L['r_bf'], L['r_rp'])
        for x, z in on_circle(L['r_bfh'], 4, math.pi / 4):
            mp.cut(rp, ycyl(s, L['yb1'] - 1.0, L['u_rp1'] + 1.0, 6.5, x, z))
        part(H, 'Retainer plate %s' % tag, rp)


# ---------- a wheel end ----------
def half_shaft(L, Dz, s):
    dmin, dmaj, n, wd = L['spline']
    sh = mp.spline_shaft((0, s * U_TIP, 0), (0, s * (U_SPLINE + 0.01), 0), dmin, dmaj, n, wd)
    # each narrower step runs 0.01 into the wider one after it, so the steps fall exactly where they are drawn
    mp.union(sh, ycyl(s, U_SPLINE, L['u_ring0'] + 0.01, Dz['d_body'] / 2.0))         # the body
    mp.union(sh, ycyl(s, L['u_ring0'], L['yb1'] + 0.01, L['d'] / 2.0))                # the seat: ring and bearing
    mp.union(sh, ycyl(s, L['yb1'], L['u_fl0'] + 0.01, L['dS'] / 2.0))                 # the shoulder d_S
    mp.union(sh, ycyl(s, L['u_fl0'], L['u_fl1'], FLANGE_D / 2.0))                      # the flange
    for x, z in on_circle(STUD_PCD / 2.0, N_WSTUD, math.pi / 6):
        mp.cut(sh, ycyl(s, L['u_fl0'] - 1.0, L['u_fl1'] + 1.0, WSTUD / 2.0, x, z))
    return sh


def wheel_end_parts(E, L, Dz, s, tag):
    part(E, 'Half shaft %s' % tag, half_shaft(L, Dz, s))
    inner, _, balls, _ = mp.bearing_bodies(L['d'], L['D_b'], L['B'], L['yb0'] if s > 0 else -L['yb1'])
    name = designation(int(L['d']))
    part(E, 'Bearing %s inner ring %s' % (name, tag), inner)
    part(E, 'Bearing %s balls %s' % (name, tag), balls)
    part(E, 'Retaining ring %s' % tag, mp.tube((0, s * L['u_ring0'], 0), (0, s * L['yb0'], 0), L['dS'] / 2.0, L['d'] / 2.0))
    for k, (x, z) in enumerate(on_circle(STUD_PCD / 2.0, N_WSTUD, math.pi / 6)):
        st = ycyl(s, L['u_fl0'] - 6.0, L['u_fl0'], 11.0, x, z)                         # the head behind the flange
        mp.union(st, ycyl(s, L['u_fl0'] - 0.01, L['u_fl1'] + STUD_OUT, WSTUD / 2.0, x, z))
        part(E, 'Wheel stud M%d %s %d' % (WSTUD, tag, k + 1), st)


# ---------- joints and the parts list ----------
def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def turns_about_y(root, moving, fixed, name):
    """An As-built revolute joint turning `moving` about the Y axis; the joint's directions are tried and the one
    that turns it about Y is kept."""
    comp = moving.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.isVisible = False
    point = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    J = adsk.fusion.JointDirections
    for direction in (J.YAxisJointDirection, J.ZAxisJointDirection, J.XAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(point))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.3
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        m = [[sum(r1[a][k] * r0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
        w = (m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1])
        if abs(w[1]) > 0.3 and abs(w[0]) < 0.01 and abs(w[2]) < 0.01:
            j.name = name
            return j
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about Y' % moving.name)


def parts_list(root):
    """{part: count} from Fusion's components: the side (L, R) and the number dropped from each name."""
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        w = o.component.name.split()
        if w[-1].isdigit():
            w = w[:-1]
        if w[-1] in ('L', 'R'):
            w = w[:-1]
        n = ' '.join(w)
        count[n] = count.get(n, 0) + 1
    return count


# ---------- the build ----------
def build(d_body=48.0, D=80.0, t=8.0, d_seat=65.0):
    Dz = design(d_body, D, t, d_seat)
    refuse(Dz)
    L = layout(Dz)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    H = mp.new_occ(root, 'Housing')
    H.isGrounded = True
    E = {'L': mp.new_occ(root, 'Wheel end L'), 'R': mp.new_occ(root, 'Wheel end R')}
    housing_parts(H, L)
    for s, tag in ((1, 'L'), (-1, 'R')):
        wheel_end_parts(E[tag], L, Dz, s, tag)
        turns_about_y(root, E[tag], H, 'Half shaft %s turns' % tag)
    for o in H.component.occurrences:
        if o.component.name in ('Banjo housing', 'Rear cover'):
            mp.colour(des, o, 'Paint - Enamel Glossy (Grey)')
    app.activeViewport.fit()
    pl = parts_list(root)
    w, h, b = Dz['w'], Dz['h'], Dz['b']
    worst = w['cases'][w['worst']]
    brg = w['bearing']
    return ('Rear axle housing built: a banjo housing for the carrier of RearAxleCarrier.py, its drum %.2f mm from the '
            'axle (%.2f mm of ground clearance on the %.0f mm static radius, at least %.0f), the carrier\'s plate at '
            'x = %.0f on %d studs M%d; two semi-floating wheel ends turning in it.\n'
            'Half shaft, 4140 Q&T 425 C, body %g mm, on supports %.0f mm apart: %.2f against yield in the hardest turn '
            '(%.0f N m with %.0f N m), %.2f in fatigue (DE-Goodman), %.2f in torsion at the splines\' %.0f mm minor '
            'diameter.\n'
            'Wheel bearing %s, %d x %d x %d: %.1f million turns against %.2f million; in the hardest turn '
            'Fe = %.2f kN (Eq. 11-12, X %.2f, Y %.2f); pressed against a shoulder of %d mm.\n'
            'Tube %g x %g, 1035 HR: %.1f MPa at the spring seat in the pothole, factor %.2f.\n\n'
            'Parts: %d components in %d kinds; %s.\n\n[%s]'
            % (L['R_out'], L['clearance'], b['r_stat'], L['clear_min'], JOINT_X, len(ad.joint_bolts()), CSTUD,
               Dz['d_body'], w['L'], w['n_yield'], worst['M'] / 1000.0, Dz['d']['T_shaft'], w['n_fatigue'],
               w['n_spline'], L['spline'][0],
               designation(brg['d']), brg['d'], brg['D'], brg['B'], w['L10_rev'] / 1e6, w['rev_needed'] / 1e6,
               w['corner_Fe'] / 1000.0, w['corner_X'], w['corner_Y'], brg['dS'],
               h['D'], h['t'], h['sigma'], h['n'],
               sum(pl.values()), len(pl), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('d_body', "The half shaft's body d_body (mm)", 48.0), ('D', "The tube's outside diameter D (mm)", 80.0),
          ('t', "The tube's wall t (mm)", 8.0), ('d_seat', "The wheel bearing's bore (mm)", 65.0)]


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
        cid = 'AxleHousingCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Axle housing', 'The rear axle housing and wheel ends')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('AxleHousing could not start:\n' + traceback.format_exc())
