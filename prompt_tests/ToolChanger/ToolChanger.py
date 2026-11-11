# -*- coding: utf-8 -*-
# =============================================================================
# ToolChanger.py - the palletizer's mechanical interface (Volume 2, Chapter 44): the robot's plate under J4's RV-80N
# to ISO 9409-1-160-6-M10 and on it an automatic tool changer: the master (body, cover, six M10 cap screws, two
# 12 mm pins), a double-acting piston whose rod passes a gland down to a cam, six segments the cam drives out into
# the tool plate's ring, and the tool plate with the same ISO pattern underneath for the gripper. Shown locked or
# free; set_state() places the moving groups by their transforms (no joints: the lock's states are poses).
#
# Every size of the standard, the changer and the lock comes from projects/palletizer/lib/tool_changer.py
# (ISO9409, CHANGER, GEOM); the few the library does not give are named below as this script's own choices.
# World in millimetres, the interface's axis along Z, the robot's face at z = 0, +Xm along X. The API works in cm.
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import math
import os
import re
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % __file__)
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP          # noqa: E402  puts every lib/ folder on sys.path
import mdfc_parts as MP               # noqa: E402
import tool_changer as TC             # noqa: E402
importlib.reload(MP)
importlib.reload(TC)

VERSION = 'ToolChanger v1.0 (prompt test, 2026-11-10)'

app = adsk.core.Application.get()
ui = app.userInterface
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create

# ---------- derived from tool_changer.py, never typed ----------
GM = TC.GEOM                          # every radius and height of the lock (mm)
CH = TC.CHANGER                       # the changer as designed (SI)
D1 = CH['iso']                        # the pitch circle, 160
ISO = TC.ISO9409[D1]                  # Table 1 of ISO 9409-1 for it
THREAD_D = TC.THREAD[ISO['d4']][0]    # M10 -> 10
HOLES = TC.iso_holes(D1)              # six at 30 + 60 k deg
R_OUT = ISO['d2'] / 2.0               # d2 200 h8
R_REC = ISO['d3'] / 2.0               # d3 100 H7
R_PIN_HOLE = ISO['d5'] / 2.0          # d5 10 H7, on +Xm
R_PC = D1 / 2.0                       # 80
N_SEG = CH['n_seg']
SEG_W = CH['seg_w'] * 1000.0          # 36
SEG_H = CH['seg_h'] * 1000.0          # 16
TRAVEL = CH['travel'] * 1000.0        # 3
BETA = CH['beta']                     # the ring's 15 deg face
LEAD = CH['alpha_free']               # the cam's 30 deg lead
FACE_RI, FACE_RO = CH['face_Ri'] * 1000.0, CH['face_Ro'] * 1000.0     # the coupling face's ring R 90 to 100
H_FLANGE = CH['h_flange']             # the master's flange the screws clamp, 20

# ---------- given in the prompt ----------
Z_ROD_END = -76.0                     # the rod's end, with the cam on it, locked (-56 free)
PASS_D, PASS_ANG = 10.0, (60.0, 120.0, 240.0, 300.0)      # service passages on the 160 circle
PIN_D, PIN_OUT, PIN_ANG = 12.0, 12.0, (0.0, 180.0)        # the master's pins

# ---------- this script's own choices (the library does not give them) ----------
PLATE_T = 25.0                        # the robot plate's thickness
TAP_DEPTH = 18.0                      # the M10 holes' depth (at least t1; the screw engages 1.5 d = 15)
REC_DEPTH = ISO['t1']                 # the 100 H7 recess, t1 = 8 deep
PIN_HOLE_DEPTH = ISO['t2']            # the 10 H7 pin hole, t2 = 10 deep
SPIGOT_CLEAR, SPIGOT_H = 0.015, ISO['t1'] - 0.5           # the cover's spigot, 100 g6, 7.5 high
SCREW_L = H_FLANGE + 1.5 * THREAD_D   # M10 x 35, as the library's bolted joint takes it
Z_HEAD = -H_FLANGE                    # the heads' bearing face: the cover's 7 and 13 of the body clamped
CB_R, CL_R = 8.75, 5.5                # counterbore 17.5 and clearance 11 for M10 (ISO 4762)
PIN_IN = 12.0                         # the pins pressed 12 mm into the master
CLEAR = 0.1                           # radial running clearance of the piston, the gland and the windows
CAM_BORE = GM['R_piston'] + 0.5       # the cam's bore under the gland
GROOVE_FLOOR = GM['z_seg'][0] - 2.0   # the tool plate's groove, 2 mm below the segments
WEB_TOP = GM['z_spigot'] - 1.0        # the web clear of the spigot's end by 1 mm
FACE_RELIEF = 0.5                     # the tool plate's top inside R 90, so only R 90 to 100 touches
LEAD_END = GM['R_seg_in'] - TRAVEL - 0.1                  # the lead starts inside the free segments
Z_CAM_MIN = GM['z_spigot'] - 0.5      # the cam never reaches the web
MAT = 'Steel'
SCREW_NAME = 'Cap screw ISO 4762 M10 x %d' % round(SCREW_L)
PIN_NAME = 'Pin ISO 8734 12 x %d' % round(PIN_OUT + PIN_IN)

_handlers = []
_last = {}
_cache = {}


# ---------- temporary bodies (world mm) ----------
def zcyl(x, y, z0, z1, r):
    return MP.cyl((x, y, z0), (x, y, z1), r)


def zcone(z0, r0, z1, r1):
    return MP.tbm().createCylinderOrCone(P3(0, 0, z0 * CM), r0 * CM, P3(0, 0, z1 * CM), r1 * CM)


def polar(r, a_deg):
    a = math.radians(a_deg)
    return r * math.cos(a), r * math.sin(a)


def rbox(th_deg, x0, x1, hw, z0, z1):
    """A box from x0 to x1 along the direction th_deg, hw either side of it, z0 to z1."""
    c, s = math.cos(math.radians(th_deg)), math.sin(math.radians(th_deg))
    xm = (x0 + x1) / 2.0
    obb = adsk.core.OrientedBoundingBox3D.create(P3(xm * c * CM, xm * s * CM, (z0 + z1) / 2.0 * CM), V3(c, s, 0),
                                                 V3(-s, c, 0), (x1 - x0) * CM, 2 * hw * CM, (z1 - z0) * CM)
    return MP.tbm().createBox(obb)


def r_chamfer(z):
    """The 15 deg face of the lip (and of the segments): through (R_bore, z_chamfer), falling outward, so that a
    segment driven out lifts the tool plate onto the master."""
    return GM['R_bore'] - (z - GM['z_chamfer']) / math.tan(math.radians(BETA))


def chamfer_cone():
    z0, z1 = GROOVE_FLOOR - 1.0, GM['z_seg'][1] + 1.0
    return zcone(z0, r_chamfer(z0), z1, r_chamfer(z1))


def r_cam(z, alpha):
    """The cam's locking zone (and the segments' inner faces), alpha to the axis, R_seg_in at the segments' top."""
    return GM['R_seg_in'] + (z - GM['z_seg'][1]) * math.tan(math.radians(alpha))


# ---------- the parts ----------
def robot_plate():
    b = zcyl(0, 0, 0.0, PLATE_T, R_OUT)
    MP.cut(b, zcyl(0, 0, -1.0, REC_DEPTH, R_REC))
    for a in HOLES:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, -1.0, TAP_DEPTH, THREAD_D / 2.0))
    x, y = polar(R_PC, 0.0)
    MP.cut(b, zcyl(x, y, -1.0, PIN_HOLE_DEPTH, R_PIN_HOLE))
    return b


def cover():
    z0, z1 = GM['z_cover']
    b = zcyl(0, 0, z0, z1, R_OUT)
    MP.union(b, zcyl(0, 0, z1 - 1.0, z1 + SPIGOT_H, R_REC - SPIGOT_CLEAR))
    for a in HOLES:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, z0 - 1.0, z1 + 1.0, CL_R))
    for a in PASS_ANG:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, z0 - 1.0, z1 + 1.0, PASS_D / 2.0))
    return b


def master_body():
    zc, zf, zs = GM['z_cover'][0], GM['z_face'], GM['z_spigot']
    b = zcyl(0, 0, zf, zc, R_OUT)
    MP.union(b, zcyl(0, 0, zs, zf + 1.0, GM['R_spigot']))
    MP.cut(b, zcyl(0, 0, GM['z_bore'][0], zc + 1.0, GM['R_piston']))                       # the piston's bore
    MP.cut(b, zcyl(0, 0, GM['z_gland'][0] - 1.0, GM['z_gland'][1] + 1.0, GM['R_rod'] + CLEAR))   # the gland
    MP.cut(b, zcyl(0, 0, zs - 1.0, GM['z_gland'][0], CAM_BORE))                          # the cam's bore
    for k in range(N_SEG):                                                                # straight windows
        MP.cut(b, rbox(360.0 * k / N_SEG, 40.0, GM['R_spigot'] + 5.0, SEG_W / 2.0 + CLEAR,
                       GM['z_seg'][0], GM['z_seg'][1] + CLEAR))
    for a in HOLES:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, Z_HEAD - 1.0, zc + 1.0, CL_R))
        MP.cut(b, zcyl(x, y, zf - 1.0, Z_HEAD, CB_R))
    for a in PIN_ANG:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, zf - 1.0, zf + PIN_IN, PIN_D / 2.0))
    for a in PASS_ANG:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, zf - 1.0, zc + 1.0, PASS_D / 2.0))
    for a, z in zip(GM['a_ports'], GM['z_ports']):                                        # release, lock
        x0, y0 = polar(GM['R_piston'] - 5.0, a)
        x1, y1 = polar(R_OUT + 1.0, a)
        MP.cut(b, MP.cyl((x0, y0, z), (x1, y1, z), GM['port_d'] / 2.0))
    return b


def piston_with_rod():
    zp = GM['z_piston']
    b = zcyl(0, 0, zp, zp + GM['piston_h'], GM['R_piston'] - CLEAR)
    return MP.union(b, zcyl(0, 0, Z_ROD_END, zp + 1.0, GM['R_rod']))


def cam(alpha):
    z_top, z_bot = GM['z_seg'][1], GM['z_seg'][0]
    r5 = r_cam(z_bot, alpha)
    tl = math.tan(math.radians(LEAD))
    zb = max(z_bot - (r5 - LEAD_END) / tl, Z_CAM_MIN)
    b = zcone(Z_ROD_END, r_cam(Z_ROD_END, alpha), z_bot, r5)                               # the locking zone
    return MP.union(b, zcone(z_bot, r5, zb, r5 - (z_bot - zb) * tl))                       # the lead


def segment(k, alpha):
    th = 360.0 * k / N_SEG
    z0, z1 = GM['z_seg']
    s = zcyl(0, 0, z0, z1, GM['R_seg_out'])
    MP.intersect(s, chamfer_cone())
    MP.cut(s, zcone(z0 - 1.0, r_cam(z0 - 1.0, alpha), z1 + 1.0, r_cam(z1 + 1.0, alpha)))
    return MP.intersect(s, rbox(th, 0.0, GM['R_seg_out'] + 4.0, SEG_W / 2.0, z0 - 1.0, z1 + 1.0))


def tool_plate():
    zf, zt = GM['z_face'], GM['z_tool']
    b = zcyl(0, 0, zt, zf, R_OUT)
    MP.cut(b, zcyl(0, 0, zf - FACE_RELIEF, zf + 1.0, FACE_RI))                             # touches on R 90-100 only
    MP.cut(b, zcyl(0, 0, WEB_TOP, zf + 1.0, GM['R_bore']))                                  # the bore over the web
    MP.cut(b, MP.intersect(zcyl(0, 0, GROOVE_FLOOR, GM['z_seg'][1] - 1.0, GM['R_groove']), chamfer_cone()))
    MP.cut(b, zcyl(0, 0, zt - 1.0, GM['z_web'][0], R_REC))                                  # the ISO recess below
    for a in HOLES:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, zt - 1.0, zt + TAP_DEPTH, THREAD_D / 2.0))
    x, y = polar(R_PC, 0.0)
    MP.cut(b, zcyl(x, y, zt - 1.0, zt + PIN_HOLE_DEPTH, R_PIN_HOLE))
    for a in PIN_ANG:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, zf - PIN_OUT - 4.0, zf + 1.0, PIN_D / 2.0 + 0.05))
    for a in PASS_ANG:
        x, y = polar(R_PC, a)
        MP.cut(b, zcyl(x, y, zt - 1.0, zf + 1.0, PASS_D / 2.0))
    return b

# ---------- the hard constraints, before anything is built ----------
def clamp_need():
    """The clamp that keeps the coupling face (R 90 to 100) closed over the standard cycle, at the changer's face."""
    if 'need' not in _cache:
        cyc = TC.cycle_loads(depth=CH['depth'])
        _cache['need'] = max(TC.face_clamp_needed(x['Fa'], x['M'], CH['face_Ri'], CH['face_Ro'])[0] for x in cyc)
    return _cache['need']


def calculate(alpha, p_bar):
    if alpha <= 0.0:
        raise ValueError('Refused: the locking angle must be above 0 deg (%.2f given)' % alpha)
    a_hold = math.degrees(math.atan(TC.MU_MIN))
    if alpha >= a_hold:
        raise ValueError('Refused: a locking angle of %.2f deg is not held by friction as low as %.2f (the cam holds '
                         'itself only under atan %.2f = %.2f deg): the lock would let go when the air fails'
                         % (alpha, TC.MU_MIN, TC.MU_MIN, a_hold))
    p = p_bar * 1e5
    Fp = math.pi / 4.0 * CH['D_piston'] ** 2 * p + CH['spring']
    Fc = TC.lock_clamp(Fp, alpha, BETA, TC.MU)
    need = clamp_need()
    if Fc < 1.5 * need:
        raise ValueError('Refused: at %.2f bar the clamp F_c = F_p / (tan(alpha + phi) tan(beta + phi)) = %.1f kN falls '
                         'under 1.5 times the %.1f kN that keeps the coupling face closed over the standard cycle '
                         '(%.1f kN wanted)' % (p_bar, Fc / 1e3, need / 1e3, 1.5 * need / 1e3))
    A_rel = math.pi * ((GM['R_piston'] / 1e3) ** 2 - (GM['R_rod'] / 1e3) ** 2)
    return dict(alpha=alpha, p_bar=p_bar, a_hold=a_hold, Fp=Fp, Fc=Fc, need=need, n=Fc / need,
                F_rel=A_rel * p, phi=math.degrees(math.atan(TC.MU)),
                bolt=TC.bolt_joint(D1, H_FLANGE, CH['cls']))


# ---------- the assembly ----------
def _place(occ, x=0.0, y=0.0, z=0.0):
    m = adsk.core.Matrix3D.create()
    m.translation = V3(x * CM, y * CM, z * CM)
    occ.transform2 = m


def set_state(*args, **kw):
    """set_state(groups, locked, tool_down=0) (or set_state(locked, tool_down) on the last build): the piston and its
    cam 20 mm up when free, each segment in by its travel when free, the tool plate lowered tool_down mm."""
    args = list(args)
    G = args.pop(0) if args and isinstance(args[0], dict) else _last['groups']
    locked = args.pop(0) if args else kw.get('locked', True)
    tool_down = args.pop(0) if args else kw.get('tool_down', 0.0)
    _place(G['piston'], z=0.0 if locked else GM['stroke'])
    for k, o in enumerate(G['segments']):
        th = math.radians(360.0 * k / N_SEG)
        d = 0.0 if locked else -TRAVEL
        _place(o, d * math.cos(th), d * math.sin(th), 0.0)
    _place(G['tool'], z=-float(tool_down))
    G['state'] = (bool(locked), float(tool_down))
    return G


def parts_list(root):
    """{kind: count} of the parts (components with bodies); Segment 1 to 6 counted as one kind."""
    out = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count:
            k = re.sub(r'\s+\d+$', '', o.component.name)
            out[k] = out.get(k, 0) + 1
    return out


def _group(root, name, grounded=False):
    occ = MP.new_occ(root, name)
    if grounded:
        occ.isGrounded = True
    return occ


def _part(parent_occ, name, tb, t=(0.0, 0.0, 0.0)):
    occ = MP.new_occ(parent_occ.component, name, t)
    MP.add_body(occ, tb, name, MAT)
    return occ


def _repeat(parent_occ, first, positions):
    for x, y in positions:
        m = adsk.core.Matrix3D.create()
        m.translation = V3(x * CM, y * CM, 0.0)
        o = parent_occ.component.occurrences.addExistingComponent(first.component, m)
        o.isGroundToParent = True


def build(alpha=CH['alpha'], p_bar=CH['p_min'] / 1e5, locked=True):
    calc = calculate(float(alpha), float(p_bar))           # refuses before any document is made
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    root = des.rootComponent

    robot = _group(root, 'Robot plate', grounded=True)
    MP.add_body(robot, robot_plate(), 'Robot plate ISO 9409-1-%d-%d-%s' % (D1, ISO['n'], ISO['d4']), MAT)

    master = _group(root, 'Master', grounded=True)
    _part(master, 'Master body', master_body())
    _part(master, 'Cover', cover())
    x0, y0 = polar(R_PC, HOLES[0])
    s0 = MP.new_occ(master.component, SCREW_NAME, (x0, y0, 0.0))
    MP.add_body(s0, MP.cap_screw(THREAD_D, SCREW_L, (x0, y0, Z_HEAD), (0.0, 0.0, 1.0)), SCREW_NAME, MAT)
    _repeat(master, s0, [polar(R_PC, a) for a in HOLES[1:]])
    px, py = polar(R_PC, PIN_ANG[0])
    p0 = MP.new_occ(master.component, PIN_NAME, (px, py, 0.0))
    MP.add_body(p0, zcyl(px, py, GM['z_face'] - PIN_OUT, GM['z_face'] + PIN_IN, PIN_D / 2.0), PIN_NAME, MAT)
    _repeat(master, p0, [polar(R_PC, a) for a in PIN_ANG[1:]])

    piston = _group(root, 'Piston')
    _part(piston, 'Piston with rod', piston_with_rod())
    _part(piston, 'Cam', cam(calc['alpha']))

    segs = []
    for k in range(N_SEG):
        o = _group(root, 'Segment %d' % (k + 1))
        MP.add_body(o, segment(k, calc['alpha']), 'Segment %d' % (k + 1), MAT)
        segs.append(o)

    tool = _group(root, 'Tool plate')
    MP.add_body(tool, tool_plate(), 'Tool plate ISO 9409-1-%d-%d-%s' % (D1, ISO['n'], ISO['d4']), MAT)

    G = dict(robot=robot, master=master, piston=piston, segments=segs, tool=tool)
    _last.clear()
    _last.update(groups=G, design=des, calc=calc)

    # measured: the rod's end and the piston's face and top, locked and free, from the body in the assembly
    rod = None
    for o in piston.childOccurrences:
        if o.component.name == 'Piston with rod':
            rod = o
    meas = {}
    for st in (True, False):
        set_state(G, st)
        bb = rod.bRepBodies.item(0).boundingBox if rod else None
        if bb:
            meas[st] = (bb.minPoint.z * 10, bb.maxPoint.z * 10 - GM['piston_h'], bb.maxPoint.z * 10)
    set_state(G, bool(locked))
    _last['measured'] = meas
    report(root, G, calc, meas, bool(locked))
    return G


def report(root, G, calc, meas, locked):
    bj = calc['bolt']
    pl = parts_list(root)
    zg0, zg1 = GM['z_gland']
    z_rel, z_lock = GM['z_ports']
    hp = GM['port_d'] / 2.0
    lines = [VERSION,
             'ISO 9409-1-%d-%d-%s on the robot plate and under the tool plate: d2 %d h8, d3 %d H7 (%g deep), '
             '%d x %s at %s deg on %d, pin hole d5 %d H7 on +Xm (t1 %g, t2 %g)'
             % (D1, ISO['n'], ISO['d4'], ISO['d2'], ISO['d3'], REC_DEPTH, ISO['n'], ISO['d4'],
                '/'.join('%g' % a for a in HOLES), D1, ISO['d5'], ISO['t1'], ISO['t2']),
             'Screws: %d x %s, class %s, through the cover and the master\'s %g mm flange into the robot plate: '
             'preload Fi = 0.75 Sp At = %.1f kN each, joint constant C = %.3f'
             % (ISO['n'], SCREW_NAME, CH['cls'], H_FLANGE, bj['Fi'] / 1e3, bj['C']),
             'Piston: D %g double-acting, rod %g, stroke %g; at %.2f bar it pushes %.2f kN locking and pulls %.2f kN '
             'releasing' % (CH['D_piston'] * 1e3, 2 * GM['R_rod'], GM['stroke'], calc['p_bar'], calc['Fp'] / 1e3,
                             calc['F_rel'] / 1e3),
             'Angles: cam %.2f deg (holds itself: under atan %.2f = %.2f deg), lead %g deg, ring face %g deg; '
             'phi = atan %.2f = %.2f deg' % (calc['alpha'], TC.MU_MIN, calc['a_hold'], LEAD, BETA, TC.MU, calc['phi']),
             'Clamp: F_c = %.1f kN against the cycle\'s need %.1f kN (1.5 x = %.1f kN): %.2f times'
             % (calc['Fc'] / 1e3, calc['need'] / 1e3, 1.5 * calc['need'] / 1e3, calc['n'])]
    for st in (True, False):
        if st in meas:
            z_end, z_face, z_top = meas[st]
            lines.append('%s: rod end %.1f (gland %g to %g), piston face %.1f over the release port %g, top %.1f '
                         'under the lock port %g: %s'
                         % ('Locked' if st else 'Free', z_end, zg0, zg1, z_face, z_rel, z_top, z_lock,
                            'ok' if (z_end < zg0 and z_face > z_rel + hp and z_top < z_lock - hp) else 'WRONG'))
    mass = {}
    for o in root.occurrences:
        k = re.sub(r'\s+\d+$', '', o.component.name)
        mass[k] = mass.get(k, 0.0) + o.physicalProperties.mass
    lines.append('Weighed: ' + ', '.join('%s %.2f kg' % (k, v) for k, v in mass.items()))
    lines.append('Parts (%d): ' % sum(pl.values()) + ', '.join('%s %d' % (k, v) for k, v in sorted(pl.items())))
    lines.append('Shown %s' % ('locked' if locked else 'free'))
    ui.messageBox('\n'.join(lines), 'ToolChanger')


# ---------- the dialog ----------
class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.command.commandInputs
            build(alpha=ins.itemById('alpha').value, p_bar=ins.itemById('p_bar').value,
                  locked=ins.itemById('state').selectedItem.name == 'Locked')
        except ValueError as e:
            ui.messageBox(str(e), 'ToolChanger')
        except Exception:
            ui.messageBox('ToolChanger failed:\n%s' % traceback.format_exc(), 'ToolChanger')


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if __name__ != 'mdfc_check':
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            ins.addFloatSpinnerCommandInput('alpha', "The cam's locking angle (deg)", '', 0.1, 30.0, 0.1, CH['alpha'])
            ins.addFloatSpinnerCommandInput('p_bar', "The supply's least pressure (bar)", '', 0.1, 12.0, 0.1,
                                            CH['p_min'] / 1e5)
            dd = ins.addDropDownCommandInput('state', 'The state', adsk.core.DropDownStyles.TextListDropDownStyle)
            dd.listItems.add('Locked', True)
            dd.listItems.add('Free', False)
            for h, ev in ((_Execute(), cmd.execute), (_Destroy(), cmd.destroy)):
                ev.add(h)
                _handlers.append(h)
        except Exception:
            ui.messageBox('ToolChanger dialog failed:\n%s' % traceback.format_exc(), 'ToolChanger')


CMD_ID = 'mdfc_ToolChanger_prompt_test'


def run(context):
    try:
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Tool changer', 'ISO 9409-1-160-6-M10 tool changer')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if __name__ != 'mdfc_check':
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ToolChanger failed:\n%s' % traceback.format_exc(), 'ToolChanger')
