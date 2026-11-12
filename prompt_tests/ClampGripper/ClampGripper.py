# =============================================================================
# ClampGripper.py  -  Chapter 45 of Volume 2 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer's
# parallel-jaw gripper for cartons. An adapter to ISO 9409-1-160-6-M10 under the tool changer's tool plate, an
# aluminium beam with two HGR25 rails, two jaws on HGH25CA blocks that squeeze a pack of cartons between rubber pads,
# one ballscrew (right-hand on +x, left-hand on -x) that moves the jaws together, its BK25 and BF25 bearing units, a
# motor with a gearhead and a brake; and the pack it grips. Written from projects/palletizer/ClampGripper/PROMPT.txt.
#
#   INPUTS        the dialog: cartons along (nx), cartons across (ny), the squeeze N, the state (gripping or open)
#   DERIVED       projects/palletizer/lib/clamp_gripper.py: CARTON, LAY, SCREW, BK25, BF25, GUIDE, squeeze_needed();
#                 common/BallScrewAxis/BallScrewAxis.py: groove_section() for every ball groove
#   HARD RULES    a pack over 160 kg, more than three along or two across; a squeeze over 5000 N, under the pack's
#                 need, or one that presses the sides over 40 kPa
#   CONVENTIONS   a new parametric design; mm in the world, z = 0 the mounting face, z up, x the squeeze, y across;
#                 groups Adapter, Frame, Screw (grounded), Jaw A (+x), Jaw B (-x), Pack; set_state() places the jaws
#                 by their transforms; no joints
#   ACCEPTANCE    the message; no interference gripping, open and over the jaws' range; the grooves' hands; the
#                 adapter's ISO pattern; the groups weighed; four refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import math
import os
import re
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    """The first folder above this script that holds common/companion_paths.py."""
    here = os.path.dirname(os.path.abspath(__file__))
    d = here
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('No common/companion_paths.py in any folder above %s' % here)
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP          # noqa: E402  every lib/ folder is now on sys.path
import mdfc_parts as MP               # noqa: E402  the parts library
import clamp_gripper as CG            # noqa: E402  the design calculations of Chapter 45
for _mod in (MP, CG):
    importlib.reload(_mod)            # Fusion keeps modules: pick up any edit of the libraries


def _load(path, name):
    """A script of the companion loaded as a module (its run() is not called)."""
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


BSA = _load(os.path.join(CP.ROOT, 'common', 'BallScrewAxis', 'BallScrewAxis.py'), 'mdfc_BallScrewAxis')

VERSION = 'ClampGripper 1.0 prompt test (2026-11-11)'
CM = 0.1                                    # the API works in cm: mm x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

# ---------- derived: the library's data ----------
LAY, SC, CARTON = CG.LAY, CG.SCREW, CG.CARTON
BK, BF, GD = CG.BK25, CG.BF25, CG.GUIDE
ZS = LAY['z_screw']                         # mm, the screw's axis

# ---------- the lengths along x, from the prompt ----------
BEAM_X = (-1000.0, 900.0)                   # the beam's ends
X_BK, X_BF = -880.0, 880.0                  # the fixed and the supported bearing units' middles
X_BRACKET = -1000.0                         # the motor bracket's face against the beam's end
GROOVE_RH = (215.0, 760.0)
GROOVE_LH = (-760.0, -215.0)
J_MIN, J_MAX = LAY['j_min'], LAY['j_max']   # the jaws' range, 200 to 650 mm

# ---------- the hard limits, from the prompt ----------
M_PACK_MAX = 160.0                          # kg
NX_MAX, NY_MAX = 3, 2

# ---------- the adapter, ISO 9409-1-160-6-M10 ----------
AD_D, AD_T = 200.0, 16.0
SPIGOT_D = 100.0 - 0.023                    # 100 g6 (es -0.012, ei -0.034, ISO 286) drawn at the middle of its zone
SPIGOT_H = 8.0                              # standing into the tool plate's recess
PCD = 160.0
ISO_ANGLES = [30.0 + 60.0 * k for k in range(6)]          # from +Xm (world +x) about +z
M10_HOLE, M10_CB, M10_CB_DEEP, M10_L = 11.0, 18.0, 11.0, 25.0   # ISO 273 medium; a counterbore for an ISO 4762 head
PIN_D, PIN_DEEP, PIN_UP = 10.0, 12.0, 8.0   # the pin on +Xm on the 160 circle, 12 in the adapter, 8 standing up
M8_BEAM = [(45.0, 45.0), (45.0, -45.0), (-45.0, 45.0), (-45.0, -45.0)]   # the beam's four M8 into the adapter
M8_BEAM_L = 20.0
FIT = 0.005                                 # mm: where parts fit line to line, the hole is drawn this much larger

# ---------- the beam, rails, blocks ----------
Z_TOP, Z_BOT = LAY['frame']                 # -16, -116
T_TOP, T_BOT, T_SIDE = LAY['walls']
RAIL_Y = LAY['rail_y']
RAIL_X = (-LAY['rail_len'] / 2.0, LAY['rail_len'] / 2.0)
H1 = 5.5                                    # mm, HGH25CA: the block's clearance to the rail's base (HIWIN)
SLOT = 1.0                                  # mm, the blocks drawn clear of the rails all round

# ---------- the jaws ----------
PAD_T, ARM_T = LAY['pad_t'], LAY['arm_t']
RIB_W, RIB_H = LAY['rib']
RIB_Y = LAY['rib_y']
CAR_L, CAR_W = LAY['carriage_len'], LAY['carriage_w']
D_BLK = LAY['d_blocks']
JAW_W = LAY['jaw_w']
ARM_HOLE = 36.0                             # mm, the screw's clearance through the arm
NUT_PCD = (SC['D'] + SC['D1']) / 2.0        # the flange's six M8 between the nut's body and the flange's rim
NUT_M8_L = 25.0
NUT_ANGLES = [30.0 + 60.0 * k for k in range(6)]          # from +y towards +z round the screw's axis

# ---------- the ball groove (BallScrewAxis.ballscrew()'s proportions on the library's core and ball) ----------
R_G = 0.52 * SC['Dw']                       # the groove's radius
D_M = SC['dk'] + 2.0 * R_G                  # the balls' circle, 33.65 mm on the core of 28.7
R_LAND = (SC['d'] - 0.4) / 2.0              # the land's radius
R_BORE = (SC['d'] + 1.6) / 2.0              # the nut's bore
CHUNK = 200.0                               # mm, the longest single sweep of a groove (20 turns)

# ---------- the drive beyond the bracket ----------
BRACKET_T, BRACKET_Z0 = 15.0, -280.0
LANTERN = (70.0, 45.0, 37.0)                # length, outer and inner radius
COUPLING = (45.0, 20.0)                     # length, radius
GEARHEAD = (100.0, 45.0, 8.0)               # length, radius, output shaft radius
MOTOR = (200.0, 90.0)                       # length, square
BRAKE = (50.0, 40.0)                        # length, radius
X_DRIVE_END = -1045.0                       # the screw's drive end, inside the coupling

ALU = ('Aluminum 6061', 'Aluminum', 'Aluminum 6061-T6')
STEEL = ('Steel', 'Steel, Alloy', 'Steel AISI 1045 207 HB')
IRON = ('Iron, Cast', 'Cast Iron', 'Steel')
RUBBER = ('Rubber, Nitrile', 'Rubber', 'Rubber, Silicone', 'Rubber, Natural', 'Rubber, Butyl')
GREY, STEEL_L, BLUE = 'Paint - Enamel Glossy (Grey)', 'Steel - Satin', 'Paint - Enamel Glossy (Blue)'
RED, YELLOW = 'Paint - Enamel Glossy (Red)', 'Paint - Enamel Glossy (Yellow)'

GROUPS = ('Adapter', 'Frame', 'Screw', 'Jaw A', 'Jaw B', 'Pack')
_last = {}                                  # what the last build made: its groups and the jaws' positions


# ---------- small helpers ----------
_MATS = {}


def _material(names):
    """The first of these Fusion Material Library materials that exists (None if none)."""
    if names in _MATS:
        return _MATS[names]
    found = None
    app = adsk.core.Application.get()
    for libname in ('Fusion Material Library', 'Fusion 360 Material Library'):
        lib = app.materialLibraries.itemByName(libname)
        if not lib:
            continue
        for n in names:
            m = lib.materials.itemByName(n)
            if m:
                found = m
                break
        if found:
            break
    _MATS[names] = found
    return found


def _kind_name(kind, tag):
    return kind if tag is None else '%s (%s)' % (kind, tag)


def part(design, grp, kind, tag, tb, mats=None, look=None, body_name=None):
    """A component inside the group `grp` holding the temporary body `tb` (world mm, the group at rest)."""
    occ = MP.new_occ(grp.component, _kind_name(kind, tag))
    body = MP.add_body(occ, tb, body_name or kind)
    m = _material(mats) if mats else None
    if m:
        try:
            body.material = m
        except Exception:
            pass
    if look:
        MP.colour(design, occ, look)
    return occ, body


def xs(s, a, b):
    """The x range from a to b mm out from the middle on the side s (+1 for Jaw A, -1 for Jaw B), in order."""
    return (min(s * a, s * b), max(s * a, s * b))


def along(x0, x1, r, y=0.0, z=ZS):
    """A cylinder along x through (y, z)."""
    return MP.cyl((x0, y, z), (x1, y, z), r)


def nut_bolts():
    """(y, z) of the nut flange's six M8."""
    r = NUT_PCD / 2.0
    return [(r * math.cos(math.radians(a)), ZS + r * math.sin(math.radians(a))) for a in NUT_ANGLES]


def parts_list(root):
    """{kind: count} of every component that holds a body."""
    out = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count:
            k = re.sub(r' \([^()]*\)$', '', o.component.name)
            out[k] = out.get(k, 0) + 1
    return out


# ---------- the ball grooves ----------
_SECTION = []
_TWIST = {1: 1.0, -1: 1.0}       # the sign the twist value needs for each hand, measured on each groove's first sweep


def _section():
    """The groove's section square to the axis, the helix crossing angle 0 (BallScrewAxis.groove_section()). It is
    symmetric about that angle, so one section, turned to the helix's angle, serves both hands."""
    if not _SECTION:
        _SECTION.extend(BSA.groove_section(D_M / 2.0, R_G, SC['lead'], 0.0))
    return _SECTION


def _sweep(comp, body_name, xa, xb, hand):
    """One sweep of the groove from xa to xb (xa < xb) on the helix whose angle is hand 2 pi x / P about +x (hand +1
    right-hand, -1 left-hand), angles from +y towards +z. Returns a function that cuts it with a given twist sign."""
    P = SC['lead']
    ph = hand * 2.0 * math.pi * xa / P
    c, s = math.cos(ph), math.sin(ph)
    pts = [(u * c - v * s, u * s + v * c) for u, v in _section()]
    sk = comp.sketches.add(BSA.cross_plane(comp, xa))
    sk.isComputeDeferred = True
    model = [sk.modelToSketchSpace(P3(xa * CM, u * CM, (ZS + v) * CM)) for u, v in pts]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(model[0], model[1])
    prev = first.endSketchPoint
    for q in model[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    ps = comp.sketches.add(BSA.plane_at(comp, ZS))
    axis = ps.sketchCurves.sketchLines.addByTwoPoints(ps.modelToSketchSpace(P3(xa * CM, 0, ZS * CM)),
                                                      ps.modelToSketchSpace(P3(xb * CM, 0, ZS * CM)))
    ps.isVisible = False
    sw = comp.features.sweepFeatures

    def cut(sign):
        inp = sw.createInput(sk.profiles.item(0), comp.features.createPath(axis, False), CUT)
        inp.twistAngle = VI(sign * hand * 2.0 * math.pi * (xb - xa) / P)
        inp.participantBodies = [comp.bRepBodies.itemByName(body_name)]
        return sw.add(inp)
    return cut


def _hand_ok(body, xa, hand):
    """Probe the groove just cut: where the helix stands at +90 deg (a quarter turn past a whole number of leads), a
    point between the groove's bottom and the land is outside the body, and half a turn round it is inside."""
    P = SC['lead']
    x = P * (math.ceil((xa + 2.0) / P - hand * 0.25) + hand * 0.25)
    th = hand * 2.0 * math.pi * x / P
    r = (D_M / 2.0 - R_G + R_LAND) / 2.0
    pc = adsk.fusion.PointContainment

    def at(t):
        return body.pointContainment(P3(x * CM, r * math.cos(t) * CM, (ZS + r * math.sin(t)) * CM))
    return at(th) == pc.PointOutsidePointContainment and at(th + math.pi) == pc.PointInsidePointContainment


def cut_groove(comp, body_name, x0, x1, hand, probe=False):
    """The ball groove from x0 to x1 cut into the body `body_name` of `comp`, in sweeps of at most CHUNK that overlap
    by 0.5 mm. With probe, the first sweep's hand is measured and its twist reversed if Fusion turned it the other way."""
    n = max(1, int(math.ceil((x1 - x0) / CHUNK - 1e-9)))
    for i in range(n):
        xa = x0 + (x1 - x0) * i / n - (0.5 if i else 0.0)
        xb = x0 + (x1 - x0) * (i + 1) / n
        cut = _sweep(comp, body_name, xa, xb, hand)
        f = cut(_TWIST[hand])
        if probe and i == 0 and not _hand_ok(comp.bRepBodies.itemByName(body_name), xa, hand):
            f.deleteMe()
            for h in ((1, -1) if hand == 1 else (-1,)):      # a right-hand result reverses the convention for both
                _TWIST[h] = -_TWIST[h]
            f = cut(_TWIST[hand])
            if not _hand_ok(comp.bRepBodies.itemByName(body_name), xa, hand):
                raise RuntimeError('The %s-hand groove could not be cut with either twist.'
                                   % ('right' if hand == 1 else 'left'))


# ---------- the refusals ----------
def refuse(nx, ny, N):
    """The hard constraints; returns squeeze_needed() for an accepted pack and squeeze."""
    m = CG.pack_mass(nx, ny)
    if nx < 1 or ny < 1:
        raise ValueError('A pack needs at least one carton along the squeeze and one across (%d x %d asked).'
                         % (nx, ny))
    if m > M_PACK_MAX + 1e-9 or nx > NX_MAX or ny > NY_MAX:
        raise ValueError('A pack of %d x %d cartons, %.0f kg, is refused: the gripper takes at most %.0f kg, at most '
                         'three along the squeeze and two across.' % (nx, ny, m, M_PACK_MAX))
    if N > CG.N_MAX + 1e-9:
        raise ValueError('A squeeze of %.0f N is refused: it is over the %.0f N the screw, the guides and the arms are '
                         'built for.' % (N, CG.N_MAX))
    s = CG.squeeze_needed(nx, ny)
    why = []
    if N < s['need'] - 1e-9:
        why.append('the %d x %d pack (%.0f kg) needs %.0f N (pads %.0f N, carton on carton %.0f N, the %s governing), '
                   'more than %.0f N' % (nx, ny, s['m'], s['need'], s['pad'], s['cc'], s['governs'], N))
    if N > s['N_side'] + 1e-9:
        why.append('%.0f N presses the cartons\' sides at %.1f kPa, over the %.0f kPa they accept (%.0f N at most on '
                   'this pack)' % (N, N / s['area'] / 1e3, CG.P_SIDE / 1e3, s['N_side']))
    if why:
        raise ValueError('A squeeze of %.0f N is refused: %s.' % (N, '; and '.join(why)))
    return s


# ---------- the jaws' positions ----------
def set_state(groups, j=None):
    """Place the two jaws at +-j (mm, the pads' faces) by their transforms; set_state(j) uses the last build's
    groups. The screw's two hands keep the jaws symmetric, so one number places both."""
    if j is None:
        groups, j = _last['groups'], groups
    j = float(j)
    if j < J_MIN - 1e-6 or j > J_MAX + 1e-6:
        raise ValueError('The jaws run from j = %.0f to %.0f mm; %.2f mm is outside.' % (J_MIN, J_MAX, j))
    d = j - _last['j_built']
    for name, sgn in (('Jaw A', 1.0), ('Jaw B', -1.0)):
        m = adsk.core.Matrix3D.create()
        m.translation = V3(sgn * d * CM, 0.0, 0.0)
        groups[name].transform2 = m
    _last['j'] = j
    return j


# ---------- the groups ----------
def build_adapter(design, grp):
    plate = MP.cyl((0, 0, -AD_T), (0, 0, 0), AD_D / 2.0)
    MP.union(plate, MP.cyl((0, 0, -1.0), (0, 0, SPIGOT_H), SPIGOT_D / 2.0))
    r = PCD / 2.0
    for a in ISO_ANGLES:
        x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        MP.cut(plate, MP.cyl((x, y, -AD_T - 1), (x, y, 1.0), M10_HOLE / 2.0))
        MP.cut(plate, MP.cyl((x, y, -AD_T - 1), (x, y, -AD_T + M10_CB_DEEP), M10_CB / 2.0))
    MP.cut(plate, MP.cyl((r, 0, -PIN_DEEP), (r, 0, 1.0), PIN_D / 2.0))
    for x, y in M8_BEAM:                                        # threaded for the beam's M8, from below
        MP.cut(plate, MP.cyl((x, y, -AD_T - 1), (x, y, -3.0), 4.0 + FIT))
    part(design, grp, 'Adapter plate', None, plate, ALU, GREY)
    part(design, grp, 'Pin 10 mm', None, MP.dowel(PIN_D - 2 * FIT, (r, 0, -PIN_DEEP), (r, 0, PIN_UP)),
         STEEL, STEEL_L)
    for k, a in enumerate(ISO_ANGLES):
        x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        tb = MP.cap_screw(10.0, M10_L, (x, y, -AD_T + M10_CB_DEEP), (0, 0, 1))
        part(design, grp, 'Cap screw ISO 4762 M10 x 25', 'T%d' % (k + 1), tb, STEEL, STEEL_L)


def build_frame(design, grp):
    x0, x1 = BEAM_X
    hw = LAY['frame_w'] / 2.0
    beam = MP.box(x0, x1, -hw, hw, Z_BOT, Z_TOP)
    MP.cut(beam, MP.box(x0 - 1, x1 + 1, -hw + T_SIDE, hw - T_SIDE, Z_BOT + T_BOT, Z_TOP - T_TOP))
    for x, y in M8_BEAM:
        MP.cut(beam, MP.cyl((x, y, Z_TOP - T_TOP - 1), (x, y, Z_TOP + 1), 4.5))          # the M8's clearance
        MP.cut(beam, MP.cyl((x, y, Z_BOT - 1), (x, y, Z_BOT + T_BOT + 1), 8.0))          # reached from below
    part(design, grp, 'Beam', None, beam, ALU, GREY)
    for k, (x, y) in enumerate(M8_BEAM):                       # from inside the beam up into the adapter
        tb = MP.cap_screw(8.0, M8_BEAM_L, (x, y, Z_TOP - T_TOP), (0, 0, 1))
        part(design, grp, 'Cap screw ISO 4762 M8 x 20', 'F%d' % (k + 1), tb, STEEL, STEEL_L)
    z_rail = Z_BOT - GD['HR']
    for tag, yr in (('+y', RAIL_Y), ('-y', -RAIL_Y)):
        tb = MP.box(RAIL_X[0], RAIL_X[1], yr - GD['WR'] / 2.0, yr + GD['WR'] / 2.0, z_rail, Z_BOT)
        part(design, grp, 'Rail HGR25', tag, tb, STEEL, STEEL_L)
    # the bearing units hang from seats under the beam: each unit's base h above the screw's axis
    for unit, xm, name in ((BK, X_BK, 'BK25'), (BF, X_BF, 'BF25')):
        z_base = ZS + unit['h']
        xa, xb = xm - unit['L'] / 2.0, xm + unit['L'] / 2.0
        part(design, grp, 'Seat ' + name, None, MP.box(xa, xb, -unit['B'] / 2.0, unit['B'] / 2.0, z_base, Z_BOT),
             ALU, GREY)
        house = MP.box(xa, xb, -unit['B'] / 2.0, unit['B'] / 2.0, z_base - unit['H'], z_base)
        MP.cut(house, along(xa - 1, xb + 1, unit['d'] / 2.0 + FIT))
        part(design, grp, 'Bearing unit ' + name, None, house, IRON, BLUE)
    # the motor bracket against the beam's end, and beyond it the lantern, gearhead, motor and brake
    xb0 = X_BRACKET - BRACKET_T
    br = MP.box(xb0, X_BRACKET, -hw, hw, BRACKET_Z0, Z_TOP)
    MP.cut(br, along(xb0 - 1, X_BRACKET + 1, 15.0))
    part(design, grp, 'Motor bracket', None, br, ALU, GREY)
    x_l = xb0 - LANTERN[0]
    part(design, grp, 'Lantern', None, MP.tube((x_l, 0, ZS), (xb0, 0, ZS), LANTERN[1], LANTERN[2]), ALU, GREY)
    x_g = x_l - GEARHEAD[0]
    gh = along(x_g, x_l, GEARHEAD[1])
    MP.union(gh, along(x_l - 1.0, X_DRIVE_END - 5.0, GEARHEAD[2]))
    part(design, grp, 'Gearhead 3 to 1', None, gh, STEEL, BLUE)
    x_m = x_g - MOTOR[0]
    q = MOTOR[1] / 2.0
    part(design, grp, 'Servo motor', None, MP.box(x_m, x_g, -q, q, ZS - q, ZS + q), ALU, BLUE)
    part(design, grp, 'Brake, spring-applied', None, along(x_m - BRAKE[0], x_m, BRAKE[1]), STEEL, GREY)


def build_screw(design, grp):
    jk = (X_BK - BK['L'] / 2.0 - 4.0, X_BK + BK['L'] / 2.0 + 9.0)               # the fixed bearing's journal
    jf = (X_BF - BF['L'] / 2.0 - 15.0, X_BF + BF['L'] / 2.0 + 2.0)              # the supported bearing's journal
    left = along(X_DRIVE_END, jk[0] + 1.0, 10.0)                                # the drive end, d 20
    MP.union(left, along(jk[0], jk[1] + 1.0, BK['d'] / 2.0))
    MP.union(left, along(jk[1], 0.0, R_LAND))
    right = along(0.0, jf[0] + 1.0, R_LAND)
    MP.union(right, along(jf[0], jf[1], BF['d'] / 2.0))
    occ = MP.new_occ(grp.component, 'Ballscrew R32-10')
    for tb, name in ((left, 'Left-hand half'), (right, 'Right-hand half')):
        b = MP.add_body(occ, tb, name)
        m = _material(STEEL)
        if m:
            try:
                b.material = m
            except Exception:
                pass
    MP.colour(design, occ, STEEL_L)
    comp = occ.component
    cut_groove(comp, 'Right-hand half', GROOVE_RH[0], GROOVE_RH[1], 1, probe=True)
    cut_groove(comp, 'Left-hand half', GROOVE_LH[0], GROOVE_LH[1], -1, probe=True)
    # the coupling between the screw's drive end and the gearhead's shaft, beyond the bracket
    xc1 = X_BRACKET - BRACKET_T - 10.0
    xc0 = xc1 - COUPLING[0]
    cp = along(xc0, xc1, COUPLING[1])
    MP.cut(cp, along(X_DRIVE_END - 1.0, xc1 + 1.0, 10.0 + FIT))
    MP.cut(cp, along(xc0 - 1.0, X_DRIVE_END, GEARHEAD[2] + FIT))
    part(design, grp, 'Coupling', None, cp, ALU, GREY)


def build_jaw(design, grp, s, j, tag):
    """One jaw on the side s (+1 Jaw A, right-hand nut; -1 Jaw B, left-hand nut), its pad's face at s j."""
    hw = JAW_W / 2.0
    z_car = LAY['carriage']                  # (-156, -176)
    x_in = j + PAD_T                         # the arm's inner face
    x_out = x_in + ARM_T                     # its outer face
    part(design, grp, 'Pad', tag, MP.box(*xs(s, j, x_in), -hw, hw, LAY['pad'][1], LAY['pad'][0]), RUBBER, RED)
    arm = MP.box(*xs(s, x_in, x_out), -hw, hw, LAY['arm_bottom'], z_car[1])
    for yr in (RIB_Y, -RIB_Y):
        MP.union(arm, MP.box(*xs(s, x_out - 1.0, x_out + RIB_H), yr - RIB_W / 2.0, yr + RIB_W / 2.0,
                             LAY['arm_bottom'], z_car[1]))
    MP.cut(arm, along(s * (x_in - 1.0), s * (x_out + 1.0), ARM_HOLE / 2.0))
    for y, z in nut_bolts():
        MP.cut(arm, MP.cyl((s * (x_in - 1.0), y, z), (s * (x_out + 1.0), y, z), 4.0 + FIT))
    part(design, grp, 'Arm', tag, arm, ALU, GREY)
    part(design, grp, 'Carriage', tag, MP.box(*xs(s, x_in, x_in + CAR_L), -CAR_W / 2.0, CAR_W / 2.0, z_car[1], z_car[0]),
         ALU, GREY)
    z_blk = Z_BOT - H1
    z_head = Z_BOT - GD['HR']
    k = 0
    for dx in (-D_BLK / 2.0, D_BLK / 2.0):
        xc = x_in + CAR_L / 2.0 + dx
        for yr in (RAIL_Y, -RAIL_Y):
            k += 1
            blk = MP.box(*xs(s, xc - GD['L'] / 2.0, xc + GD['L'] / 2.0), yr - GD['W'] / 2.0, yr + GD['W'] / 2.0,
                         z_car[0], z_blk)
            MP.cut(blk, MP.box(*xs(s, xc - GD['L'] / 2.0 - 1.0, xc + GD['L'] / 2.0 + 1.0),
                               yr - GD['WR'] / 2.0 - SLOT, yr + GD['WR'] / 2.0 + SLOT, z_head - SLOT, z_blk + 1.0))
            part(design, grp, 'Block HGH25CA', '%s%d' % (tag, k), blk, STEEL, STEEL_L)
    # the nut, its flange on the arm's outer face, its body outboard
    xf1 = x_out + SC['L1']
    xe = x_out + SC['L']
    nut = along(s * x_out, s * xf1, SC['D1'] / 2.0)
    MP.union(nut, along(s * (xf1 - 1.0), s * xe, SC['D'] / 2.0))
    MP.cut(nut, along(s * (x_out - 1.0), s * (xe + 1.0), R_BORE))
    for y, z in nut_bolts():
        MP.cut(nut, MP.cyl((s * (x_out - 1.0), y, z), (s * (xf1 + 1.0), y, z), 4.5))
    hand = 'right-hand' if s > 0 else 'left-hand'
    occ, _ = part(design, grp, 'Nut R32-10K5-DEB ' + hand, tag, nut, STEEL, BLUE, body_name='Nut')
    cut_groove(occ.component, 'Nut', *xs(s, x_out - 1.0, xe + 1.0), hand=1 if s > 0 else -1)
    for n, (y, z) in enumerate(nut_bolts()):
        tb = MP.cap_screw(8.0, NUT_M8_L, (s * xf1, y, z), (-s, 0, 0))
        part(design, grp, 'Cap screw ISO 4762 M8 x 25', '%s%d' % (tag, n + 1), tb, STEEL, STEEL_L)


def _carton_material(design, body):
    """A copy of a library material in the design, its density set so that a carton weighs CARTON['m']."""
    try:
        mat = design.materials.itemByName('Carton of drinks')
        if mat is None:
            base = _material(('Paper', 'Cardboard', 'Wood, Pine', 'ABS Plastic')) or _material(STEEL)
            mat = design.materials.addByCopy(base, 'Carton of drinks')
            body.material = mat
            prop = mat.materialProperties.itemById('structural_Density')
            mass = body.physicalProperties.mass
            if prop and mass > 0:
                prop.value = prop.value * CARTON['m'] / mass
        else:
            body.material = mat
    except Exception:
        pass


def build_pack(design, grp, nx, ny):
    l, w, h = CARTON['l'] * 1000.0, CARTON['w'] * 1000.0, CARTON['h'] * 1000.0
    z1 = LAY['pack_top']
    n = 0
    for i in range(nx):
        for k in range(ny):
            n += 1
            x0, y0 = -nx * l / 2.0 + i * l, -ny * w / 2.0 + k * w
            occ, body = part(design, grp, 'Carton', str(n), MP.box(x0, x0 + l, y0, y0 + w, z1 - h, z1), None, YELLOW)
            _carton_material(design, body)


def weigh(groups):
    """Each gripper group's mass (kg), centre's z (mm) and moment of inertia about the gripper's axis z (kg m2)."""
    out = {}
    for name in GROUPS[:5]:
        pp = groups[name].physicalProperties
        r = pp.getXYZMomentsOfInertia()                       # kg cm2 about the world's axes
        out[name] = (pp.mass, pp.centerOfMass.z * 10.0, r[3] / 1e4)
    return out


# ---------- the gripper ----------
def build(nx=2, ny=2, N=4700.0, state='gripping'):
    nx, ny, N = int(round(nx)), int(round(ny)), float(N)
    if state not in ('gripping', 'open'):
        raise ValueError('The state is gripping or open, not %r.' % state)
    s = refuse(nx, ny, N)
    j_grip = nx * CARTON['l'] * 1000.0 / 2.0
    j_open = j_grip + CG.CLEAR * 1000.0

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    groups = {name: MP.new_occ(root, name) for name in GROUPS}
    _last.clear()
    _last.update(groups=groups, j_grip=j_grip, j_open=j_open, j_built=j_grip, nx=nx, ny=ny, N=N, need=s)

    build_adapter(design, groups['Adapter'])
    build_frame(design, groups['Frame'])
    build_screw(design, groups['Screw'])
    build_jaw(design, groups['Jaw A'], 1, j_grip, 'A')
    build_jaw(design, groups['Jaw B'], -1, j_grip, 'B')
    build_pack(design, groups['Pack'], nx, ny)
    for name in ('Adapter', 'Frame', 'Screw'):
        groups[name].isGrounded = True
    j = set_state(groups, j_grip if state == 'gripping' else j_open)
    try:
        if design.snapshots.hasPendingSnapshot:
            design.snapshots.add()
    except Exception:
        pass
    app.activeViewport.fit()

    # ---------- the message ----------
    wt = weigh(groups)
    m_tot = sum(v[0] for v in wt.values())
    z_tot = sum(v[0] * v[1] for v in wt.values()) / m_tot
    i_tot = sum(v[2] for v in wt.values())
    acc = s['acc']
    gd = CG.guides(N, s['m'], wt['Jaw A'][0], acc)
    am = CG.arm(N)
    pl = parts_list(root)
    a_core = math.pi / 4.0 * SC['dk'] ** 2
    lines = [
        'Clamp gripper built for a pack of %d x %d cartons (%.0f x %.0f x %.0f mm, %.0f kg each): %.0f kg; the payload '
        'at the flange %.1f kg (changer %.1f, gripper %.1f, pack %.0f).'
        % (nx, ny, CARTON['l'] * 1e3, CARTON['w'] * 1e3, CARTON['h'] * 1e3, CARTON['m'], s['m'],
           CG.payload(nx, ny), CG.M_CHANGER, CG.M_GRIPPER, s['m']),
        'Squeeze %.0f N against the %.0f N this pack needs (pads %.0f N, carton on carton %.0f N; the %s governs: '
        '|a + g| %.2f m/s2 on the cycle, %.2f in a brake stop; factors %.2f and %.2f on mu %.2f): %.2f times the need.'
        % (N, s['need'], s['pad'], s['cc'], s['governs'], acc['cycle'], acc['stop'], CG.S_CYCLE, CG.S_STOP,
           CG.MU_PAD, N / s['need']),
        'The sides: %.0f N on %.3f m2 of pad, %.1f kPa against the %.0f kPa they accept (%.0f N at most on this pack).'
        % (N, s['area'], N / s['area'] / 1e3, CG.P_SIDE / 1e3, s['N_side']),
        'Screw: HIWIN peeled %.0f x %.0f, core %.1f mm, its axis at z = %.0f; right-hand groove from x = %.0f to %.0f, '
        'left-hand from %.0f to %.0f (balls %.3f mm on a %.2f mm circle, groove radius %.3f); in tension %.0f N '
        'between the nuts, %.1f MPa on the core, C0 / N = %.1f.'
        % (SC['d'], SC['lead'], SC['dk'], ZS, GROOVE_RH[0], GROOVE_RH[1], GROOVE_LH[0], GROOVE_LH[1], SC['Dw'], D_M,
           R_G, N, N / a_core, SC['C0'] / N),
        'Nuts: R32-10K5-DEB, right-hand on Jaw A and left-hand on Jaw B (C %.0f N, C0 %.0f N), each flange on the '
        'arm\'s outer face with six M8.' % (SC['C'], SC['C0']),
        'Bearings: BK25 fixed at x = %.0f (d %.0f, %.0f long, %.0f N axial at most), BF25 supported at x = %.0f '
        '(d %.0f, %.0f long), each on a seat under the beam; coupling, 3:1 gearhead and motor with its brake beyond '
        'the bracket at x = %.0f.' % (X_BK, BK['d'], BK['L'], BK['Fmax'], X_BF, BF['d'], BF['L'], X_BRACKET),
        'Guides: two HGR25 rails, %.0f mm long at y = +-%.0f; four %s blocks a jaw, %.0f mm apart along x; the most '
        'loaded block %.0f N in a brake stop, static factor %.2f (C0 %.0f N).'
        % (LAY['rail_len'], RAIL_Y, GD['name'], D_BLK, gd['P'], gd['fSL'], GD['C0']),
        'Arms: %.1f MPa at the ribs\' tips, the pad %.3f mm out under the squeeze.' % (am['stress'] / 1e6,
                                                                                         am['defl'] * 1e3),
        'Jaws: gripping at j = %.0f mm, open at %.0f mm (the pads %.0f mm off the pack), their range %.0f to %.0f mm; '
        'shown %s at j = %.0f mm.' % (j_grip, j_open, j_open - j_grip, J_MIN, J_MAX, state, j),
        'Weighed: ' + '; '.join('%s %.2f kg at z %.1f mm, %.3f kg m2' % (k, v[0], v[1], v[2]) for k, v in wt.items())
        + '. The gripper %.2f kg, its centre at z %.1f mm, %.3f kg m2 about its axis.' % (m_tot, z_tot, i_tot),
        'Parts (%d components in %d kinds): %s.' % (sum(pl.values()), len(pl),
                                                    ', '.join('%s %d' % (k, v) for k, v in sorted(pl.items()))),
        '[%s]' % VERSION,
    ]
    _last['weighed'] = wt
    return '\n\n'.join(lines)


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(nx=ins.itemById('nx').value, ny=ins.itemById('ny').value, N=ins.itemById('N').value,
                     state=ins.itemById('state').selectedItem.name)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addIntegerSpinnerCommandInput('nx', 'Cartons along the squeeze', 1, 9, 1, 2)
        ins.addIntegerSpinnerCommandInput('ny', 'Cartons across', 1, 9, 1, 2)
        ins.addValueInput('N', 'Squeeze, the jaws\' normal force (N)', '', VI(4700.0))
        dd = ins.addDropDownCommandInput('state', 'State', adsk.core.DropDownStyles.TextListDropDownStyle)
        dd.listItems.add('gripping', True)
        dd.listItems.add('open', False)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'ClampGripperPromptTestCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Clamp gripper',
                                                         'The palletizer\'s parallel-jaw gripper for cartons')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Clamp gripper could not start:\n' + traceback.format_exc())
