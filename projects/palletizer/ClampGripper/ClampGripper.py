# =============================================================================
# ClampGripper.py  -  Chapter 45 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer's
# parallel-jaw gripper for cartons. An adapter to ISO 9409-1-160-6-M10 under the tool changer's tool plate; an
# aluminium beam with two HIWIN HGR25 rails; two jaws on four HGH25CA blocks each, a ribbed arm and a rubber pad;
# one HIWIN 32 x 10 ballscrew, right-hand on the +x half and left-hand on the -x half, with an R32-10K5-DEB nut in
# each jaw, on a BK25 fixed and a BF25 supported bearing; a motor with gearhead and brake on a bracket at -x. The
# sizes are lib/clamp_gripper.py's. Written from PROMPT.txt beside it; checked by checks/clamp_gripper_check.py.
#
#   INPUTS        the dialog: the pack (cartons along the squeeze, across), the squeeze (N), gripping or open
#   DERIVED       clamp_gripper: CARTON, LAY (the layout), SCREW, BK25, BF25, GUIDE, design() (the squeeze each
#                 pack needs and the most the cartons accept)
#   HARD RULES    build() refuses a pack heavier than the design pack, a squeeze over the gripper's 5000 N, a
#                 squeeze under the pack's need, and one that presses the cartons' sides over 40 kPa
#   CONVENTIONS   z = 0 the gripper's mounting face, z up; x the squeeze, y across; groups Adapter and Frame
#                 (grounded), Screw (grounded), Jaw A (+x), Jaw B (-x), Pack; set_state() places the jaws
#   ACCEPTANCE    the message; the check: no interference gripping, open and over the jaws' range, the pads on
#                 the pack, the screw's two hands measured, the ISO pattern measured, the groups weighed
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import json
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                         # the project's folder: what this script writes goes in its results/
ROOT = PROJECT                                          # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import tool_changer           # noqa: E402
import clamp_gripper as cg    # noqa: E402
import mdfc_parts as mp       # noqa: E402
importlib.reload(tool_changer)                  # Chapter 44's module, which this one builds on
importlib.reload(cg)
importlib.reload(mp)


def _load(name, script):
    spec = importlib.util.spec_from_file_location(name, CP.script(script))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


BSA = _load('mdfc_ballscrewaxis', 'BallScrewAxis')

VERSION = 'ClampGripper 1.0 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

L = cg.LAY
SC = cg.SCREW
J_REF = 400.0                            # the jaws are built gripping the design pack: their pads at x = +-400
ZS = L['z_screw']
R_G = 0.52 * SC['Dw']                    # the groove's radius, 0.52 of the ball (Chapter 21)
D_M = SC['dk'] + 2 * R_G                 # the balls' circle, so that the groove's root is HIWIN's d_k
X_BK, X_BF = -880.0, 880.0               # the bearing units' centres
X_FRAME = (-1000.0, 900.0)               # the beam's ends: the motor's bracket at -x
THREAD = (215.0, 760.0)                  # each hand's threaded length, |x|
ISO = (30.0, 90.0, 150.0, 210.0, 270.0, 330.0)
M8 = ((55.0, 55.0), (-55.0, 55.0), (-55.0, -55.0), (55.0, -55.0))


def along_x(y, z, x0, x1, r):
    return mp.cyl((x0, y, z), (x1, y, z), r)


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def at(r, deg):
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


def group(root, name, grounded=False):
    o = mp.new_occ(root, name)
    o.isGroundToParent = False
    if grounded:
        o.isGrounded = True
    return o


def part(g, name, tb, material='Steel'):
    o = mp.new_occ(g.component, name)
    mp.add_body(o, tb, name, material)
    return o


def bearing_x(g, name, d, D, B, x0, z):
    """A ball bearing on the x axis at height z, its faces at x0 and x0 + B: the rings and the balls in one part."""
    gm = mp.bearing_geometry(d, D)
    b = mp.tube((x0, 0, z), (x0 + B, 0, z), gm['in_od'] / 2, d / 2)
    mp.union(b, mp.tube((x0, 0, z), (x0 + B, 0, z), D / 2, gm['out_id'] / 2))
    for k in range(gm['Z']):
        a = 2 * math.pi * k / gm['Z']
        mp.union(b, mp.sphere((x0 + B / 2, gm['dm'] / 2 * math.cos(a), z + gm['dm'] / 2 * math.sin(a)), gm['Dw'] / 2))
    return part(g, name, b)


def groove_x(occ, body, x0, x1, hand):
    """The ball groove cut into the screw from x0 to x1: Chapter 21's section swept along +x, one turn per lead,
    right-hand (hand +1) or left-hand (hand -1, the section and the twist mirrored)."""
    comp = occ.component
    P = SC['lead']
    section = BSA.groove_section(D_M / 2, R_G, P, 2 * math.pi * x0 / P)
    plane = comp.constructionPlanes.createInput()
    plane.setByOffset(comp.yZConstructionPlane, VI(x0 * CM))
    sk = comp.sketches.add(comp.constructionPlanes.add(plane))
    sk.isComputeDeferred = True
    pts = [sk.modelToSketchSpace(P3(x0 * CM, u * CM, (ZS + hand * v) * CM)) for u, v in section]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    pi_ = comp.constructionPlanes.createInput()
    pi_.setByOffset(comp.xYConstructionPlane, VI(ZS * CM))
    ps = comp.sketches.add(comp.constructionPlanes.add(pi_))
    ps.isVisible = False
    axis = ps.sketchCurves.sketchLines.addByTwoPoints(P3(x0 * CM, 0, 0), P3(x1 * CM, 0, 0))
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk.profiles.item(0), comp.features.createPath(axis, False),
                         adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.twistAngle = VI(hand * 2 * math.pi * (x1 - x0) / P)
    inp.participantBodies = [body]
    return sw.add(inp)


# ---------- the parts ----------
def adapter(root):
    g = group(root, 'Adapter', True)
    b = zcyl(L['adapter'][1], 0.0, 100.0)
    mp.union(b, zcyl(0.0, 8.0, 49.99))                                   # the spigot 100 g6 into the tool plate
    for a in ISO:
        x, y = at(80.0, a)
        mp.cut(b, zcyl(-17.0, 1.0, 5.5, x, y))                             # six M10 clearance holes on 160
        mp.cut(b, zcyl(-17.0, -5.5, 8.75, x, y))                           # their heads' counterbores
    mp.cut(b, zcyl(-8.0, 9.0, 5.0, 80.0, 0.0))                             # the pin on +Xm
    for x, y in M8:
        mp.cut(b, zcyl(-17.0, -1.0, 4.0, x, y))                            # the frame's four M8, tapped (drawn at 8)
    part(g, 'Adapter plate', b, 'Aluminum')
    for k, a in enumerate(ISO):
        x, y = at(80.0, a)
        part(g, 'Cap screw M10x25 %d' % (k + 1), mp.cap_screw(10.0, 25.0, (x, y, -5.5), (0, 0, 1)))
    part(g, 'Dowel pin 10x17', zcyl(-8.0, 9.0, 5.0, 80.0, 0.0))
    return g


def frame(root):
    g = group(root, 'Frame', True)
    zt, zb = L['frame']
    top, bot, side = L['walls']
    w = L['frame_w'] / 2
    b = mp.box(X_FRAME[0], X_FRAME[1], -w, w, zb, zt)
    mp.cut(b, mp.box(X_FRAME[0] - 1, X_FRAME[1] + 1, -w + side, w - side, zb + bot, zt - top))
    for x, y in M8:
        mp.cut(b, zcyl(zt - top - 1, zt + 1, 4.5, x, y))                   # the M8 through the top
        mp.cut(b, zcyl(zb - 1, zb + bot + 1, 10.0, x, y))                  # a key's way in through the bottom
    part(g, 'Beam', b, 'Aluminum')
    for k, (x, y) in enumerate(M8):
        part(g, 'Cap screw M8x25 %d' % (k + 1), mp.cap_screw(8.0, 25.0, (x, y, zt - top), (0, 0, 1)))
    ry, rl = L['rail_y'], L['rail_len'] / 2
    for k, s in enumerate((1, -1)):
        part(g, 'Rail HGR25 %d' % (k + 1), mp.box(-rl, rl, s * ry - 11.5, s * ry + 11.5, zb - 22.0, zb))
    zh = ZS + cg.BK25['h']                                                 # the units' bases, hanging
    for name, x, hw in (('Seat BK25', X_BK, 21.0), ('Seat BF25', X_BF, 15.0)):
        part(g, name, mp.box(x - hw, x + hw, -40.0, 40.0, zh, zb), 'Aluminum')
    hk = cg.BK25
    hb = mp.box(X_BK - 21.0, X_BK + 21.0, -53.0, 53.0, zh - hk['H'], zh)
    mp.cut(hb, along_x(0, ZS, X_BK - 22.0, X_BK + 22.0, 26.0))
    mp.union(hb, mp.tube((X_BK + 15.0, 0, ZS), (X_BK + 21.0, 0, ZS), 26.0, 15.0))   # the retaining shoulder
    part(g, 'Fixed bearing BK25', hb)
    for k in range(2):
        bearing_x(g, 'Angular contact bearing 7205 %d' % (k + 1), 25.0, 52.0, 15.0, X_BK - 15.0 + 15.0 * k, ZS)
    fb = mp.box(X_BF - 15.0, X_BF + 15.0, -53.0, 53.0, zh - cg.BF25['H'], zh)
    mp.cut(fb, along_x(0, ZS, X_BF - 16.0, X_BF + 16.0, 26.0))
    part(g, 'Supported bearing BF25', fb)
    bearing_x(g, 'Deep groove bearing 6205', 25.0, 52.0, 15.0, X_BF - 7.5, ZS)
    br = mp.box(X_FRAME[0] - 15.0, X_FRAME[0], -70.0, 70.0, -300.0, zt)
    mp.cut(br, along_x(0, ZS, X_FRAME[0] - 16.0, X_FRAME[0] + 1.0, 30.0))
    part(g, 'Motor bracket', br, 'Aluminum')
    part(g, 'Coupling', mp.tube((X_FRAME[0] + 10.0, 0, ZS), (X_BK - 30.0, 0, ZS), 28.0, 10.0), 'Aluminum')
    x0 = X_FRAME[0] - 15.0
    gh = along_x(0, ZS, x0 - 60.0, x0, 45.0)
    mp.union(gh, along_x(0, ZS, x0, X_FRAME[0] + 45.0, 10.0))           # its output shaft into the coupling
    part(g, 'Gearhead 3:1', gh, 'Aluminum')
    m = along_x(0, ZS, x0 - 220.0, x0 - 60.0, 50.0)
    mp.union(m, along_x(0, ZS, x0 - 250.0, x0 - 220.0, 45.0))             # the brake at the motor's back
    part(g, 'Motor with brake', m, 'Aluminum')
    return g


def screw(root):
    """One shaft, kept as two bodies that meet at x = 0, one per hand: with both long grooves in one body Fusion's
    kernel can no longer measure it (its volume, mass and point tests fail), while each half alone measures."""
    g = group(root, 'Screw', True)
    sl = along_x(0, ZS, X_FRAME[0] + 50.0, X_BK - 21.0, 10.0)                       # the drive journal into the coupling
    mp.union(sl, along_x(0, ZS, X_BK - 21.0, X_BK + 21.0, 12.5))                    # the fixed bearing's seat
    mp.union(sl, along_x(0, ZS, X_BK + 21.0, 0.0, SC['d'] / 2))
    sr = along_x(0, ZS, 0.0, X_BF - 15.0, SC['d'] / 2)
    mp.union(sr, along_x(0, ZS, X_BF - 15.0, X_BF + 15.0, 12.5))                    # the supported bearing's seat
    o = mp.new_occ(g.component, 'Ballscrew 32x10 right and left')
    mp.add_body(o, sr, 'Right-hand half', 'Steel')
    mp.add_body(o, sl, 'Left-hand half', 'Steel')
    for nm, x0, x1, hand in (('Right-hand half', THREAD[0], THREAD[1], 1), ('Left-hand half', -THREAD[1], -THREAD[0], -1)):
        body = [b for b in o.component.bRepBodies if b.name == nm][0]
        groove_x(o, body, x0, x1, hand)
    return g


def jaw(root, name, side):
    """A jaw built gripping the design pack, its pad's face at x = side x J_REF; side +1 the +x jaw (right-hand
    nut), -1 the -x jaw (left-hand)."""
    g = group(root, name)
    s = side
    X = lambda a, b: (min(s * (J_REF + a), s * (J_REF + b)), max(s * (J_REF + a), s * (J_REF + b)))
    c0, c1 = L['carriage']
    cx = X(L['pad_t'], L['pad_t'] + L['carriage_len'])
    part(g, 'Carriage plate', mp.box(cx[0], cx[1], -L['carriage_w'] / 2, L['carriage_w'] / 2, c1, c0), 'Aluminum')
    zb = L['frame'][1]
    for xc in (L['pad_t'] + 42.0, L['pad_t'] + 42.0 + L['d_blocks']):
        for ry in (L['rail_y'], -L['rail_y']):
            bx = X(xc - 42.0, xc + 42.0)
            blk = mp.box(bx[0], bx[1], ry - 24.0, ry + 24.0, c0, zb - 5.5)
            mp.cut(blk, mp.box(bx[0] - 1, bx[1] + 1, ry - 11.6, ry + 11.6, zb - 22.1, zb))
            part(g, 'Block HGH25CA %s%s' % ('i' if xc < 100 else 'o', '+' if ry > 0 else '-'), blk)
    ax = X(L['pad_t'], L['pad_t'] + L['arm_t'])
    a = mp.box(ax[0], ax[1], -L['jaw_w'] / 2, L['jaw_w'] / 2, L['arm_bottom'], c1)
    rx = X(L['pad_t'] + L['arm_t'], L['pad_t'] + L['arm_t'] + L['rib'][1])
    for ry in (L['rib_y'], -L['rib_y']):
        mp.union(a, mp.box(rx[0], rx[1], ry - L['rib'][0] / 2, ry + L['rib'][0] / 2, L['arm_bottom'], c1))
    mp.cut(a, along_x(0, ZS, ax[0] - 1, ax[1] + 1, 17.0))
    for k in range(6):                                                     # the nut's six M8, tapped (drawn at 8)
        y, z = at(32.5, 30.0 + 60.0 * k)
        mp.cut(a, along_x(y, ZS + z, ax[0] - 1, ax[1] + 1, 4.0))
    part(g, 'Arm', a, 'Aluminum')
    px = X(0.0, L['pad_t'])
    part(g, 'Pad', mp.box(px[0], px[1], -L['jaw_w'] / 2, L['jaw_w'] / 2, L['pad'][1], L['pad'][0]), 'Rubber')
    f0 = L['pad_t'] + L['arm_t']
    fx = X(f0, f0 + SC['L1'])
    nb = along_x(0, ZS, fx[0], fx[1], SC['D1'] / 2)
    bx = X(f0 + SC['L1'], f0 + SC['L'])
    mp.union(nb, along_x(0, ZS, bx[0], bx[1], SC['D'] / 2))
    mp.cut(nb, along_x(0, ZS, min(fx[0], bx[0]) - 1, max(fx[1], bx[1]) + 1, SC['d'] / 2 + 0.4))
    for k in range(6):
        y, z = at(32.5, 30.0 + 60.0 * k)
        mp.cut(nb, along_x(y, ZS + z, fx[0] - 1, fx[1] + 1, 4.5))
    part(g, 'Ball nut R32-10K5-DEB %s' % ('right' if s > 0 else 'left'), nb)
    for k in range(6):
        y, z = at(32.5, 30.0 + 60.0 * k)
        part(g, 'Cap screw M8x25 nut %d' % (k + 1),
             mp.cap_screw(8.0, 25.0, (s * (J_REF + f0 + SC['L1']), y, ZS + z), (-s, 0, 0)))
    return g


def pack(root, nx, ny):
    g = group(root, 'Pack')
    c = cg.CARTON
    lx, ly, h = c['l'] * 1000, c['w'] * 1000, c['h'] * 1000
    for i in range(nx):
        for k in range(ny):
            x0 = -nx * lx / 2 + i * lx
            y0 = -ny * ly / 2 + k * ly
            part(g, 'Carton %d' % (i * ny + k + 1),
                 mp.box(x0, x0 + lx, y0, y0 + ly, L['pack_top'] - h, L['pack_top']), 'Paper')
    return g


# ---------- the states ----------
def place(occ, dx=0.0, dy=0.0, dz=0.0):
    m = adsk.core.Matrix3D.create()
    m.translation = V3(dx * CM, dy * CM, dz * CM)
    occ.transform2 = m


def set_state(G_, j):
    """Both jaws' pads at x = +-j (mm): the screw's two hands move them together."""
    place(G_['Jaw A'], dx=j - J_REF)
    place(G_['Jaw B'], dx=-(j - J_REF))
    adsk.doEvents()


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0 or o.fullPathName.startswith('Pack'):
            continue
        nm = o.component.name
        if nm.endswith(')') and ' (' in nm:
            nm = nm[:nm.rindex(' (')]
        wds = nm.split()
        if wds[-1].isdigit() or wds[-1] in ('i+', 'i-', 'o+', 'o-'):
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


def refuse(nx, ny, N):
    m = cg.pack_mass(nx, ny)
    m_max = cg.pack_mass(cg.PACK['nx'], cg.PACK['ny'])
    if m > m_max or nx > 3 or ny > 2:
        raise ValueError('A pack of %d x %d cartons, %.0f kg: the gripper takes up to three along and two across, '
                         'and up to %.0f kg.' % (nx, ny, m, m_max))
    if N > cg.N_MAX:
        raise ValueError('A squeeze of %.0f N is over the %.0f N the screw, the guides and the arms are built for.'
                         % (N, cg.N_MAX))
    s = cg.squeeze_needed(nx, ny)                     # the stops' accelerations for this pack's payload
    if N < s['need']:
        raise ValueError('A squeeze of %.0f N is under the %.0f N this pack needs to stay in the jaws.' % (N, s['need']))
    if N > s['N_side']:
        raise ValueError('A squeeze of %.0f N presses the cartons\' sides at %.1f kPa, over the %.0f kPa they accept.'
                         % (N, N / s['area'] / 1e3, cg.P_SIDE / 1e3))
    return s


_last = {}


def build(nx=2, ny=2, N=cg.N_SET, gripping=True):
    nx, ny = int(round(nx)), int(round(ny))
    s = refuse(nx, ny, N)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    G_ = {'Adapter': adapter(root), 'Frame': frame(root), 'Screw': screw(root),
          'Jaw A': jaw(root, 'Jaw A', 1), 'Jaw B': jaw(root, 'Jaw B', -1), 'Pack': pack(root, nx, ny)}
    for k, col in (('Adapter', 'Paint - Enamel Glossy (Blue)'), ('Frame', 'Paint - Enamel Glossy (Grey)'),
                   ('Jaw A', 'Paint - Enamel Glossy (Yellow)'), ('Jaw B', 'Paint - Enamel Glossy (Yellow)'),
                   ('Screw', 'Paint - Enamel Glossy (Red)')):
        mp.colour(des, G_[k], col)
    j_grip = nx * cg.CARTON['l'] * 1000 / 2
    set_state(G_, j_grip if gripping else j_grip + cg.CLEAR * 1000)
    _last.update(groups=G_, nx=nx, ny=ny, N=N, j_grip=j_grip)
    app.activeViewport.fit()
    n = parts_list(root)
    return ('Clamp gripper built, %s, on a pack of %d x %d cartons (%.0f kg).\nSqueeze %.0f N (the pack needs %.0f N; '
            'the sides take up to %.0f N); the pads at x = +-%.0f mm.\nBallscrew 32 x 10 right and left, R32-10K5-DEB '
            'nuts, BK25 and BF25; rails HGR25, HGH25CA blocks.\n\nParts: %d components; %s.\n\n[%s]'
            % ('gripping' if gripping else 'open', nx, ny, s['m'], N, s['need'], s['N_side'],
               j_grip if gripping else j_grip + cg.CLEAR * 1000, sum(n.values()),
               ', '.join('%d %s' % (v, k) for k, v in sorted(n.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('nx', 'Cartons along the squeeze', 2.0), ('ny', 'Cartons across', 2.0), ('N', 'The squeeze (N)', cg.N_SET)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = {k: ins.itemById(k).value for k, _, _ in FIELDS}
            vals['gripping'] = ins.itemById('gripping').value
            ui.messageBox(build(**vals), VERSION)
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
        ins.addBoolValueInput('gripping', 'Gripping', True, '', True)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'ClampGripperCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Clamp gripper', 'The palletizer\'s carton gripper of Chapter 45')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ClampGripper could not start:\n' + traceback.format_exc())
