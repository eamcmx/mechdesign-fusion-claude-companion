# =============================================================================
# BagGripper.py  -  Chapter 47 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer's gripper for
# bags. The adapter to ISO 9409-1-160-6-M10 of Chapter 45 on a box beam; end plates carrying two jaws, each a steel
# box tube on stub shafts in 6206 bearings with eight tines that close under the bag; each jaw turned by a Festo
# DSBC-80-125 with a clamping unit on a lever at one end; a press plate on two DSBC-32-50; the bag and the pick-up
# station's rollers. The sizes are lib/bag_gripper.py's. Written from PROMPT.txt beside it; checked by
# checks/bag_gripper_check.py.
#
#   INPUTS        the dialog: the bag's mass (kg), its width (mm), the supply pressure (bar)
#   DERIVED       bag_gripper: LAY, DSBC, BRG, CYL, the jaw's moment in every stop for the payload
#   HARD RULES    build() refuses a bag the cylinders or their clamping units do not hold in the worst stop, a bag
#                 the jaws do not fit, and a pressure under the 6 bar the cylinders are rated at
#   CONVENTIONS   z = 0 the gripper's mounting face, z up; x along the bag's 800; groups Adapter and Frame
#                 (grounded), Jaw A and B, Cylinder A and B with their rods, Press, Bag, Station; set_state(phi, e)
#                 turns the jaws open by phi and sets the press plate's extension e
#   ACCEPTANCE    the message; the check: closed on the bag, the sweep clear of the bag and the rollers, the
#                 cylinders' lengths, the ISO pattern, the groups weighed and the jaw's inertia written
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
import tool_changer           # noqa: E402
import clamp_gripper          # noqa: E402
import vacuum_gripper         # noqa: E402
import bag_gripper as bg      # noqa: E402
import mdfc_parts as mp       # noqa: E402
importlib.reload(tool_changer)                  # the modules this one builds on, as Fusion keeps old ones
importlib.reload(clamp_gripper)
importlib.reload(vacuum_gripper)
importlib.reload(bg)
importlib.reload(mp)

VERSION = 'BagGripper 1.0 (2026-11-03)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

L = bg.LAY
ISO = (30.0, 90.0, 150.0, 210.0, 270.0, 330.0)
M8 = ((55.0, 55.0), (-55.0, 55.0), (-55.0, -55.0), (55.0, -55.0))
YH, ZH = L['hinge']
CG_ = bg.cylinder_geometry()
YP, ZP = CG_['pin']
CYL = bg.DSBC[bg.CYL['bore']]
PR = bg.DSBC[L['press']['bore']]


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def xcyl(x0, x1, r, y, z):
    return mp.cyl((x0, y, z), (x1, y, z), r)


def xtube(x0, x1, ro, ri, y, z):
    return mp.tube((x0, y, z), (x1, y, z), ro, ri)


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


def turned(tb):
    """A copy turned half a turn about z: the -y jaw's parts from the +y jaw's, the -x end's from the +x end's."""
    b = mp.tbm().copy(tb)
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.pi, V3(0, 0, 1), P3(0, 0, 0))
    mp.tbm().transform(b, m)
    return b


# ---------- the parts ----------
def adapter(root):
    g = group(root, 'Adapter', True)
    b = zcyl(L['adapter'][1], 0.0, 100.0)
    mp.union(b, zcyl(0.0, 8.0, 49.99))
    for a in ISO:
        x, y = at(80.0, a)
        mp.cut(b, zcyl(-17.0, 1.0, 5.5, x, y))
        mp.cut(b, zcyl(-17.0, -5.5, 8.75, x, y))
    mp.cut(b, zcyl(-8.0, 9.0, 5.0, 80.0, 0.0))
    for x, y in M8:
        mp.cut(b, zcyl(-17.0, -1.0, 4.0, x, y))
    part(g, 'Adapter plate', b, 'Aluminum')
    for k, a in enumerate(ISO):
        x, y = at(80.0, a)
        part(g, 'Cap screw M10x25 %d' % (k + 1), mp.cap_screw(10.0, 25.0, (x, y, -5.5), (0, 0, 1)))
    part(g, 'Dowel pin 10x17', zcyl(-8.0, 9.0, 5.0, 80.0, 0.0))
    return g


def end_plate():
    """The +x end plate with its bearing bosses and its bag wall; the -x one is the same turned."""
    x0, x1 = L['plate_x']
    z0, z1 = L['plate_z']
    b = mp.box(x0, x1, -L['plate_y'], L['plate_y'], z1, z0)
    mp.union(b, mp.box(x0, x1, -L['wall_y'], L['wall_y'], L['wall_z'], z1))
    bo0, bo1 = L['boss']
    for s in (1, -1):
        mp.union(b, xcyl(x1 - 0.01, bo1, L['boss_r'], s * YH, ZH))
        mp.cut(b, xcyl(x0 - 1, bo1 + 1, bg.BRG[30][0] / 2, s * YH, ZH))
    for y, z in ((50.0, -40.0), (-50.0, -40.0), (50.0, -92.0), (-50.0, -92.0)):
        mp.cut(b, xcyl(x0 - 1, x1 + 1, 4.5, y, z))
    return b


def frame(root):
    g = group(root, 'Frame', True)
    zt, zb = L['beam']
    t = L['beam_t']
    hx, hy = L['beam_x'], L['beam_y']
    b = mp.box(-hx, hx, -hy, hy, zb, zt)
    mp.cut(b, mp.box(-hx - 1, hx + 1, -hy + t, hy - t, zb + t, zt - t))
    for x, y in M8:
        mp.cut(b, zcyl(zt - t - 1, zt + 1, 4.5, x, y))
        mp.cut(b, zcyl(zb - 1, zb + t + 1, 10.0, x, y))
    tg = 32.5 / 2
    for px in L['press']['x']:
        for dx in (-tg, tg):
            for dy in (-tg, tg):
                mp.cut(b, zcyl(zb - 1, zb + t + 1, 3.3, px + dx, dy))       # the press cylinders' M6
                mp.cut(b, zcyl(zt - t - 1, zt + 1, 6.0, px + dx, dy))       # the way in from above
    part(g, 'Beam', b, 'Aluminum')
    for k, x in enumerate((1, -1)):
        blk = mp.box(hx - L['block'], hx, -hy + t, hy - t, zb + t, zt - t)
        for y, z in ((50.0, -40.0), (-50.0, -40.0), (50.0, -92.0), (-50.0, -92.0)):
            mp.cut(blk, xcyl(hx - 16, hx + 1, 4.0, y, z))
        part(g, 'End block %d' % (k + 1), blk if x > 0 else turned(blk), 'Aluminum')
    for x, y in M8:
        part(g, 'Cap screw M8x20 %d' % (M8.index((x, y)) + 1), mp.cap_screw(8.0, 20.0, (x, y, zt - t), (0, 0, 1)))
    ep = end_plate()
    part(g, 'End plate 1', ep, 'Aluminum')
    part(g, 'End plate 2', turned(ep), 'Aluminum')
    k = 0
    for s in (1, -1):
        for y, z in ((50.0, -40.0), (-50.0, -40.0), (50.0, -92.0), (-50.0, -92.0)):
            k += 1
            sc = mp.cap_screw(8.0, 30.0, (L['plate_x'][1], y, z), (-1, 0, 0))
            part(g, 'Cap screw M8x30 %d' % k, sc if s > 0 else turned(sc))
    # the four bearings, 6206, in the bosses
    od, w = bg.BRG[30][0], bg.BRG[30][1]
    b0, b1 = L['bearing_x']
    k = 0
    for s in (1, -1):
        for sy in (1, -1):
            k += 1
            ring = xtube(b0, b1, od / 2, L['stub_d'] / 2, sy * YH, ZH)
            part(g, 'Bearing 6206 %d' % k, ring if s > 0 else turned(ring))
    # the cylinders' rear brackets: an arm from the end plate's face to a lug beside the swivel flange's eye
    br = mp.box(L['plate_x'][1], L['cyl_x'] + 30.0, YP - 60.0, YP - 25.0, ZP - 30.0, ZP + 30.0)
    mp.union(br, mp.box(L['cyl_x'] + 10.0, L['cyl_x'] + 30.0, YP - 60.0, YP + 12.0, ZP - 20.0, ZP + 20.0))
    mp.union(br, xcyl(L['cyl_x'] - 10.0, L['cyl_x'] + 10.01, CYL['EK'] / 2, YP, ZP))
    part(g, 'Bracket 1', br, 'Steel')
    part(g, 'Bracket 2', turned(br), 'Steel')
    # the press cylinders, DSBC-32-50, under the beam on four M6 each
    E = PR['E']
    body_h = 94.0 + L['press']['stroke']
    for k, px in enumerate(L['press']['x']):
        cb = mp.box(px - E / 2, px + E / 2, -E / 2, E / 2, zb - body_h, zb)
        mp.cut(cb, zcyl(zb - body_h - 1, zb - body_h + L['press']['stroke'] + 10, PR['rod'] / 2, px, 0.0))
        for dx in (-tg, tg):
            for dy in (-tg, tg):
                mp.cut(cb, zcyl(zb - 12, zb + 1, 3.0, px + dx, dy))
        part(g, 'Cylinder DSBC-32-50 %d' % (k + 1), cb, 'Aluminum')
        for j, (dx, dy) in enumerate(((-tg, -tg), (tg, -tg), (tg, tg), (-tg, tg))):
            part(g, 'Cap screw M6x16 %d' % (4 * k + j + 1), mp.cap_screw(6.0, 16.0, (px + dx, dy, zb + t), (0, 0, -1)))
    return g


def jaw_bodies():
    """The +y jaw, closed: the box tube, the two end caps with their stub shafts (the driven one long to the lever),
    eight tines, the lever with its pin."""
    yh, zh = YH, ZH
    hb = bg.TUBE['b'] / 2
    tt = bg.TUBE['t']
    tx = L['tube_x']
    out = []
    tb = mp.box(-tx, tx, yh - hb, yh + hb, zh - hb, zh + hb)
    mp.cut(tb, mp.box(-tx - 1, tx + 1, yh - hb + tt, yh + hb - tt, zh - hb + tt, zh + hb - tt))
    out.append(('Tube', tb, 'Steel'))
    for k, (xa, xb, xs) in enumerate(((tx - 8.0, tx, L['lever_x'][1]), (-tx, -tx + 8.0, -L['bearing_x'][1] - 2.0))):
        cap = mp.box(xa, xb, yh - hb + tt, yh + hb - tt, zh - hb + tt, zh + hb - tt)
        mp.union(cap, xcyl(xa if xs > 0 else xb, xs, L['stub_d'] / 2, yh, zh))
        out.append(('Stub shaft %s' % ('long' if k == 0 else 'short'), cap, 'Steel'))
    t2 = L['tine_t'] / 2
    for k, x in enumerate(L['tine_x']):
        tn = mp.box(x - t2, x + t2, L['leg_y'][0], L['leg_y'][1], L['foot_z'][1], L['leg_top'])
        mp.union(tn, mp.box(x - t2, x + t2, L['tip_y'], L['leg_y'][1], L['foot_z'][0], L['foot_z'][1]))
        out.append(('Tine %d' % (k + 1), tn, 'Steel'))
    x0, x1 = L['lever_x']
    r = L['lever']
    lv = xcyl(x0, x1, 25.0, yh, zh)
    mp.union(lv, mp.box(x0, x1, yh - 20.0, yh + 20.0, zh, zh + r))
    mp.union(lv, xcyl(x0, x1, 20.0, yh, zh + r))
    mp.cut(lv, xcyl(x0 - 1, x1 + 1, L['stub_d'] / 2, yh, zh))
    mp.union(lv, xcyl(x1 - 0.01, x1 + 20.0, CYL['EK'] / 2, yh, zh + r))
    out.append(('Lever', lv, 'Steel'))
    return out


def cylinder_bodies():
    """Cylinder A, closed (fully out), along +y from its rear pin: the swivel flange's eye and block, the body with
    the clamping unit and its bore for the rod; and the moving part: the rod and its end with the eye."""
    xc = L['cyl_x']
    E = CYL['E']
    fl = CYL['FL']                                                   # SNCB: FL, pin to the rear face (p. 25)
    s_body = YP + fl
    body_len = CYL['XC'] - fl + (CYL['ZW'] - CYL['ZJ']) - CYL['WH']  # to the clamping unit's front face
    s_front = s_body + body_len + bg.CYL['stroke']
    body = xcyl(xc - 10.0, xc + 10.0, 20.0, YP, ZP)
    mp.cut(body, xcyl(xc - 11.0, xc + 11.0, CYL['EK'] / 2, YP, ZP))
    mp.union(body, mp.box(xc - 25.0, xc + 25.0, YP + 20.0, s_body, ZP - 30.0, ZP + 30.0))
    mp.union(body, mp.box(xc - E / 2, xc + E / 2, s_body, s_front, ZP - E / 2, ZP + E / 2))
    mp.cut(body, mp.cyl((xc, s_body + 10.0, ZP), (xc, s_front - (CYL['ZW'] - CYL['ZJ']) - 10.0, ZP), bg.CYL['bore'] / 2))   # the barrel's bore
    mp.cut(body, mp.cyl((xc, s_front - bg.CYL['stroke'] - 40.0, ZP), (xc, s_front + 1.0, ZP), CYL['rod'] / 2))
    y_end = YP + CG_['L_ext'] - L['rod_end']                       # the rod's end face, closed
    rod = mp.cyl((xc, s_front - 20.0, ZP), (xc, y_end, ZP), CYL['rod'] / 2)
    re = mp.box(xc - 10.0, xc + 10.0, y_end, YH, ZP - CYL['rod'] / 2, ZP + CYL['rod'] / 2)
    mp.union(re, xcyl(xc - 10.0, xc + 10.0, 20.0, YH, ZP))
    mp.cut(re, xcyl(xc - 11.0, xc + 11.0, CYL['EK'] / 2, YH, ZP))
    return body, rod, re, dict(s_body=s_body, s_front=s_front, y_end=y_end)


def jaws_and_cylinders(root):
    G_ = {}
    for name, flip in (('A', False), ('B', True)):
        g = group(root, 'Jaw %s' % name)
        for nm, tb, mat in jaw_bodies():
            part(g, '%s %s' % (nm, name), turned(tb) if flip else tb, mat)
        G_['Jaw %s' % name] = g
        body, rod, re, _ = cylinder_bodies()
        gc = group(root, 'Cylinder %s' % name)
        part(gc, 'Cylinder DSBC-%d-%d-C %s' % (bg.CYL['bore'], bg.CYL['stroke'], name), turned(body) if flip else body, 'Aluminum')
        G_['Cylinder %s' % name] = gc
        gr = group(root, 'Rod %s' % name)
        part(gr, 'Piston rod %s' % name, turned(rod) if flip else rod)
        part(gr, 'Rod end %s' % name, turned(re) if flip else re)
        G_['Rod %s' % name] = gr
    return G_


def press(root):
    g = group(root, 'Press')
    px, py, pt = L['press']['plate']
    top = L['bag_top'] + pt
    part(g, 'Press plate', mp.box(-px / 2, px / 2, -py / 2, py / 2, L['bag_top'], top), 'Aluminum')
    zb = L['beam'][1]
    front = zb - 94.0 - L['press']['stroke']
    for k, x in enumerate(L['press']['x']):
        part(g, 'Press rod %d' % (k + 1), zcyl(top, front + L['press']['stroke'] - L['press']['ext'] + 0.0, PR['rod'] / 2, x, 0.0))
    return g


def bag(root, w):
    g = group(root, 'Bag')
    b = bg.BAG
    part(g, 'Bag', mp.box(-b['l'] / 2, b['l'] / 2, -w / 2, w / 2, L['bag_bottom'], L['bag_top']), 'Paper')
    return g


def station(root):
    g = group(root, 'Station')
    r = bg.PICK['roller_d'] / 2
    for k, x in enumerate(range(-400, 401, int(bg.PICK['pitch']))):
        part(g, 'Roller %d' % (k + 1), mp.cyl((float(x), -420.0, L['bag_bottom'] - r), (float(x), 420.0, L['bag_bottom'] - r), r))
    return g


# ---------- the states ----------
def _rot_x(ang, y0, z0):
    m = adsk.core.Matrix3D.create()
    m.setToRotation(ang, V3(1, 0, 0), P3(0, y0 * CM, z0 * CM))
    return m


def set_state(G_, phi=0.0, e=None):
    """The jaws open by phi (deg): A turns about its hinge so that its tines move out; B, its turned copy, the other
    way; each cylinder turns about its rear pin to follow the lever, its rod slides in by what the lever's travel
    shortens it. The press plate at extension e (mm of its 50), at the bag when e = 24."""
    e = L['press']['ext'] if e is None else e
    ph = math.radians(phi)
    r = L['lever']
    tip = (YH - r * math.sin(ph), ZH + r * math.cos(ph))
    psi = math.atan2(tip[1] - ZP, tip[0] - YP)
    Lc = math.hypot(tip[0] - YP, tip[1] - ZP)
    d = CG_['L_ext'] - Lc
    for name, s in (('A', 1.0), ('B', -1.0)):
        G_['Jaw %s' % name].transform2 = _rot_x(s * ph, s * YH, ZH)
        G_['Cylinder %s' % name].transform2 = _rot_x(s * psi, s * YP, ZP)
        m = adsk.core.Matrix3D.create()
        m.translation = V3(0, -s * d * CM, 0)
        m.transformBy(_rot_x(s * psi, s * YP, ZP))
        G_['Rod %s' % name].transform2 = m
    m = adsk.core.Matrix3D.create()
    m.translation = V3(0, 0, (L['press']['ext'] - e) * CM)
    G_['Press'].transform2 = m
    adsk.doEvents()
    return dict(L=Lc, psi=math.degrees(psi))


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0 or o.fullPathName.startswith('Bag') or o.fullPathName.startswith('Station'):
            continue
        nm = o.component.name
        if nm.endswith(')') and ' (' in nm:
            nm = nm[:nm.rindex(' (')]
        wds = nm.split()
        if len(wds) > 1 and wds[-1] in ('A', 'B'):
            wds = wds[:-1]                                                   # the jaw
        if len(wds) > 1 and wds[-1].isdigit():
            wds = wds[:-1]                                                   # the part's own number
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


def refuse(m, w, p_bar):
    if p_bar < 6.0 - 1e-9:
        raise ValueError('A supply of %.1f bar: the cylinders are worked at 6 bar; less closes the jaws with less.' % p_bar)
    if w > 2 * (L['leg_y'][0] - 10.0) or w < 2 * (L['tip_y'] + 100.0):
        raise ValueError('A bag %.0f mm wide: the jaws take bags from %.0f to %.0f mm across.'
                         % (w, 2 * (L['tip_y'] + 100.0), 2 * (L['leg_y'][0] - 10.0)))
    b = dict(bg.BAG, m=m)
    d = bg.design()
    pl = clamp_gripper.M_CHANGER + d['m_gripper'] + m
    acc = bg.accelerations(pl)
    M = bg.jaw_moment(acc['stop']['a_v'], acc['stop']['a_h'], bag=b)['M']
    r = L['lever'] / 1000.0
    S_ = CYL['F_adv'] * r / M
    S_h = CYL['F_hold'] * r / M
    if S_ < bg.S_STOP or S_h < bg.S_STOP:
        raise ValueError('A bag of %.0f kg: the worst stop opens a jaw with %.1f N m; the cylinder holds it %.2f times '
                         'and its clamping unit %.2f times, under %.2f.' % (m, M, S_, S_h, bg.S_STOP))
    return dict(M=M, S=S_, S_hold=S_h)


_last = {}


def build(m=bg.BAG['m'], w=bg.BAG['w'], p_bar=6.0):
    h = refuse(m, w, p_bar)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    G_ = {'Adapter': adapter(root), 'Frame': frame(root)}
    G_.update(jaws_and_cylinders(root))
    G_['Press'] = press(root)
    G_['Bag'] = bag(root, w)
    G_['Station'] = station(root)
    for k, col in (('Adapter', 'Paint - Enamel Glossy (Blue)'), ('Frame', 'Paint - Enamel Glossy (Grey)'),
                   ('Jaw A', 'Paint - Enamel Glossy (Yellow)'), ('Jaw B', 'Paint - Enamel Glossy (Yellow)'),
                   ('Press', 'Paint - Enamel Glossy (Grey)')):
        mp.colour(des, G_[k], col)
    set_state(G_, 0.0)
    _last.update(groups=G_, m=m, w=w)
    app.activeViewport.fit()
    pl = parts_list(root)
    return ('Bag gripper built on a bag of %.0f kg, %.0f mm wide.\nTwo jaws of eight tines, each turned by a DSBC-%d-%d '
            'with a clamping unit on a %.0f mm lever; the worst stop opens a jaw with %.1f N m, held %.2f times by the '
            'cylinder and %.2f times by its clamping unit.\n\nParts: %d components; %s.\n\n[%s]'
            % (m, w, bg.CYL['bore'], bg.CYL['stroke'], L['lever'], h['M'], h['S'], h['S_hold'], sum(pl.values()),
               ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('m', 'The bag\'s mass (kg)', bg.BAG['m']), ('w', 'The bag\'s width (mm)', bg.BAG['w']),
          ('p_bar', 'The supply pressure (bar)', 6.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = {k: ins.itemById(k).value for k, _, _ in FIELDS}
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
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'BagGripperCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Bag gripper', 'The palletizer\'s bag gripper of Chapter 47')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('BagGripper could not start:\n' + traceback.format_exc())
