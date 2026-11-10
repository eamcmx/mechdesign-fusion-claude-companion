# =============================================================================
# PalletizerDetail.py  -  Chapter 55 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer of
# Chapters 41 to 48 in detail, one assembly at a time, each built by its own script, loaded, not copied:
#   robot   PalletizerRobot.py with the rods' six eyes holding their bearings' outer rings between a shoulder and a
#           DIN 472 circlip, and the pins holding the inner rings between a collar and an end washer;
#   bag     StandardParts.py (the bag gripper as Chapter 50 detailed it) with each jaw's driven end locating and its
#           short end floating, the end plates' bosses lengthened for DIN 472 circlips, the short stubs' shoulders
#           and DIN 471 circlips, and the plugs set 4 mm into the jaw's tube and welded all round;
#   unit    ComplianceUnit.py with the O-ring's groove to ERIKS's dynamic table and DIN 908 plugs in the galleries;
#   vacuum  VacuumGripper.py with the plate held to the beams by through bolts and nuts.
# Then every made part's material and every fitting face's class as Chapter 51's MaterialsFits.py gives them, and by
# hand what this chapter decides against the rules: the rods' eyes H7, the floating ends H7, the jaws in 1020, the
# unit's tube in two zones, the bearings' clearance classes and the HIWIN codes in the descriptions.
# Written from PROMPT.txt beside it; checked by checks/palletizer_detail_check.py.
#
#   INPUTS        the dialog: the assembly (bag by default), the plugs' fillet weld leg (mm)
#   DERIVED       palletizer_detail: eyes(), jaw_ends(), plug_weld(), gland(), gallery_plug(), vacuum_joint(),
#                 robot_bearings(), bag_6206(), HIWIN
#   HARD RULES    build() refuses a weld leg under Shigley's Table 9-6 minimum for the 4 mm wall or over the wall,
#                 and anything the assembly's own script refuses; nothing of the four scripts', StandardParts' or
#                 MaterialsFits' code copied
#   CONVENTIONS   as each assembly's script; the changed parts keep their names
#   ACCEPTANCE    the message; the check: each assembly's new geometry, the classes and designations, no interference
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
SCRIPTS = os.path.dirname(HERE)
LIB = os.path.join(os.path.dirname(SCRIPTS), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_pd55_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


MF = _load('MaterialsFits')
import mdfc_parts as mp                  # noqa: E402
import palletizer_detail as XP           # noqa: E402
importlib.reload(XP)

VERSION = 'PalletizerDetail 1.0 (2026-11-03)'
CM = 0.1
VI = adsk.core.ValueInput.createByReal
KEYS = ('bag', 'robot', 'unit', 'vacuum')
STEEL_1020 = 'Steel AISI 1020 107 HR'
_last = {}


def near(a, b, tol=1e-6):
    return abs(a - b) < tol


def centre_mm(tb):
    bb = tb.boundingBox
    return [(getattr(bb.minPoint, a) + getattr(bb.maxPoint, a)) * 5.0 for a in 'xyz'], \
           [(getattr(bb.maxPoint, a) - getattr(bb.minPoint, a)) * 10.0 for a in 'xyz']


def refuse(weld):
    w = XP.plug_weld()
    if weld < w['h_min'] - 1e-9:
        raise ValueError('The plugs\' fillet weld of %.1f mm is under Shigley\'s Table 9-6 minimum of %.1f mm for the '
                         'tube\'s %.0f mm wall.' % (weld, w['h_min'], w['h_max']))
    if weld > w['h_max'] + 1e-9:
        raise ValueError('The plugs\' fillet weld of %.1f mm is over the tube\'s %.0f mm wall, which Table 9-6 says it '
                         'may not exceed.' % (weld, w['h_max']))


# ---------- the robot: the rods' eyes and the pins ----------
def build_robot():
    RB = _load('PalletizerRobot')
    EY = {e['name']: e for e in XP.eyes()}
    PINS = {e['pin']: e for e in XP.eyes()}
    plain_part = RB.part

    def rod_eye(g, name, c, y0, y1, br, ro):
        e = EY[name]
        b = XP.SRB[e['bearing']]
        s = e['side']
        face_free = e['eye'][1] if s > 0 else e['eye'][0]
        body = RB.ytube(c[0], c[1], y0, y1, ro, e['lip_d'] / 2.0)
        mp.cut(body, RB.ycyl(c[0], c[1], *sorted((e['ring'][0], face_free + s * 1.0)), b['D'] / 2.0))
        mp.cut(body, RB.ycyl(c[0], c[1], *sorted(e['groove']), e['groove_d'] / 2.0))
        RB.part(g, name + ' eye', body)
        RB.ring_y(g, 'Bearing %s %s' % (br['name'], name), c[0], c[1], (y0 + y1) / 2.0, br['B'], br['D'] / 2,
                  br['d'] / 2)
        cs, a, bw, d3 = e['circlip']
        g0 = e['groove'][0]
        RB.part(g, 'Circlip DIN 472 %gx%g %s' % (b['D'], cs, name),
                RB.ytube(c[0], c[1], *sorted((g0, g0 + s * cs)), e['groove_d'] / 2.0, e['groove_d'] / 2.0 - bw))

    def part(g, name, tb, material='Steel'):
        if not (name.startswith('Pin ') and name[4:] in PINS):
            return plain_part(g, name, tb, material)
        e = PINS[name[4:]]
        b = XP.SRB[e['bearing']]
        (x, _, z), _ = centre_mm(tb)
        s = e['side']
        body = RB.ycyl(x, z, *sorted(e['collar']), e['collar_d'] / 2.0)
        mp.union(body, RB.ycyl(x, z, *sorted((e['ring'][0] - s * 0.01, e['pin_end'])), b['d'] / 2.0))
        sd, sl = e['screw']
        depth = sl - e['washer_t'] + 2.0
        mp.cut(body, RB.ycyl(x, z, *sorted((e['pin_end'] + s * 1.0, e['pin_end'] - s * depth)), sd / 2.0))
        o = plain_part(g, name, body, material)
        Dw, tw, hole = XP.END_WASHER[e['bearing']]
        plain_part(g, 'End washer pin %s' % e['pin'], RB.ytube(x, z, *sorted(e['washer']), Dw / 2.0, hole / 2.0))
        plain_part(g, 'Cap screw M%dx%d pin %s' % (sd, sl, e['pin']),
                   mp.cap_screw(float(sd), float(sl), (x, e['washer'][1], z), (0.0, -float(s), 0.0)))
        return o

    RB.rod_eye, RB.part = rod_eye, part
    msg = RB.build()
    return RB, msg


# ---------- the bag gripper: the jaws' ends, the circlips, the welds ----------
def weld_bead(g, xf, sgn, yh, zh, h):
    """A fillet weld all round inside the tube's end, drawn as a quarter round of radius h in the corner between the
    plug's face at xf and each of the tube's four inner walls, on the tube's end side of the plug (sgn)."""
    hb = g.bg.TUBE['b'] / 2.0 - g.bg.TUBE['t']
    x0, x1 = sorted((xf, xf + sgn * h))
    bead = None
    for ay, s in (('y', 1), ('y', -1), ('z', 1), ('z', -1)):
        if ay == 'y':
            yw = yh + s * hb
            c = mp.cyl((xf, yw, zh - hb), (xf, yw, zh + hb), h)
            bx = mp.box(x0, x1, *sorted((yw, yw - s * h)), zh - hb, zh + hb)
        else:
            zw = zh + s * hb
            c = mp.cyl((xf, yh - hb, zw), (xf, yh + hb, zw), h)
            bx = mp.box(x0, x1, yh - hb, yh + hb, *sorted((zw, zw - s * h)))
        q = mp.intersect(c, bx)
        bead = q if bead is None else mp.union(bead, q)
    return bead


def patch_bag(g, weld):
    """BagGripper as StandardParts loads it, changed where this chapter decides."""
    E = XP.jaw_ends()
    tx = g.L['tube_x']
    hb = g.bg.TUBE['b'] / 2.0 - g.bg.TUBE['t']
    plain_xcyl, plain_ep, plain_jaw, plain_part, plain_tube = g.xcyl, g.end_plate, g.jaw_bodies, g.part, g.xtube
    sh = E['short']
    a1, b1, d31, s1, g1, m1, n1 = E['din471']
    a, bw, d3, cs, gd, gm, gn = E['din472']
    bo1 = g.L['boss'][1]

    def xcyl(x0, x1, r, y, z):
        if near(x0, tx - 8.0) and near(r, g.L['stub_d'] / 2):                 # the long stub, from the plug's inner face
            return plain_xcyl(tx - 12.0, x1, r, y, z)
        if near(x0, tx - 0.01) and near(x1, g.L['bearing_x'][0]):             # its shoulder, from the plug's outer face
            return plain_xcyl(tx - 4.01, x1, r, y, z)
        if near(x0, -tx + 8.0) and near(x1, -g.L['bearing_x'][1] - 2.0):      # the short stub: shoulder, seat, groove
            b = plain_xcyl(-tx + 12.0, -sh['end'], r, y, z)
            mp.union(b, plain_xcyl(-sh['shoulder'][0] + 0.01, -sh['shoulder'][1], 18.0, y, z))
            mp.cut(b, plain_tube(-sh['groove'][1], -sh['groove'][0], r + 0.01, g1 / 2.0, y, z))
            return b
        return plain_xcyl(x0, x1, r, y, z)

    def end_plate():
        b = plain_ep()
        for s in (1, -1):
            e = E['locating' if s > 0 else 'floating']
            y, z = s * g.YH, g.ZH
            mp.union(b, plain_tube(e['lip'][0], e['lip'][1], 31.01, XP.BOSS['lip'] / 2.0, y, z))
            mp.union(b, plain_tube(bo1 - 0.01, E['boss'][1], g.L['boss_r'], 31.0, y, z))
            mp.cut(b, plain_xcyl(e['groove'][0], e['groove'][1], gd / 2.0, y, z))
        return b

    def circlips():
        out = []
        for s in (1, -1):
            e = E['locating' if s > 0 else 'floating']
            out.append(plain_tube(e['groove'][0], e['groove'][0] + cs, gd / 2.0, gd / 2.0 - bw, s * g.YH, g.ZH))
        return out

    def part(gr, name, tb, material='Steel'):
        o = plain_part(gr, name, tb, material)
        if name in ('End plate 1', 'End plate 2'):
            k0 = 1 if name.endswith('1') else 3
            for j, c in enumerate(circlips()):
                plain_part(gr, 'Circlip DIN 472 62x2 %d' % (k0 + j), c if name.endswith('1') else g.turned(c))
        return o

    def jaw_bodies():
        out = plain_jaw()
        yh, zh = g.YH, g.ZH
        out.append(('Fillet weld long', weld_bead(g, tx - 4.0, 1, yh, zh, weld), 'Steel'))
        out.append(('Fillet weld short', weld_bead(g, -tx + 4.0, -1, yh, zh, weld), 'Steel'))
        out.append(('Circlip DIN 471 30x1.5', plain_tube(-sh['groove'][0] - s1, -sh['groove'][0], g1 / 2.0 + b1, g1 / 2.0,
                                                         yh, zh), 'Steel'))
        return out

    g.xcyl, g.end_plate, g.jaw_bodies, g.part = xcyl, end_plate, jaw_bodies, part
    return g


def build_bag(weld):
    SP = _load('StandardParts')
    plain_load, plain_box = SP._load, mp.box
    import bag_gripper as bgm
    tx = bgm.LAY['tube_x']
    hb = bgm.TUBE['b'] / 2.0 - bgm.TUBE['t']

    def box(x0, x1, y0, y1, z0, z1):
        b0 = orig['box']
        if near(y1 - y0, 2 * hb) and near(z1 - z0, 2 * hb):
            if near(x0, tx - 8.0) and near(x1, tx):
                return b0(tx - 12.0, tx - 4.0, y0, y1, z0, z1)                  # the plugs set 4 mm in for the weld
            if near(x0, -tx) and near(x1, -tx + 8.0):
                return b0(-tx + 4.0, -tx + 12.0, y0, y1, z0, z1)
        return b0(x0, x1, y0, y1, z0, z1)

    orig = {}

    def load(name):
        m = plain_load(name)
        if name != 'BagGripper':
            return m
        orig['box'] = mp.box                     # BagGripper reloads mdfc_parts as it loads: patch after that
        mp.box = box
        return patch_bag(m, weld)

    SP._load = load
    try:
        msg = SP.build()
    finally:
        SP._load = plain_load
        mp.box = orig.get('box', plain_box)
    return SP, msg


# ---------- the compliance unit: the O-ring's gland and the gallery plugs ----------
def build_unit():
    CU = _load('ComplianceUnit')
    G = XP.gland()
    P = XP.gallery_plug()
    plain_zcyl, plain_part, plain_gal = CU.zcyl, CU.part, CU.galleries
    r_tube = CU.AIR['tube'][0] / 2.0
    h_or = XP.pi * XP.GLAND['W'] ** 2 / 4.0 / XP.GLAND['E']        # the squeezed ring's length, its area kept

    def zcyl(z0, z1, r, x=0.0, y=0.0):
        if near(r, r_tube + 2.5) and near(z1 - z0, 3.5) and x == 0.0 and y == 0.0:
            zc = (z0 + z1) / 2.0
            return plain_zcyl(zc - G['width'] / 2.0, zc + G['width'] / 2.0, G['groove_d'] / 2.0, x, y)
        return plain_zcyl(z0, z1, r, x, y)

    def part(g, name, tb, material='Steel'):
        if name == 'O-ring 20x2.5':
            (x, y, z), _ = centre_mm(tb)
            tb = mp.tube((0.0, 0.0, z - h_or / 2.0), (0.0, 0.0, z + h_or / 2.0), G['groove_d'] / 2.0, r_tube)
        elif name.startswith('Plug '):
            (x, y, z), size = centre_mm(tb)
            if abs(size[1] - size[2]) < 1e-3:                         # along x, its outer end at the +x edge
                e = x + size[0] / 2.0
                p = lambda t: (e - t, y, z)
            else:                                                      # along y, its outer end at the -y edge
                e = y - size[1] / 2.0
                p = lambda t: (x, e + t, z)
            tb = mp.cyl(p(P['cb_t'] + P['i']), p(P['cb_t'] - 0.01), P['d'] / 2.0)
            mp.union(tb, mp.cyl(p(P['cb_t']), p(0.0), P['dk'] / 2.0))
        return plain_part(g, name, tb, material)

    def galleries(b, z, x_edge, y_edge, z_face, plugs, g, tag):
        plain_gal(b, z, x_edge, y_edge, z_face, plugs, g, tag)
        if not plugs:
            return
        for ys in (1, -1):
            mp.cut(b, mp.cyl((x_edge - P['cb_t'] - P['tapped'], ys * CU.YA, z), (x_edge + 1.0, ys * CU.YA, z), P['d'] / 2.0))
            mp.cut(b, mp.cyl((x_edge - P['cb_t'], ys * CU.YA, z), (x_edge + 1.0, ys * CU.YA, z), P['cb_d'] / 2.0))
        mp.cut(b, mp.cyl((0.0, -y_edge - 1.0, z), (0.0, -y_edge + P['cb_t'] + P['tapped'], z), P['d'] / 2.0))
        mp.cut(b, mp.cyl((0.0, -y_edge - 1.0, z), (0.0, -y_edge + P['cb_t'], z), P['cb_d'] / 2.0))

    CU.zcyl, CU.part, CU.galleries = zcyl, part, galleries
    msg = CU.build()
    return CU, msg


def split_tube(des, CU):
    """The tube's outside split where it leaves its boss: s6 above, in the boss, and g6 below, through the O-ring."""
    z = CU.Z_TOP - CU.AIR['boss'][1]
    for o in des.rootComponent.allOccurrences:
        if o.component.name != 'Tube 20x16':
            continue
        comp = o.component
        faces = adsk.core.ObjectCollection.create()
        for b in comp.bRepBodies:
            for f in b.faces:
                g = f.geometry
                bb = f.boundingBox
                if isinstance(g, adsk.core.Cylinder) and near(g.radius * 10, CU.AIR['tube'][0] / 2.0, 1e-3) \
                        and bb.minPoint.z * 10 < z - 0.01 < bb.maxPoint.z * 10:
                    faces.add(f)
        if faces.count:
            pi_ = comp.constructionPlanes.createInput()
            pi_.setByOffset(comp.xYConstructionPlane, VI(z * CM))
            pl = comp.constructionPlanes.add(pi_)
            pl.isLightBulbOn = False
            comp.features.splitFaceFeatures.add(comp.features.splitFaceFeatures.createInput(faces, pl, True))
    return z


# ---------- the vacuum gripper: through bolts for the plate ----------
def build_vacuum():
    VG = _load('VacuumGripper')
    plain_zcyl, plain_part = VG.zcyl, VG.part
    ps = VG.L['plate_screw']
    zb, t = VG.L['beam'][1], VG.L['beam_t']
    p0, p1 = VG.L['plate']
    w_d, w_D, w_t = XP.VAC['washer']
    J = XP.vacuum_joint()

    def zcyl(z0, z1, r, x=0.0, y=0.0):
        if near(r, ps['d'] / 2.0) and near(z0, zb + t - ps['L'] - 0.5):
            return plain_zcyl(p1 - 1.0, p0 + 1.0, ps['clear_r'], x, y)         # through the plate, not tapped
        return plain_zcyl(z0, z1, r, x, y)

    SL = XP.SLEEVE
    sx = XP.sleeve_fixing()
    holes = lambda x, y: [(x + SL['pcd'] / 2.0 * math.cos(math.radians(a)), y + SL['pcd'] / 2.0 * math.sin(math.radians(a)))
                          for a in (90.0, 210.0, 330.0)]

    def part(g, name, tb, material='Steel'):
        if name == 'Plate':                                            # the sleeves' tapped holes, the thread's depth
            for _, x, y in VG.cups_xy():
                for hx, hy in holes(x, y):
                    mp.cut(tb, plain_zcyl(p0 - sx['thread'] - 1.0, p0 + 0.01, SL['d'] / 2.0, hx, hy))
            return plain_part(g, name, tb, material)
        if name.startswith('Sleeve '):                                 # each sleeve on a foot flange, three M4 screws
            (x, y, _), _ = centre_mm(tb)
            mp.union(tb, mp.tube((x, y, p0), (x, y, p0 + SL['flange_t']), SL['flange_d'] / 2.0, VG.L['sleeve_r'][1]))
            for hx, hy in holes(x, y):
                mp.cut(tb, plain_zcyl(p0 - 1.0, p0 + SL['flange_t'] + 1.0, SL['d'] / 2.0 + 0.2, hx, hy))
            o = plain_part(g, name, tb, material)
            k = name.split()[-1]
            for j, (hx, hy) in enumerate(holes(x, y)):
                plain_part(g, 'Cap screw M4x%d s%s %d' % (sx['L'], k, j + 1),
                           mp.cap_screw(SL['d'], float(sx['L']), (hx, hy, p0 + SL['flange_t']), (0, 0, -1)))
            return o
        if not name.startswith('Cap screw M6x14 '):
            return plain_part(g, name, tb, material)
        k = name.split()[-1]
        (x, y, _), _ = centre_mm(tb)
        zw = zb + t
        plain_part(g, 'Washer ISO 7089 6 head %s' % k, mp.tube((x, y, zw), (x, y, zw + w_t), w_D / 2.0, w_d / 2.0))
        o = plain_part(g, 'Cap screw M6x%d %s' % (J['L'], k), mp.cap_screw(6.0, float(J['L']), (x, y, zw + w_t), (0, 0, -1)))
        plain_part(g, 'Washer ISO 7089 6 nut %s' % k, mp.tube((x, y, p1 - w_t), (x, y, p1), w_D / 2.0, w_d / 2.0))
        plain_part(g, 'Nut ISO 4032 M6 %s' % k, mp.nut(6.0, (x, y, p1 - w_t), (0, 0, -1), m=XP.VAC['nut_m']))
        return o

    VG.zcyl, VG.part = zcyl, part
    msg = VG.build()
    return VG, msg


# ---------- after the build: by hand ----------
def mark(f, value, kind, des, new):
    if not f.attributes.itemByName('mdfc', 'fit'):
        new.append(f)
    f.attributes.add('mdfc', 'fit', value)
    f.appearance = MF.appearance(des, MF.COLOURS[kind])


def describe(root, start, text, out):
    for o in root.allOccurrences:
        if o.component.name.startswith(start) and o.component.bRepBodies.count:
            o.component.description = text
            out[o.component.name] = text


def by_hand(key, des, mod):
    root = des.rootComponent
    new, changed, desc = [], [], {}
    if key == 'robot':
        eyes = {e['name'] + ' eye': e for e in XP.eyes()}
        for o in root.allOccurrences:
            e = eyes.get(o.component.name)
            if not e:
                continue
            for b in o.bRepBodies:
                for f in b.faces:
                    g = f.geometry
                    if isinstance(g, adsk.core.Cylinder) and near(g.radius * 10, XP.SRB[e['bearing']]['D'] / 2.0, 1e-3):
                        mark(f, 'H7 Ra 3.2', 'bearing seat', des, new)
                        changed.append(f)
        need = {(r['bearing'], r['rod']): r['needs'] for r in XP.robot_bearings()}
        for start, nm, rod in (('Bearing 22213 E Parallel rod', '22213 E', True), ('Bearing 22207 E Levelling', '22207 E', True),
                               ('Bearing 22207 E triangle', '22207 E', False), ('Bearing 61836', '61836', False),
                               ('Bearing 61856 MA', '61856 MA', False)):
            c = need[(nm, rod)]
            describe(root, start, 'SKF %s%s' % (nm, '' if c == 'Normal' else '/' + c), desc)
    elif key == 'bag':
        lib = MF.library_material(STEEL_1020)
        for o in root.allOccurrences:
            nm = o.component.name
            if nm.startswith(('Tube', 'Stub shaft', 'Fillet weld')):
                for b in o.bRepBodies:
                    b.material = lib
            if nm.startswith('End plate'):
                for b in o.bRepBodies:
                    for f in b.faces:
                        g = f.geometry
                        bb = f.boundingBox
                        cx = (bb.minPoint.x + bb.maxPoint.x) * 5.0
                        cy = (bb.minPoint.y + bb.maxPoint.y) * 5.0
                        x0, x1 = sorted((abs(bb.minPoint.x * 10), abs(bb.maxPoint.x * 10)))
                        if isinstance(g, adsk.core.Cylinder) and near(g.radius * 10, 31.0, 1e-3) and cx * cy < 0 \
                                and x0 < mod_bag_x()[1] and x1 > mod_bag_x()[0]:
                            mark(f, 'H7 Ra 3.2', 'bearing seat', des, new)
                            changed.append(f)
            if nm.startswith('Stub shaft'):
                for b in o.bRepBodies:
                    for f in b.faces:
                        g = f.geometry
                        if isinstance(g, adsk.core.Cylinder) and near(g.radius * 10, 15.0, 1e-3):
                            f.attributes.add('mdfc', 'process', 'ground after welding, both stubs of a jaw in one setting')
        bg = XP.bag_6206()
        c = XP.nearest_zero_class(bg['least'], bg['most'])
        describe(root, 'Bearing 6206', 'SKF 6206-2RS1/%s' % c, desc)
    elif key == 'unit':
        z = mod.Z_TOP - mod.AIR['boss'][1]
        for o in root.allOccurrences:
            if o.component.name != 'Tube 20x16':
                continue
            for b in o.bRepBodies:
                for f in b.faces:
                    g = f.geometry
                    bb = f.boundingBox
                    if isinstance(g, adsk.core.Cylinder) and near(g.radius * 10, mod.AIR['tube'][0] / 2.0, 1e-3):
                        above = bb.minPoint.z * 10 > z - 0.01
                        mark(f, 's6 Ra 0.8' if above else 'g6 Ra 0.4', 'pressed' if above else 'sliding', des, new)
                        changed.append(f)
        H = XP.HIWIN['unit']
        describe(root, 'Block HGH25CA', 'HIWIN %s' % H['block'], desc)
        describe(root, 'Rail HGR25', 'HIWIN %s' % H['rail'], desc)
        describe(root, 'Plug ', 'DIN 908 M10 x 1, a sealing ring under its collar', desc)
        G = XP.gland()
        describe(root, 'O-ring 20x2.5', 'O-ring 20 x 2.5 in a groove %.2f x %.2f (ERIKS Table 3.D-1)'
                 % (G['groove_d'], G['width']), desc)
    elif key == 'vacuum':
        J = XP.vacuum_joint()
        describe(root, 'Cap screw M6x%d' % J['L'], 'ISO 4762 M6 x %d - 8.8 - zinc-plated' % J['L'], desc)
        describe(root, 'Washer ISO 7089 6', 'ISO 7089 6 - 200 HV', desc)
        describe(root, 'Nut ISO 4032 M6', 'ISO 4032 M6 - 8', desc)
        describe(root, 'Cap screw M4x%d' % XP.sleeve_fixing()['L'], 'ISO 4762 M4 x %d - 8.8 - zinc-plated' % XP.sleeve_fixing()['L'], desc)
    return changed, new, desc


def mod_bag_x():
    import bag_gripper as bgm
    return bgm.LAY['bearing_x']


def build(key='bag', weld=4.0):
    if key not in KEYS:
        raise ValueError('No assembly called %s: Chapter 55 details %s.' % (key, ', '.join(KEYS)))
    refuse(weld)
    app = adsk.core.Application.get()
    mod, msg = {'robot': build_robot, 'bag': lambda: build_bag(weld), 'unit': build_unit,
                'vacuum': build_vacuum}[key]()
    des = adsk.fusion.Design.cast(app.activeProduct)
    z_split = split_tube(des, mod) if key == 'unit' else None     # before the census: a face split later shares
    msg51 = MF.apply(key)                                          # its attributes with its other half
    changed, new, desc = by_hand(key, des, mod)
    _last.clear()
    _last.update(key=key, weld=weld, mod=mod, changed=changed, new=new, desc=desc, mf=dict(MF._last))
    return ('Palletizer in detail: the %s, built by %s; then Chapter 51\'s materials and fits, and by hand %d faces '
            '(%d new) and %d descriptions.\n%s\n%s\n\n[%s]'
            % (key, {'robot': 'PalletizerRobot', 'bag': 'StandardParts', 'unit': 'ComplianceUnit',
                     'vacuum': 'VacuumGripper'}[key], len(changed), len(new), len(desc), msg.split('\n')[0],
               msg51.split('\n')[0], VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('weld', 'The plugs\' fillet weld leg (mm)', 4.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            vals = {k: float(inputs.itemById(k).value) * 10.0 for k, _, _ in FIELDS}
            key = inputs.itemById('key').selectedItem.name
            ui.messageBox(build(key, **vals))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('PalletizerDetail failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('key', 'Assembly', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in KEYS:
            dd.listItems.add(k, k == 'bag')
        for k, label, v in FIELDS:
            cmd.commandInputs.addValueInput(k, label, 'mm', adsk.core.ValueInput.createByReal(v * CM))


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcPalletizerDetail'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Palletizer in detail', 'Chapter 55: the palletizer in detail')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PalletizerDetail failed:\n%s' % traceback.format_exc())
