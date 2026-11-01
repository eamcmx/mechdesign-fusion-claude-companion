# =============================================================================
# AxleHousing.py  -  Chapter 30 of "Mechanical Design in Autodesk Fusion with Claude": the rear axle's housing and its
# two semi-floating wheel ends. The banjo housing is a drum round the axle that holds the carrier, a neck at its
# front with the flange the carrier bolts to, a rear cover, and the tubes out to the bearings, with the spring seats
# and the brakes' flanges; each wheel end is a half shaft with its splines, flange and studs, turning in a deep-groove
# bearing held in the tube's end by a retainer, with an oil seal. The sizing is lib/axle_design.py (wheel_end,
# housing_tube); the parts lib/mdfc_parts.py. Written from PROMPT.txt beside it; checked by
# checks/axle_housing_check.py.
#
#   INPUTS        the dialog: the shaft's body, the tube's diameter and wall, the bearing's bore
#   DERIVED       axle_design.wheel_end() and housing_tube(); the positions below
#   HARD RULES    build() refuses a shaft whose factor against yield in the hardest turn or in fatigue falls under
#                 1.5, a bearing that does not last the truck's life, a tube under 1.5 in the pothole
#   CONVENTIONS   the axle along Y, forward +X, up +Z, the axle's middle at the origin; two subassemblies of real
#                 parts (housing; wheel end L and R), every part a component; bearings from Table 11-2
#   ACCEPTANCE    the message; no interference; each half shaft turning on its bearing; the parts list
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

LIB = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import axle_design as ad        # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(ad)
importlib.reload(mp)

VERSION = 'AxleHousing 1.1 (2026-11-01)'
CM = 0.1
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
DRUM_R, DRUM_Y = 168.07, 135.0      # the drum round the axle: its outside reaches 196.93 mm above the road
NECK_R, NECK_X = 166.0, (60.0, 165.0)   # the neck that holds the carrier, its flange's face at x = 165
REAR_R, REAR_X = 130.0, (-185.0, -60.0) # the rear neck, past the drum's back; its cover's face at x = -185
SEAL_BORE, RET_IN = 50.0, 45.0      # the seal's bore radius; the retainer plate's and retaining ring's meeting radius
FILL_Z, DRAIN_Z = 30.0, -100.0
WALL = 8.0
SEAT_Y = 575.0                      # the spring seats
SPLINE = (35.0, 40.0, 8, 7.0)       # minor, major, number, width (mm)
Y_INNER = 100.0                     # the shaft's inner end in the side gear
STUD_PCD = 139.7


def design(d_body=48.0, D=80.0, t=8.0, d_seat=65):
    b = ad.brief()
    w = ad.wheel_end(b, d_body=d_body, d_seat=int(round(d_seat)))
    h = ad.housing_tube(b, w, D=D, t=t)
    return dict(b=b, w=w, h=h, d_body=d_body)


def refuse(Dz):
    w, h = Dz['w'], Dz['h']
    short = []
    if w['n_yield'] < 1.5:
        short.append('the shaft keeps %.2f against yield in the hardest turn' % w['n_yield'])
    if w['n_fatigue'] < 1.5:
        short.append('the shaft keeps %.2f in fatigue' % w['n_fatigue'])
    if w['L10_rev'] < w['rev_needed']:
        short.append('bearing %d lasts %.1f million turns of the %.1f million needed'
                     % (w['bearing']['d'], w['L10_rev'] / 1e6, w['rev_needed'] / 1e6))
    if h['n'] < 1.5:
        short.append('the tube keeps %.2f in the pothole' % h['n'])
    if short:
        raise ValueError('The wheel end falls short: %s.' % '; '.join(short))


# ---------- the housing ----------
def housing(root, Dz):
    """The banjo housing and everything fixed to it, the bearings' outer rings and seals included."""
    w, h = Dz['w'], Dz['h']
    asm = mp.new_occ(root, 'Housing')
    parts = []
    y_b, Bb = w['y_b'], w['bearing']
    y0 = y_b - Bb['B'] / 2.0
    y_end = y_b + Bb['B'] / 2.0 + 9.0                       # the tube's end, past the bearing and its retainer plate
    body = mp.tube((0, -DRUM_Y, 0), (0, DRUM_Y, 0), DRUM_R, DRUM_R - WALL)          # the drum round the axle
    for s in (-1, 1):                                                                # its end walls
        mp.union(body, mp.tube((0, s * DRUM_Y, 0), (0, s * (DRUM_Y - WALL), 0), DRUM_R, h['D'] / 2.0 - 0.01))
    for x0, x1, r in ((NECK_X[0], NECK_X[1], NECK_R), (REAR_X[1], REAR_X[0], REAR_R)):     # the front and rear necks
        mp.union(body, mp.tube((x0, 0, 0), (x1, 0, 0), r, r - WALL))
    flange = mp.intersect(mp.tube((NECK_X[1] - 14.0, 0, 0), (NECK_X[1], 0, 0), 205.0, NECK_R - WALL),
                          mp.box(NECK_X[1] - 15.0, NECK_X[1] + 1.0, -206.0, 206.0, ad.JOINT_BOTTOM, 206.0))
    mp.union(body, flange)                                                           # the carrier's flange, flat below
    mp.union(body, mp.tube((REAR_X[0], 0, 0), (REAR_X[0] + 8.0, 0, 0), 160.0, REAR_R - WALL))    # the cover's flange
    mp.cut(body, mp.cyl((NECK_X[0] - 1, 0, 0), (NECK_X[1] + 40.0, 0, 0), NECK_R - WALL))              # the openings
    mp.cut(body, mp.cyl((REAR_X[0] - 30.0, 0, 0), (0, 0, 0), REAR_R - WALL))
    for yb, zb in ad.joint_bolts():
        mp.cut(body, mp.cyl((NECK_X[1] - 15, yb, zb), (NECK_X[1] + 1, yb, zb), 6.05))
    for kk in range(10):
        t_ = 2 * math.pi * kk / 10
        mp.cut(body, mp.cyl((REAR_X[0] - 1, 145 * math.cos(t_), 145 * math.sin(t_)),
                            (REAR_X[0] + 9, 145 * math.cos(t_), 145 * math.sin(t_)), 4.05))
    for s in (-1, 1):
        mp.union(body, mp.tube((0, s * (DRUM_Y - 0.5 * WALL), 0), (0, s * (y_b - 60.0), 0), h['D'] / 2.0, h['d'] / 2.0))
        # the tube's end: the bearing's housing, its shoulder, the seal's bore, the brake's flange round it
        bh = mp.cyl((0, s * (y_b - 65.0), 0), (0, s * y_end, 0), Bb['D'] / 2.0 + 10.0)
        mp.cut(bh, mp.cyl((0, s * (y_b - 66.0), 0), (0, s * (y_b - 60.0), 0), h['d'] / 2.0))
        mp.cut(bh, mp.cyl((0, s * (y_b - 60.01), 0), (0, s * y0, 0), SEAL_BORE))
        mp.cut(bh, mp.cyl((0, s * (y0 - 0.01), 0), (0, s * (y_end + 1.0), 0), Bb['D'] / 2.0))
        mp.union(body, bh)
        fl = mp.tube((0, s * (y_b - 45.0), 0), (0, s * (y_b - 33.0), 0), 105.0, Bb['D'] / 2.0 + 9.99)
        for kk in range(4):
            t_ = math.pi / 4 + kk * math.pi / 2
            mp.cut(fl, mp.cyl((92 * math.cos(t_), s * (y_b - 46), 92 * math.sin(t_)),
                              (92 * math.cos(t_), s * (y_b - 32), 92 * math.sin(t_)), 6.6))
        mp.union(body, fl)
        seat = mp.box(-45.0, 45.0, s * SEAT_Y - 40.0, s * SEAT_Y + 40.0, h['D'] / 2.0 - 6.0, h['D'] / 2.0 + 10.0)
        mp.cut(seat, mp.cyl((0, s * SEAT_Y - 41, 0), (0, s * SEAT_Y + 41, 0), h['D'] / 2.0 - 0.01))
        mp.union(body, seat)
    o = mp.new_occ(asm.component, 'Banjo housing')
    mp.add_body(o, body, 'Banjo housing', 'Steel')
    parts.append(o)
    cover = mp.cyl((REAR_X[0] - 8.0, 0, 0), (REAR_X[0], 0, 0), 160.0)
    for kk in range(10):
        t_ = 2 * math.pi * kk / 10
        mp.cut(cover, mp.cyl((REAR_X[0] - 9, 145 * math.cos(t_), 145 * math.sin(t_)),
                             (REAR_X[0] + 1, 145 * math.cos(t_), 145 * math.sin(t_)), 4.5))
    for zz in (FILL_Z, DRAIN_Z):
        mp.cut(cover, mp.cyl((REAR_X[0] - 9, 0, zz), (REAR_X[0] + 1, 0, zz), 11.0))
    o = mp.new_occ(asm.component, 'Rear cover')
    mp.add_body(o, cover, 'Rear cover', 'Steel')
    parts.append(o)
    for kk in range(10):
        t_ = 2 * math.pi * kk / 10
        o = mp.new_occ(asm.component, 'Cover screw M8x20 %d' % (kk + 1))
        mp.add_body(o, mp.cap_screw(8.0, 20.0, (REAR_X[0] - 8.0, 145 * math.cos(t_), 145 * math.sin(t_)), (1, 0, 0)),
                    'Screw', 'Steel')
        parts.append(o)
    for kk, (yb, zb) in enumerate(ad.joint_bolts()):
        p0 = (NECK_X[1] - 14.0, yb, zb)
        o = mp.new_occ(asm.component, 'Carrier stud M12 %d' % (kk + 1))
        mp.add_body(o, mp.cyl(p0, (NECK_X[1] + 16.0 + 12.0, p0[1], p0[2]), 6.0), 'Stud', 'Steel')
        parts.append(o)
        o = mp.new_occ(asm.component, 'Carrier nut M12 %d' % (kk + 1))
        mp.add_body(o, mp.nut(12.0, (NECK_X[1] + 16.0, p0[1], p0[2]), (1, 0, 0)), 'Nut', 'Steel')
        parts.append(o)
    o = mp.new_occ(asm.component, 'Breather')
    mp.add_body(o, mp.cyl((0, 40.0, DRUM_R), (0, 40.0, DRUM_R + 14.0), 5.0), 'Breather', 'Brass')
    parts.append(o)
    for nm, zz in (('Filler plug M22', FILL_Z), ('Drain plug M22', DRAIN_Z)):
        o = mp.new_occ(asm.component, nm)
        plug = mp.cyl((REAR_X[0] - 8.0, 0, zz), (REAR_X[0] + 0.5, 0, zz), 10.95)
        mp.union(plug, mp.cyl((REAR_X[0] - 16.0, 0, zz), (REAR_X[0] - 8.0, 0, zz), 14.0))
        mp.add_body(o, plug, nm, 'Steel')
        parts.append(o)
    # the parts of each wheel end that stay with the housing: the bearing's outer ring, its retainer plate, the seal
    for side in (-1, 1):
        tag = 'R' if side > 0 else 'L'
        _, outer, _, _ = mp.bearing_bodies(Bb['d'], Bb['D'], Bb['B'], y0 if side > 0 else -y0 - Bb['B'])
        o = mp.new_occ(asm.component, 'Bearing 6213 outer ring %s' % tag)
        mp.add_body(o, outer, 'Outer ring', 'Steel')
        parts.append(o)
        yr = y_b + Bb['B'] / 2.0
        ret = mp.tube((0, side * yr, 0), (0, side * (yr + 8.0), 0), Bb['D'] / 2.0 - 0.01, RET_IN)
        o = mp.new_occ(asm.component, 'Retainer plate %s' % tag)
        mp.add_body(o, ret, 'Retainer plate', 'Steel')
        parts.append(o)
        o = mp.new_occ(asm.component, 'Oil seal 74x100x10 %s' % tag)
        mp.add_body(o, mp.tube((0, side * (y0 - 12.0), 0), (0, side * (y0 - 2.0), 0), SEAL_BORE - 0.05, 37.05),
                    'Seal', 'Rubber, Silicone')
        parts.append(o)
    return asm, parts


# ---------- a wheel end ----------
def wheel_end_asm(root, Dz, side):
    """The parts that turn: the half shaft, the bearing's inner ring and balls, the retaining ring, the studs.
    side = +1 (right) or -1 (left)."""
    w = Dz['w']
    s = side
    tag = 'R' if s > 0 else 'L'
    asm = mp.new_occ(root, 'Wheel end %s' % tag)
    parts = []
    y_b, Bb = w['y_b'], w['bearing']
    y0 = y_b - Bb['B'] / 2.0
    y_fl = y_b + Bb['B'] / 2.0 + 14.0                     # the flange's inner face, past the tube's end
    dmin, dmaj, n, wd = SPLINE
    sh = mp.spline_shaft((0, s * Y_INNER, 0), (0, s * (Y_INNER + 60.0), 0), dmin, dmaj, n, wd)
    mp.union(sh, mp.cyl((0, s * (Y_INNER + 59.99), 0), (0, s * (y0 - 40.0), 0), Dz['d_body'] / 2.0))   # the body
    mp.union(sh, mp.cyl((0, s * (y0 - 40.01), 0), (0, s * y0, 0), Bb['dS'] / 2.0))                   # the shoulder
    mp.union(sh, mp.cyl((0, s * (y0 - 0.01), 0), (0, s * y_fl, 0), Bb['d'] / 2.0))                  # the seat
    fl = mp.cyl((0, s * (y_fl - 0.01), 0), (0, s * (y_fl + 14.0), 0), 95.0)
    for kk in range(6):
        t_ = 2 * math.pi * kk / 6
        mp.cut(fl, mp.cyl((STUD_PCD / 2 * math.cos(t_), s * (y_fl - 1), STUD_PCD / 2 * math.sin(t_)),
                          (STUD_PCD / 2 * math.cos(t_), s * (y_fl + 15), STUD_PCD / 2 * math.sin(t_)), 7.0))
    mp.union(sh, fl)
    o = mp.new_occ(asm.component, 'Half shaft %s' % tag)
    mp.add_body(o, sh, 'Half shaft', 'Steel')
    parts.append(o)
    inner, _, balls, _ = mp.bearing_bodies(Bb['d'], Bb['D'], Bb['B'], y0 if s > 0 else -y0 - Bb['B'])
    for nm, body in (('inner ring', inner), ('balls', balls)):
        o = mp.new_occ(asm.component, 'Bearing 6213 %s %s' % (nm, tag))
        mp.add_body(o, body, nm, 'Steel')
        parts.append(o)
    yr = y_b + Bb['B'] / 2.0
    o = mp.new_occ(asm.component, 'Retaining ring %s' % tag)
    mp.add_body(o, mp.tube((0, s * yr, 0), (0, s * (yr + 8.0), 0), RET_IN - 0.5, Bb['d'] / 2.0), 'Retaining ring', 'Steel')
    parts.append(o)
    for kk in range(6):
        t_ = 2 * math.pi * kk / 6
        o = mp.new_occ(asm.component, 'Wheel stud M14 %s %d' % (tag, kk + 1))
        p0 = (STUD_PCD / 2 * math.cos(t_), s * (y_fl - 4.0), STUD_PCD / 2 * math.sin(t_))
        p1 = (p0[0], s * (y_fl + 14.0 + 40.0), p0[2])
        stud = mp.cyl((p0[0], s * (y_fl - 0.02), p0[2]), p1, 6.95)
        mp.union(stud, mp.cyl(p0, (p0[0], s * (y_fl - 0.01), p0[2]), 10.0))  # the head, against the flange
        mp.add_body(o, stud, 'Stud', 'Steel')
        parts.append(o)
    return asm, parts


# ---------- joints ----------
def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_y(root, moving, fixed, name):
    comp = moving.component
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0)).createForAssemblyContext(moving)
    J = adsk.fusion.JointDirections
    for direction in (J.YAxisJointDirection, J.ZAxisJointDirection, J.XAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
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


def build(d_body=48.0, D=80.0, t=8.0, d_seat=65):
    Dz = design(d_body, D, t, d_seat)
    refuse(Dz)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design_ = adsk.fusion.Design.cast(app.activeProduct)
    design_.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design_.rootComponent
    hs, _ = housing(root, Dz)
    hs.isGrounded = True
    ends = [wheel_end_asm(root, Dz, s)[0] for s in (-1, 1)]
    for e in ends:
        revolute_y(root, e, hs, 'Half shaft %s turns' % e.component.name[-1])
    for occ, c in ((hs, 'Paint - Enamel Glossy (Grey)'), (ends[0], 'Paint - Enamel Glossy (Blue)'),
                   (ends[1], 'Paint - Enamel Glossy (Blue)')):
        mp.colour(design_, occ, c)
    app.activeViewport.fit()
    w, h = Dz['w'], Dz['h']
    pl = parts_list(root)
    return ('Rear axle housing and wheel ends built: half shafts of 4140 Q&T, body %.0f mm, splines %d x %.0f x %.0f; '
            'factors %.2f against yield in the hardest turn, %.2f in fatigue, %.2f at the splines.\n'
            'Wheel bearings %d: %.1f million turns of %.1f needed.\n'
            'Tubes %.0f x %.0f: %.1f MPa in the pothole at the spring seat, factor %.2f.\n\n'
            'Parts: %d components; %s.\n\n[%s]'
            % (Dz['d_body'], SPLINE[2], SPLINE[0], SPLINE[1], w['n_yield'], w['n_fatigue'], w['n_spline'],
               6200 + w['bearing']['d'] // 5, w['L10_rev'] / 1e6, w['rev_needed'] / 1e6, h['D'], h['t'], h['sigma'],
               h['n'], sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('d_body', 'Half shaft body (mm)', 48.0), ('D', 'Tube outside diameter (mm)', 80.0),
          ('t', 'Tube wall (mm)', 8.0), ('d_seat', 'Wheel bearing bore (mm)', 65.0)]


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
