# =============================================================================
# ComplianceUnit.py  -  Chapter 48 of "Mechanical Design in Autodesk Fusion with Claude": the compliance unit, a
# vertical float between the tool changer and the vacuum or bag gripper. A top plate to ISO 9409-1-160-6-M10 with two
# cheeks; an HGR25 rail on each cheek; a carriage with two wings on four HGH25CA blocks, its underside repeating the
# tool plate's, hanging by four lugs on stop blocks on the cheeks; four springs between round seats screwed into the
# top plate and the carriage; the air's galleries and the tube that carries it across the float into a sealed
# receiver; a sensor that sees the carriage rise. The sizes are lib/compliance_unit.py's. Written from PROMPT.txt
# beside it; checked by checks/compliance_unit_check.py.
#
#   INPUTS        the dialog: the gripper (vacuum or bag), the stroke (mm), the springs' preload (N)
#   DERIVED       compliance_unit: LAY, GUIDE, the springs chosen to Shigley, the ring factor with each gripper
#   HARD RULES    build() refuses the clamp gripper (the changer's ring would open in the worst stop), a stroke over
#                 the rails' 30 mm, and a preload under what keeps the carriage down in the stops
#   CONVENTIONS   z = 0 the unit's top face on the tool plate, z up; groups Body (grounded) and Carriage;
#                 set_state(lift) raises the carriage by its transform
#   ACCEPTANCE    the message; the check: no interference at rest and raised but the springs, the carriage's lugs on
#                 the stops, nothing below the gripper's face, the rails' and blocks' places, the tube in its
#                 receiver, the sensor's gap, the ISO pattern on both faces and the passages lined up, weighed
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
PROJECT = os.path.dirname(HERE)                         # the project's folder: what this script writes goes in its results/
ROOT = PROJECT                                          # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import tool_changer           # noqa: E402
import clamp_gripper          # noqa: E402
import vacuum_gripper         # noqa: E402
import bag_gripper            # noqa: E402
import compliance_unit as cu  # noqa: E402
import mdfc_parts as mp       # noqa: E402
for _m in (tool_changer, clamp_gripper, vacuum_gripper, bag_gripper, cu, mp):
    importlib.reload(_m)                      # the modules this one builds on, as Fusion keeps old ones

VERSION = 'ComplianceUnit 1.2 (2026-11-03)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

L = cu.LAY
GD = cu.GUIDE
ISO = cu.ISO_B                 # the six M10 on the 160 circle, in the body's angles (the body turned 90 deg to the changer)
PIN = cu.PIN_B
AIR = cu.AIR
HZ = cu.height()
Z_TOP = -L['top_t']                                   # the top plate's underside
Z_RAIL = (Z_TOP, Z_TOP - HZ['rail'])                  # the rails, from the top plate down
Z_CAR = (Z_RAIL[1], Z_RAIL[1] - L['carriage_t'])      # the carriage's plate; its underside is the gripper's face
Z_WING = Z_TOP - L['stroke']                          # the wings' tops, the stroke under the top plate
BLOCKS = (Z_WING - L['gap'] - GD['L'] / 2, Z_WING - L['gap'] - GD['L'] / 2 - L['d'])   # the blocks' centres
Z_GAL = (AIR['z_gal'], Z_CAR[1] + 10.0 + (L['carriage_t'] - 10.0) / 2)   # the galleries' axes: housing, carriage
PASS = [(AIR['r'] * math.cos(math.radians(a_)), AIR['r'] * math.sin(math.radians(a_))) for a_ in AIR['angles']]
XA = max(abs(x) for x, y in PASS)                     # the passages along the galleries
YA = max(abs(y) for x, y in PASS)                     # the galleries along x at y = +-YA


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


def mirrored_x(tb):
    """A copy turned half a turn about z: the -x cheek's parts from the +x cheek's."""
    b = mp.tbm().copy(tb)
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.pi, V3(0, 0, 1), P3(0, 0, 0))
    mp.tbm().transform(b, m)
    return b


# ---------- the spring, as Chapter 46's: swept once and copied ----------
def spring_body(root, sp):
    D, d, L1 = sp['D'], sp['d'], sp['L1']
    tmp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = tmp.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    pts = adsk.core.ObjectCollection.create()
    turns = sp['Na'] + 2.0
    n = int(turns * 12)
    z_top, z_bot = -d / 2, -L1 + d / 2
    for i in range(n + 1):
        t = i / n
        th = 2 * math.pi * turns * t
        u = t * turns
        if u <= 1.0:
            z = z_top - d * u
        elif u >= turns - 1.0:
            z = z_bot + d * (turns - u)
        else:
            z = (z_top - d) + ((z_bot + d) - (z_top - d)) * (u - 1.0) / (turns - 2.0)
        pts.add(P3(D / 2 * math.cos(th) * CM, D / 2 * math.sin(th) * CM, z * CM))
    spl = sk.sketchCurves.sketchFittedSplines.add(pts)
    path = comp.features.createPath(spl, False)
    pi_ = comp.constructionPlanes.createInput()
    pi_.setByDistanceOnPath(path, VI(0.0))
    pl = comp.constructionPlanes.add(pi_)
    sk2 = comp.sketches.add(pl)
    c = sk2.modelToSketchSpace(P3(D / 2 * CM, 0, z_top * CM))
    sk2.sketchCurves.sketchCircles.addByCenterRadius(c, d / 2 * CM)
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk2.profiles.item(0), path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    sw.add(inp)
    tb = mp.tbm().copy(comp.bRepBodies.item(0))
    tmp.deleteMe()
    return tb


def moved(tb, dx, dy, dz):
    b = mp.tbm().copy(tb)
    m = adsk.core.Matrix3D.create()
    m.translation = V3(dx * CM, dy * CM, dz * CM)
    mp.tbm().transform(b, m)
    return b


# ---------- the air's path, the same in the housing and in the carriage ----------
def galleries(b, z, x_edge, y_edge, z_face, plugs, g, tag):
    """Cut into b the four passages from the face z_face to the galleries at z, the galleries along x at y = +-YA from
    their plugged ends at x_edge, and the one along y at x = 0 from its plugged end at -y_edge; add the plugs."""
    r = AIR['gallery'] / 2
    for x, y in PASS:
        if z < z_face:                                                       # down from the face to the gallery's foot
            mp.cut(b, zcyl(z - r, z_face + 1.0, AIR['passage'] / 2, x, y))
        else:                                                                # up from the face to the gallery's top
            mp.cut(b, zcyl(z_face - 1.0, z + r, AIR['passage'] / 2, x, y))
    for ys in (1, -1):
        mp.cut(b, mp.cyl((-XA - r, ys * YA, z), (x_edge + 1.0, ys * YA, z), r))
    mp.cut(b, mp.cyl((0.0, -y_edge - 1.0, z), (0.0, YA + r, z), r))
    if plugs:
        for k, ys in enumerate((1, -1)):
            part(g, 'Plug %s %d' % (tag, k + 1), mp.cyl((x_edge - 6.0, ys * YA, z), (x_edge, ys * YA, z), r))
        part(g, 'Plug %s 3' % tag, mp.cyl((0.0, -y_edge, z), (0.0, -y_edge + 6.0, z), r))


# ---------- the body ----------
def body(root, sp):
    g = group(root, 'Body', True)
    px, py = L['plate']
    b = mp.box(-px / 2, px / 2, -py / 2, py / 2, Z_TOP, 0.0)
    mp.union(b, zcyl(0.0, 8.0, 49.99))                                  # the spigot into the tool plate's recess
    for a in ISO:
        x, y = at(80.0, a)
        mp.cut(b, zcyl(Z_TOP - 1.0, 1.0, 5.5, x, y))
        mp.cut(b, zcyl(Z_TOP - 1.0, Z_TOP + 10.5, 8.75, x, y))            # counterbored from below
    xp, yp = at(80.0, PIN)
    mp.cut(b, zcyl(-8.0, 9.0, 5.0, xp, yp))
    x0, x1 = L['cheek_x']
    cy = L['cheek_y']
    for xs in (-1, 1):                                                       # the cheeks, machined with the plate
        a, c = sorted((xs * x0, xs * x1))
        mp.union(b, mp.box(a, c, -cy, cy, Z_RAIL[1], Z_TOP + 0.01))
    for x, y in L['spring_xy']:
        mp.cut(b, zcyl(Z_TOP - 1.0, 1.0, 5.0, x, y))                        # the seats' M10 studs, tapped
    # the tube's boss under the plate and its bore up to the galleries
    mp.union(b, zcyl(Z_TOP - AIR['boss'][1], Z_TOP + 0.01, AIR['boss'][0]))
    mp.cut(b, zcyl(Z_TOP - AIR['boss'][1] - 1.0, Z_GAL[0], AIR['tube'][0] / 2))
    galleries(b, Z_GAL[0], px / 2, py / 2, 0.0, True, g, 'housing')
    b_ref = [b]
    for k, a in enumerate(ISO):
        x, y = at(80.0, a)
        part(g, 'Cap screw M10x25 %d' % (k + 1), mp.cap_screw(10.0, 25.0, (x, y, Z_TOP + 10.5), (0, 0, 1)))
    part(g, 'Dowel pin 10x17', zcyl(-8.0, 9.0, 5.0, xp, yp))
    # the rails on the cheeks' inner faces, their screws tapped into the cheeks
    rail_y = GD['WR'] / 2
    rail = mp.box(x0 - GD['HR'], x0, -rail_y, rail_y, Z_RAIL[1], Z_RAIL[0])
    ry = [Z_RAIL[0] - 29.0 - 60.0 * k for k in range(4)]
    for z in ry:
        for xs in (1, -1):
            mp.cut(b_ref[0], mp.cyl((xs * (x0 - 1.0), 0.0, z), (xs * (x0 + 16.0), 0.0, z), 3.0))
        mp.cut(rail, mp.cyl((x0 - GD['HR'] - 1.0, 0.0, z), (x0 + 1.0, 0.0, z), 3.0))
        mp.cut(rail, mp.cyl((x0 - GD['HR'] - 1.0, 0.0, z), (x0 - GD['HR'] + 9.0, 0.0, z), 5.5))   # counterbored, D 11 h 9
    # the holes in the cheeks for the stop blocks' M8
    for xs in (1, -1):
        for ys in (1, -1):
            ym = ys * (L['lug_y'][0] + L['lug_y'][1]) / 2
            for z in L['stop_screws']:
                mp.cut(b_ref[0], mp.cyl((xs * (x0 - 1.0), ym, z), (xs * (x1 + 1.0), ym, z), 4.0))
    part(g, 'Housing', b, 'Aluminum')
    for k, xs in enumerate((1, -1)):
        part(g, 'Rail HGR25 %d' % (k + 1), rail if xs > 0 else mirrored_x(rail))
        for j, z in enumerate(ry):
            sc = mp.cap_screw(6.0, 20.0, (x0 - GD['HR'] + 9.0, 0.0, z), (1, 0, 0))
            part(g, 'Cap screw M6x20 %d' % (4 * k + j + 1), sc if xs > 0 else mirrored_x(sc))
    # the tube, pressed into its boss, down into the receiver
    ai = cu.air()
    part(g, 'Tube 20x16', mp.tube((0.0, 0.0, ai['tube_bot']), (0.0, 0.0, ai['tube_top']), AIR['tube'][0] / 2, AIR['tube'][1] / 2))
    # the spring seats: round bars screwed into the top plate by an M10 stud, a spigot into each spring's top
    r_o, r_sp, h_sp = L['seat']
    zs = L['seat_bottom']
    for k, (x, y) in enumerate(L['spring_xy']):
        st = zcyl(zs, Z_TOP, r_o, x, y)
        mp.union(st, zcyl(Z_TOP - 0.01, -4.0, 5.0, x, y))
        mp.union(st, zcyl(zs - h_sp, zs + 0.01, r_sp, x, y))
        part(g, 'Spring seat %d' % (k + 1), st, 'Aluminum')
        part(g, 'Spring %d' % (k + 1), moved(sp, x, y, zs))
    # the stops: a block on each cheek's inner face either side of the rail, two M8 from the cheek's outside; the
    # carriage's lugs rest on their tops
    lz1 = Z_WING - L['lug_t']
    lz0 = lz1 - L['stop_h']
    k = 0
    for xs in (1, -1):
        for ys in (1, -1):
            k += 1
            a, c = sorted((xs * L['stop_x'][0], xs * L['stop_x'][1]))
            y_a, y_b = sorted((ys * L['lug_y'][0], ys * L['lug_y'][1]))
            sb = mp.box(a, c, y_a, y_b, lz0, lz1)
            ym = ys * (L['lug_y'][0] + L['lug_y'][1]) / 2
            for j, z in enumerate(L['stop_screws']):
                mp.cut(sb,mp.cyl((xs * (x0 + 1.0), ym, z), (xs * (x0 - 12.0), ym, z), 4.0))
                part(g, 'Cap screw M8x30 %d' % (2 * (k - 1) + j + 1), mp.cap_screw(8.0, 30.0, (xs * x1, ym, z), (-xs, 0, 0)))
            part(g, 'Stop block %d' % k, sb, 'Aluminum')
    # the sensor: an M12 proximity sensor 40 long looking along -y at the +x wing's side face, 2 mm off it, its face's
    # lower edge `sense` over the wing's top: the wing's face enters its field once the carriage has risen that much,
    # and passes it, 2 mm off, through the rest of the stroke; held by a bracket under the top plate
    xs_ = (L['wing_x'][0] + L['wing_x'][1]) / 2
    zc = Z_WING + L['sense'] + 6.0
    y0 = L['wing_y'] + 2.0
    part(g, 'Sensor M12', mp.cyl((xs_, y0, zc), (xs_, y0 + 40.0, zc), 6.0), 'ABS Plastic')
    part(g, 'Sensor bracket', mp.box(xs_ - 10.5, xs_ + 10.5, y0 + 4.0, y0 + 38.0, zc + 6.0, Z_TOP), 'Aluminum')
    return g


def carriage(root):
    g = group(root, 'Carriage')
    cx, cy = L['carriage_xy']
    b = mp.box(-cx, cx, -cy, cy, Z_CAR[1], Z_CAR[0])
    w0, w1 = L['wing_x']
    for xs in (-1, 1):
        a, c = sorted((xs * w0, xs * w1))
        mp.union(b, mp.box(a, c, -L['wing_y'], L['wing_y'], Z_CAR[0] - 0.01, Z_WING))
    # the gripper's face, as the tool plate's: the 100 H7 recess 10 deep, six M10 tapped 15 deep, the pin hole
    mp.cut(b, zcyl(Z_CAR[1] - 1.0, Z_CAR[1] + 10.0, 50.0))
    for a in ISO:
        x, y = at(80.0, a)
        mp.cut(b, zcyl(Z_CAR[1] - 1.0, Z_CAR[1] + 15.0, 4.25, x, y))
    xp, yp = at(80.0, PIN)
    mp.cut(b, zcyl(Z_CAR[1] - 1.0, Z_CAR[1] + 10.0, 5.0, xp, yp))
    # the lugs that rest on the stops, two on each wing, and the spigots for the springs' feet
    for xs in (1, -1):
        for ys in (1, -1):
            a, c = sorted((xs * (L['lug_x'][0] - 0.01), xs * L['lug_x'][1]))
            y_a, y_b = sorted((ys * L['lug_y'][0], ys * L['lug_y'][1]))
            mp.union(b, mp.box(a, c, y_a, y_b, Z_WING - L['lug_t'], Z_WING))
    r_o, r_sp, h_sp = L['seat']
    for x, y in L['spring_xy']:
        mp.union(b, zcyl(Z_CAR[0] - 0.01, Z_CAR[0] + h_sp, r_sp, x, y))
    # the receiver: the tube slides in its bore, sealed by an O-ring in a groove near its top
    rr, rh = AIR['receiver']
    mp.union(b, zcyl(Z_CAR[0] - 0.01, Z_CAR[0] + rh, rr))
    mp.cut(b, zcyl(Z_GAL[1], Z_CAR[0] + rh + 1.0, AIR['tube'][0] / 2))
    z_or = Z_CAR[0] + rh - 7.0
    mp.cut(b, zcyl(z_or - 1.75, z_or + 1.75, AIR['tube'][0] / 2 + 2.5))
    galleries(b, Z_GAL[1], cx, cy, Z_CAR[1], True, g, 'carriage')
    part(g, 'Carriage', b, 'Aluminum')
    part(g, 'O-ring 20x2.5', mp.tube((0.0, 0.0, z_or - 1.7), (0.0, 0.0, z_or + 1.7), AIR['tube'][0] / 2 + 2.45, AIR['tube'][0] / 2), 'Rubber')
    k = 0
    for xs in (1, -1):
        for zc in BLOCKS:
            k += 1
            # the block from the wing's face to the rail's running face, and its two skirts down the rail's sides to
            # H1 = 5.5 over the rail's foot, 0.1 clear of the rail
            z0, z1 = zc - GD['L'] / 2, zc + GD['L'] / 2
            xr = L['cheek_x'][0] - GD['HR']
            bl = mp.box(w1, xr, -GD['W'] / 2, GD['W'] / 2, z0, z1)
            for sy in (1, -1):
                a, c = sorted((sy * (GD['WR'] / 2 + 0.1), sy * GD['W'] / 2))
                mp.union(bl, mp.box(xr - 0.01, L['cheek_x'][0] - 5.5, a, c, z0, z1))
            part(g, 'Block HGH25CA %d' % k, bl if xs > 0 else mirrored_x(bl))
    return g


# ---------- the states ----------
def set_state(G_, lift=0.0):
    m = adsk.core.Matrix3D.create()
    m.translation = V3(0, 0, lift * CM)
    G_['Carriage'].transform2 = m
    adsk.doEvents()


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        nm = o.component.name
        if nm.endswith(')') and ' (' in nm:
            nm = nm[:nm.rindex(' (')]
        wds = nm.split()
        if len(wds) > 1 and wds[-1].isdigit():
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


def refuse(gripper, stroke, F_pre):
    d = cu.design()
    if gripper not in ('vacuum', 'bag'):
        raise ValueError('The %s gripper: with the unit in the stack the changer\'s ring would open in the worst stop '
                         '(%.2f).' % (gripper, d['ring'].get(gripper, {}).get(HZ['H'], 0.0)))
    if stroke > L['stroke'] + 1e-9:
        raise ValueError('A stroke of %.0f mm: the rails give %.0f mm.' % (stroke, L['stroke']))
    need = d['grippers'][gripper]['springs']['F_pre']
    if F_pre < need - 1e-6:
        raise ValueError('A preload of %.0f N: the stops that throw the load up need %.0f N with the %s gripper.'
                         % (F_pre, need, gripper))
    return d


_last = {}


def build(gripper='bag', stroke=None, F_pre=None):
    stroke = L['stroke'] if stroke is None else stroke
    d0 = cu.design()
    F_pre = d0['F_pre'] if F_pre is None else F_pre
    d = refuse(gripper, stroke, F_pre)
    sp = d['coil']
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    spb = spring_body(root, sp)
    G_ = {'Body': body(root, spb), 'Carriage': carriage(root)}
    mp.colour(des, G_['Body'], 'Paint - Enamel Glossy (Grey)')
    mp.colour(des, G_['Carriage'], 'Paint - Enamel Glossy (Yellow)')
    set_state(G_, 0.0)
    _last.update(groups=G_, gripper=gripper, coil=sp)
    app.activeViewport.fit()
    pl = parts_list(root)
    g = d['grippers'][gripper]
    return ('Compliance unit for the %s gripper: %.0f mm tall, a stroke of %.0f mm on two HGR25 rails and four HGH25CA '
            'blocks (fSL %.2f in the worst stop); four springs %.1f x %.0f, %.1f N each installed; the changer\'s ring '
            'factor %.2f.\n\nParts: %d components; %s.\n\n[%s]'
            % (gripper, HZ['H'], stroke, g['guides']['fSL'], sp['d'], sp['D'], sp['F1'], d['ring'][gripper][HZ['H']],
               sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            gr = ins.itemById('gripper').selectedItem.name
            ui.messageBox(build(gr, ins.itemById('stroke').value, ins.itemById('F_pre').value), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('gripper', 'The gripper', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in ('bag', 'vacuum', 'clamp'):
            dd.listItems.add(k, k == 'bag')
        ins.addValueInput('stroke', 'The stroke (mm)', '', VI(L['stroke']))
        ins.addValueInput('F_pre', 'The springs\' preload (N)', '', VI(cu.design()['F_pre']))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'ComplianceUnitCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Compliance unit', 'The palletizer\'s compliance unit of Chapter 48')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ComplianceUnit could not start:\n' + traceback.format_exc())
