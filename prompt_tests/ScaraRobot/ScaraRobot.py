# =============================================================================
# ScaraRobot.py  -  Chapter 38 of "Mechanical Design in Autodesk Fusion with Claude": the SCARA put together as one
# robot. A base column carrying the J1 module on its top, a first arm on J1's output flange carrying the J2 module at
# its end, and a second arm on J2's output flange whose end is the ball screw spline of Chapter 37 with its belts and
# motors. The modules are ScaraJointModules.module() and the arm's end ScaraToolJoints.design() and build_parts(),
# both scripts loaded from their own files, not copied; every height comes from scara_design's LAY38 and heights().
# Written from projects/scara_robot/ScaraRobot/PROMPT.txt alone (a prompt test).
#
#   INPUTS        the dialog: the J1 unit's size, CSG 40 or 45 (the 40 only for comparison), 45
#   DERIVED       scara_design.LAY38 and heights(); the modules from ScaraJointModules.module(), the CSG-45's twelve
#                 M8 case bolts added to its bolt table; the arm's end from ScaraToolJoints.design(), build_parts()
#   HARD RULES    build() refuses a J1 unit other than a CSG-40 or a CSG-45
#   CONVENTIONS   three groups: Base (grounded) and First arm, their frames at J1's unit's input face; Second arm,
#                 its frame at J2's axis on its base plate's top face. J1 and J2 As-built revolutes about z; J3 the
#                 carriage slides, J4 the tool's shaft turns on it; each unit's output held to the next arm by an
#                 As-built rigid joint; the units' inputs, the nuts and the motors held still in their groups.
#                 pose(q1, q2, j3, j4) in degrees and millimetres, J3 down positive
#   ACCEPTANCE    the message; 93 components in 41 kinds with the CSG-45; no interference where the links meet at
#                 seven poses; the mass properties measured part by part; the tool flange's underside 359 mm above
#                 the floor at J3 0; a CSG-32 refused
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
import companion_paths as CP    # noqa: E402            every lib/ folder on sys.path; CP.script(name) finds a script
import scara_design as sd       # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(sd)
importlib.reload(mp)


def _load(name):
    """Another script of the companion, run from its own file as a module of its own (loaded, not copied)."""
    path = CP.script(name)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


SJM = _load('ScaraJointModules')        # module(): a joint module; parts_list()
STJ = _load('ScaraToolJoints')          # design() and build_parts(): the arm's end; the joints' helpers
SJM.BOLT[45] = (8.0, 9.0)               # the CSG-45's case bolts: M8 in 9 mm holes (Table 29); twelve on 164 mm in sd.CSG

VERSION = 'ScaraRobot 1.0 (2026-11-11)'
CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
SIZES = (40, 45)                        # the J1 units it builds
ANCHOR_D, ANCHOR_EDGE = 14.0, 25.0      # the foot's anchor holes (M12 anchors) and their centres from the edges (assumed)
CLEAR = 3.0                             # the top flange's hole beyond the J1 motor flange's corners (assumed)
WINDOW = 5.0                            # the first arm's window beyond the J2 motor's flange on each side (assumed)
BOSS_J2, BOSS_NUT, PAD = 4.0, 2.0, 4.0  # the second arm's bosses beyond J2's output flange, the nuts' flanges and the
#                                         motors' flanges (assumed)
GREY, BLUE, YELLOW = ('Paint - Enamel Glossy (Grey)', 'Paint - Enamel Glossy (Blue)', 'Paint - Enamel Glossy (Yellow)')
L1 = sd.brief()['L1']


# ---------- the parts this script adds (mm, each in its group's frame) ----------
def base_parts(base, size1):
    """The base, z = 0 at J1's unit's input face: the steel foot with four anchor holes on the floor, the aluminium
    tube, and the top flange the J1 module plate sits on, its hole clear of the J1 motor's flange."""
    L = sd.LAY38
    z1 = sd.heights(size1)['z1']
    zf = -z1                                            # the floor
    a = L['foot'] / 2
    foot = mp.box(-a, a, -a, a, zf, zf + L['foot_t'])
    e = a - ANCHOR_EDGE
    for sx in (1, -1):
        for sy in (1, -1):
            mp.cut(foot, STJ.zc(sx * e, sy * e, zf - 1, zf + L['foot_t'] + 1, ANCHOR_D / 2))
    out = [SJM.part(base, 'Base foot', foot, 'Steel')]
    z_top = L['z_col'] - z1                             # the column's top face, under the J1 module plate
    z_fl = z_top - L['top_t']
    out.append(SJM.part(base, 'Base column', mp.tube((0, 0, zf + L['foot_t']), (0, 0, z_fl), L['col_r'],
                                                     L['col_r'] - L['col_t']), 'Aluminum'))
    r_hole = sd.MOTOR[1]['flange'] / 2 * math.sqrt(2) + CLEAR
    out.append(SJM.part(base, 'Base top flange', mp.tube((0, 0, z_fl), (0, 0, z_top), L['top_r'], r_hole), 'Aluminum'))
    return out


def first_arm_beam(first, size1):
    """The first arm, z = 0 at J1's unit's input face: an aluminium box from a1_back behind J1 to a1_front beyond J2,
    on J1's output flange, closed at both ends, with a window through its top and bottom at J2 for the J2 motor."""
    L = sd.LAY38
    h = sd.heights(size1)
    zb, zt = h['a1_bot'] - h['z1'], h['a1_top'] - h['z1']
    x0, x1 = -L['a1_back'], L1 + L['a1_front']
    w, t = L['a1_w'] / 2, L['a1_t']
    beam = mp.box(x0, x1, -w, w, zb, zt)
    mp.cut(beam, mp.box(x0 + t, x1 - t, -w + t, w - t, zb + t, zt - t))
    c = sd.MOTOR[2]['flange'] / 2 + WINDOW
    mp.cut(beam, mp.box(L1 - c, L1 + c, -c, c, zb - 1, zt + 1))
    return SJM.part(first, 'First arm beam', beam, 'Aluminum')


def structure(o4, x_end):
    """The second arm in its frame (x along the arm from J2, y across, z up from the base plate's top face), in place
    of ScaraToolJoints' plates and walls: 4 mm skins over the whole arm from a2_back behind J2 to the arm's end, each
    keeping the face of its plate that faces into the arm; the full 12 and 10 mm only in bosses round J2's output
    flange, the nuts and the motors' mounts; 3 mm side walls and a back wall; the holes for build_parts' nuts, shafts
    and bolts where its plates had them."""
    L, L8 = sd.LAY37, sd.LAY38
    s = sd.SP[o4['size']]
    xs, hw, H, tb, tt = L['x_s'], L['half_w'], L['H'], L['t_base'], L['t_top']
    sk, w = L8['skin'], L8['wall2']
    xa = -L8['a2_back']
    m = {j: (o4['belts'][j]['x_m'], o4['belts'][j]['y_m']) for j in (3, 4)}
    body = mp.box(xa, x_end, -hw, hw, -sk, 0.0)                        # the base skin
    mp.union(body, mp.box(xa, x_end, -hw, hw, H, H + sk))               # the top skin
    mp.union(body, mp.box(xa, x_end, hw - w, hw, -sk, H + sk))          # the side walls
    mp.union(body, mp.box(xa, x_end, -hw, -hw + w, -sk, H + sk))
    mp.union(body, mp.box(xa, xa + w, -hw, hw, -sk, H + sk))            # the back wall
    mp.union(body, STJ.zc(0.0, 0.0, -tb, 0.0, sd.CSG[20]['O'] / 2 + BOSS_J2))       # on J2's output flange
    mp.union(body, STJ.zc(xs, 0.0, -tb, 0.0, s['A1'] / 2 + BOSS_NUT))               # the spline nut's flange under it
    mp.union(body, STJ.zc(xs, 0.0, H, H + tt, s['A2'] / 2 + BOSS_NUT))              # the screw nut's flange on it
    for j, z0, z1 in ((3, H, H + tt), (4, -tb, 0.0)):                               # J3 hangs from the top, J4 stands
        p = sd.MOTOR34[j]['flange'] / 2 + PAD
        mp.union(body, mp.box(m[j][0] - p, m[j][0] + p, m[j][1] - p, m[j][1] + p, z0, z1))
    zb0, zb1, zt0, zt1 = -tb - 1, 1.0, H - 1, H + tt + 1
    mp.cut(body, STJ.zc(xs, 0, zb0, zb1, s['D1'] / 2 + 0.1), STJ.zc(xs, 0, zt0, zt1, s['D2'] / 2 + 0.1))
    for j, z0, z1, bd in ((3, zt0, zt1, 5.0), (4, zb0, zb1, 4.0)):                   # the motors' shafts and bolts
        x, y = m[j]
        F = sd.MOTOR34[j]['flange']
        mp.cut(body, STJ.zc(x, y, z0, z1, STJ.SHAFT_D[j] / 2 + 1.0))
        for px, py in STJ.circle_pts(x, y, (F / 2 - bd) * math.sqrt(2), 4, math.pi / 4):
            mp.cut(body, STJ.zc(px, py, z0, z1, bd / 2 + 0.25))
    for px, py in STJ.circle_pts(xs, 0, s['W'] / 2, 4, math.pi / 4):               # the nuts' M4 flange bolts, blind
        mp.cut(body, STJ.zc(px, py, -tb - 1, -tb + 8.0, 2.0))
    for px, py in STJ.circle_pts(xs, 0, s['W2'] / 2, 4, math.pi / 4):
        mp.cut(body, STJ.zc(px, py, H + tt - 8.0, H + tt + 1, 2.0))
    return body


def second_arm(second, o4):
    """The arm's end from build_parts() in the second arm's group; its J3 motor bolts renamed apart from the J2
    module's M5 motor bolts; its plates and walls replaced by the skinned structure; its nuts and motors held still."""
    occ = STJ.build_parts(second.component, o4)
    ae = occ['Arm end']
    oc = ae.component.occurrences
    kids = [oc.item(i) for i in range(oc.count)]
    k = 0
    for c in kids:
        if c.component.name.startswith('Motor bolt M5'):
            k += 1
            c.component.name = 'J3 motor bolt M5 %d' % k
    old = [c for c in kids if c.component.name in ('Base plate', 'Top plate', 'Side wall left', 'Side wall right')]
    plate = [c for c in old if c.component.name == 'Base plate'][0]
    x_end = max(v.geometry.x for v in plate.component.bRepBodies.item(0).vertices) / CM    # the arm's end
    for c in old:
        c.deleteMe()
    STJ.part(ae, 'Second arm structure', structure(o4, x_end), 'Aluminum')
    for nm in ('Screw side', 'Spline side', 'J3 motor side', 'J4 motor side'):
        occ[nm].isGroundToParent = True
    return occ


def j2_module(first, size1):
    """The J2 module at the first arm's end: module() builds a module at its parent's origin, so it is built inside a
    body-less component placed at J2's axis on the arm's top (the J2 unit's input face)."""
    h = sd.heights(size1)
    place = mp.new_occ(first.component, 'J2 place', (L1, 0.0, h['z2'] - h['z1']))
    mod, st, out, inp, g = SJM.module(place.component, 'J2 module', 20, 2, 0.0)
    inp.isGroundToParent = True
    return mod, st, out, inp


# ---------- joints ----------
def child(occ, *names):
    """The proxy, in the root's context, of the occurrence reached from occ through the components named."""
    for nm in names:
        hit = None
        for c in occ.childOccurrences:
            if c.component.name == nm:
                hit = c
                break
        if hit is None:
            raise RuntimeError('no %s in %s' % (nm, occ.name))
        occ = hit
    return occ


def rigid(root, a, b, name):
    """An As-built rigid joint between two occurrences as they stand."""
    try:
        inp = root.asBuiltJoints.createInput(a, b, None)
        inp.setAsRigidJointMotion()
        j = root.asBuiltJoints.add(inp)
    except Exception:
        sk = a.component.sketches.add(a.component.xYConstructionPlane)
        sk.isVisible = False
        pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(a)
        inp = root.asBuiltJoints.createInput(a, b, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRigidJointMotion()
        j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


_last = {}


def pose(q1, q2, j3=0.0, j4=0.0):
    """J1 and J2 turned q1 and q2 deg about +z, J3 down by j3 mm, J4 turned j4 deg about +z."""
    J = _last['joints']
    for key, q in (('J1', q1), ('J2', q2), ('J4', j4)):
        j, sgn = J[key]
        j.jointMotion.rotationValue = sgn * math.radians(q)
    j, sgn = J['J3']
    j.jointMotion.slideValue = -sgn * j3 * CM
    adsk.doEvents()


# ---------- the mass properties, measured ----------
def link_of(o):
    """0: the base and J1's stator and input; 1: J1's output, the first arm and the J2 module but its output; 2: J2's
    output and the second arm."""
    path = o.fullPathName
    top = path.split('+')[0].rsplit(':', 1)[0]
    if top == 'Base':
        return 1 if '+J1 output:' in path else 0
    if top == 'First arm':
        return 2 if '+J2 output:' in path else 1
    return 2


def measured(root, size1):
    """Every part's mass, centre (mm, at rest) and own inertia about a vertical axis through its centre (kg m2), as
    scara_design.mass_model() takes them; the model with the motors at their specified masses and the units at the
    catalogue's, and Chapter 36's selection on it for both J1 units."""
    rows = []
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        pp = o.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
        m, c = pp.mass, pp.centerOfMass
        _, xx, yy, zz, xy, yz, xz = pp.getXYZMomentsOfInertia()            # kg cm2 about the world's axes
        rows.append((link_of(o), o.component.name, m, c.x / CM, c.y / CM, c.z / CM,
                     (zz - m * (c.x ** 2 + c.y ** 2)) * 1e-4))
    mm = sd.mass_model(rows, size1)
    return mm, {k: sd.sizing_measured(mm, k) for k in SIZES}


def parts_list(root):
    return SJM.parts_list(root)


def build(size1=45):
    if float(size1) not in [float(k) for k in SIZES]:
        raise ValueError('The J1 unit must be a CSG-40 or a CSG-45 (the 40 only for comparison), not a CSG-%g.'
                         % float(size1))
    size1 = int(float(size1))
    o1 = SJM.design(size1)
    o4 = STJ.design(20)
    h = sd.heights(size1)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    base = mp.new_occ(root, 'Base', (0.0, 0.0, h['z1']))
    base.isGrounded = True
    first = mp.new_occ(root, 'First arm', (0.0, 0.0, h['z1']))
    second = mp.new_occ(root, 'Second arm', (L1, 0.0, h['arm2']))
    plain = base_parts(base, size1)
    m1 = SJM.module(base.component, 'J1 module', size1, 1, 0.0)
    m1[3].isGroundToParent = True                       # J1's input held still in the base
    plain.append(first_arm_beam(first, size1))
    occ2 = second_arm(second, o4)                       # before the J2 module, so that no two parts share a name
    m2 = j2_module(first, size1)
    xs = sd.LAY37['x_s']
    J = {}
    J['J1'] = STJ.revolute_z(root, first, base, (0.0, 0.0, 0.0), 'J1 turns')
    J['J2'] = STJ.revolute_z(root, second, first, (0.0, 0.0, 0.0), 'J2 turns')
    carriage, arm_end, tool = (child(second, n) for n in ('J3 carriage', 'Arm end', 'Tool shaft'))
    J['J3'] = STJ.slider_z(root, carriage, arm_end, (xs, 0.0, 0.0), 'J3 slides')
    J['J4'] = STJ.revolute_z(root, tool, carriage, (xs, 0.0, 0.0), 'J4 turns')
    rigid(root, child(base, 'J1 module', 'J1 output'), first, 'J1 output holds the first arm')
    rigid(root, child(first, 'J2 place', 'J2 module', 'J2 output'), second, 'J2 output holds the second arm')
    _last.update(joints=J, size1=size1, occ=occ2)
    for o, col in ([(x, GREY) for x in plain] + [(m1[1], GREY), (m1[2], BLUE), (m1[3], YELLOW), (m2[1], GREY),
                   (m2[2], BLUE), (m2[3], YELLOW), (occ2['Arm end'], GREY), (occ2['Tool shaft'], BLUE)] +
                   [(occ2[n], YELLOW) for n in ('Screw side', 'Spline side', 'J3 motor side', 'J4 motor side')]):
        mp.colour(des, o, col)
    app.activeViewport.fit()
    b3, b4 = o4['belts'][3], o4['belts'][4]
    s1, s2 = o1['sel1'], o1['j2'][20]
    u20 = sd.CSG[20]
    msg = ['SCARA robot built. The units: J1 a CSG-%d-50-2UH%s, J2 a CSG-20-50-2UH, J3 and J4 a HIWIN SP%d ball screw '
           'spline driven through belts 3MR-%d and 3MR-%d.' % (size1, ' (for comparison only)' if size1 == 40 else '',
                                                               o4['size'], b3['stock'], b4['stock']),
           "Chapter 36, on the assumed masses: J1 averages %.1f N m (limit %d), life %.0f h; J2 averages %.1f N m "
           "(limit %d), life %.0f h on radial moves." % (s1['Tav'], sd.CSG[size1]['Tav'], s1['Lh'], s2['Tav'],
                                                        u20['Tav'], s2['Lh'])]
    try:
        mm, sz = measured(root, size1)
        msg.append("Measured part by part, the motors at their specified masses and the units at the catalogue's: "
                   "the first link %.2f kg, %.4f kg m2 about J1; the second link with gripper and payload %.2f kg, its "
                   "centre %.1f mm from J2, %.4f kg m2 about it. J1 lasts %.0f h on the CSG-45 and %.0f h on the "
                   "CSG-40 (%.0f h wanted); J2 peaks at %.1f N m (limit %d) and averages %.1f N m (limit %d)."
                   % (mm['m1'], mm['I1'], mm['m2'], mm['c2'] * 1000, mm['d'], sz[45]['j1']['Lh'], sz[40]['j1']['Lh'],
                      sd.brief()['life_h'], sz[size1]['j2']['Tpk'], u20['Trep'], sz[size1]['j2']['Tav'], u20['Tav']))
    except Exception as e:
        msg.append('The mass properties could not be measured: %s' % e)
    try:
        tf = [o for o in root.allOccurrences if o.component.name == 'Tool flange'][0]
        msg.append("The tool flange's underside is %.1f mm above the floor at J3 0." % (tf.boundingBox.minPoint.z / CM))
    except Exception:
        pass
    pl = parts_list(root)
    msg.append('Parts: %d components in %d kinds; %s.' % (sum(pl.values()), len(pl),
                                                          ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items()))))
    msg.append('[%s]' % VERSION)
    return '\n\n'.join(msg)


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('size1', 'J1 unit size (CSG 40 or 45)', 45.0)]


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
        cid = 'ScaraRobotCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'SCARA robot', 'The SCARA of Chapter 38 put together')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ScaraRobot could not start:\n' + traceback.format_exc())
