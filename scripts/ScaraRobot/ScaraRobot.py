# =============================================================================
# ScaraRobot.py  -  Chapter 38 of "Mechanical Design in Autodesk Fusion with Claude": the SCARA put together. The
# base column with the J1 module of Chapter 36 on its top; the first arm, an aluminium box beam on J1's output
# flange, with the J2 module of Chapter 36 on its end; the second arm, whose end is Chapter 37's box with the ball
# screw spline, its root on J2's output flange. The modules and the arm's end are built by their own scripts'
# functions, loaded, not copied. J1 to J4 are joints; each gear's output is held to its arm by a rigid joint.
# Written from PROMPT.txt beside it; checked by checks/scara_robot_check.py.
#
#   INPUTS        the dialog: the J1 unit's size (40 or 45)
#   DERIVED       scara_design: LAY38 and heights() for the layout, box_section() for the first arm
#   CONVENTIONS   world: J1's axis at x = y = 0, z up from the floor; at rest the arms lie along +X
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
LIB = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import scara_design as sd       # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(sd)
importlib.reload(mp)


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, os.path.join(os.path.dirname(HERE), rel))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


JM = _load('mdfc_scara_modules', os.path.join('ScaraJointModules', 'ScaraJointModules.py'))
TJ = _load('mdfc_scara_tool', os.path.join('ScaraToolJoints', 'ScaraToolJoints.py'))
JM.BOLT.setdefault(45, (8.0, 9.0))               # the CSG-45's case bolts: M8 in 9 mm holes (Table 29)

VERSION = 'ScaraRobot 1.0 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
L1, L2 = 350.0, 250.0
AL_PARTS = ('Servo motor J3', 'Servo motor J4', 'Pulley 56T nut J3', 'Pulley 72T nut J4', 'Pulley 28T motor J3',
            'Pulley 18T motor J4', 'Tool flange')


def zc(x, y, z0, z1, r, r_in=0.0):
    return mp.tube((x, y, z0), (x, y, z1), r, r_in) if r_in > 0 else mp.cyl((x, y, z0), (x, y, z1), r)


def part(parent, name, tb, material='Steel'):
    o = mp.new_occ(parent.component, name)
    mp.add_body(o, tb, name, material)
    return o


def build_parts(root, size1):
    """Four groups: the base (grounded) with the J1 module, the first arm with the J2 module, the second arm with
    Chapter 37's end, each in a frame of its own: the base's and the first arm's origin at J1's unit's input face,
    the second arm's at J2's axis on its base plate's top face."""
    L = sd.LAY38
    H = sd.heights(size1)
    z1 = H['z1']
    base = mp.new_occ(root, 'Base', (0.0, 0.0, z1))
    arm1 = mp.new_occ(root, 'First arm', (0.0, 0.0, z1))
    arm2 = mp.new_occ(root, 'Second arm', (L1, 0.0, H['arm2']))
    for o in (arm1, arm2):
        o.isGroundToParent = False
    # the base: a foot plate, a tube, a top flange under the J1 module plate (local z from -z1 at the floor)
    zf = -z1
    foot = mp.box(-L['foot'] / 2, L['foot'] / 2, -L['foot'] / 2, L['foot'] / 2, zf, zf + L['foot_t'])
    for sx in (1, -1):
        for sy in (1, -1):
            mp.cut(foot, zc(sx * (L['foot'] / 2 - 25), sy * (L['foot'] / 2 - 25), zf - 1, zf + L['foot_t'] + 1, 7.0))
    part(base, 'Base foot', foot)
    part(base, 'Base column', zc(0, 0, zf + L['foot_t'], -12.0 - L['top_t'], L['col_r'], L['col_r'] - L['col_t']), 'Aluminum')
    top = zc(0, 0, -12.0 - L['top_t'], -12.0, L['top_r'], L['col_r'] - L['col_t'] - 4.0)   # clear of the motor's flange
    part(base, 'Base top flange', top, 'Aluminum')
    JM.module(base.component, 'J1 module', size1, 1, 0.0)
    # the first arm: a box beam from behind J1 to beyond J2, on J1's output flange; a window at J2 for its motor
    B1 = sd.CSG[size1]['B']
    w, h, t = L['a1_w'], L['a1_h'], L['a1_t']
    x0, x1 = -L['a1_back'], L1 + L['a1_front']
    bx = mp.box(x0, x1, -w / 2, w / 2, B1, B1 + h)
    mp.cut(bx, mp.box(x0 + t, x1 - t, -w / 2 + t, w / 2 - t, B1 + t, B1 + h - t))
    F2 = sd.MOTOR[2]['flange']
    mp.cut(bx, mp.box(L1 - F2 / 2 - 3, L1 + F2 / 2 + 3, -F2 / 2 - 3, F2 / 2 + 3, B1 - 1, B1 + h + 1))
    part(arm1, 'First arm beam', bx, 'Aluminum')
    mount = mp.new_occ(arm1.component, 'J2 mount', (0.0, 0.0, B1 + h + 8.0))
    JM.module(mount.component, 'J2 module', 20, 2, L1)
    # the second arm: Chapter 37's end, and its root over J2's output flange, extending the box back to J2
    o37 = TJ.design(20)
    TJ.build_parts(arm2.component, o37)
    k = 0
    for o in arm2.component.allOccurrences:                 # the J3 motor's bolts share the J2 motor's names
        if o.component.name.startswith('Motor bolt M5'):
            k += 1
            o.component.name = 'J3 motor bolt M5 %d' % k
    second_arm(arm2, o37)
    return dict(base=base, arm1=arm1, arm2=arm2, mount=mount, o37=o37, H=H)


def second_arm(arm2, o37):
    """The second arm's structure in place of Chapter 37's plates and walls, which were a layout: 4 mm skins over
    the whole arm from 45 mm behind J2 to the box's end, the full 12 and 10 mm only in bosses round the nuts, the
    motors' mounts and J2's output flange; 3 mm walls and a back wall."""
    LT, L = sd.LAY37, sd.LAY38
    end = find(arm2, 'Arm end')
    for nm in ('Base plate', 'Top plate', 'Side wall left', 'Side wall right'):
        o = find(end, nm)
        (o.nativeObject or o).deleteMe()
    # deleting those parts took their aluminium with them: the arm's end's other aluminium parts fell back to steel
    for o in arm2.component.allOccurrences:
        if o.component.name in AL_PARTS:
            for body in o.component.bRepBodies:
                mp.set_material(body, 'Aluminum')
    s = sd.SP[20]
    xs, hw = LT['x_s'], LT['half_w']
    x0, x1 = -L['a2_back'], 290.0
    tb, tt, H = LT['t_base'], LT['t_top'], LT['H']
    b3, b4 = o37['belts'][3], o37['belts'][4]
    m3, m4 = (b3['x_m'], b3['y_m']), (b4['x_m'], b4['y_m'])
    sk = L['skin']
    # the base: skin on top, bosses down to the full thickness
    bp = mp.box(x0, x1, -hw, hw, -sk, 0.0)
    mp.union(bp, zc(xs, 0, -tb, 0.0, s['D1'] / 2 + 14.0))
    mp.union(bp, zc(0.0, 0.0, -tb, 0.0, 45.0))
    mp.union(bp, mp.box(m4[0] - 24, m4[0] + 24, m4[1] - 24, m4[1] + 24, -tb, 0.0))
    mp.cut(bp, zc(xs, 0, -tb - 1, 1, s['D1'] / 2 + 0.1))
    mp.cut(bp, zc(m4[0], m4[1], -tb - 1, 1, TJ.SHAFT_D[4] / 2 + 1.0))
    for px, py in TJ.circle_pts(m4[0], m4[1], (sd.MOTOR34[4]['flange'] / 2 - 4.0) * math.sqrt(2), 4, math.pi / 4):
        mp.cut(bp, zc(px, py, -tb - 1, 1, 2.25))
    for (xx, yy) in TJ.circle_pts(xs, 0, s['W'] / 2, 4, math.pi / 4):
        mp.cut(bp, zc(xx, yy, -tb - 1, -tb + 8.0, 2.0))
    # the top: skin on top, bosses down
    tp = mp.box(x0, x1, -hw, hw, H + tt - sk, H + tt)
    mp.union(tp, zc(xs, 0, H, H + tt, s['D2'] / 2 + 14.0))
    mp.union(tp, mp.box(m3[0] - 34, m3[0] + 34, m3[1] - 34, m3[1] + 34, H, H + tt))
    mp.cut(tp, zc(xs, 0, H - 1, H + tt + 1, s['D2'] / 2 + 0.1))
    mp.cut(tp, zc(m3[0], m3[1], H - 1, H + tt + 1, TJ.SHAFT_D[3] / 2 + 1.0))
    for px, py in TJ.circle_pts(m3[0], m3[1], (sd.MOTOR34[3]['flange'] / 2 - 5.0) * math.sqrt(2), 4, math.pi / 4):
        mp.cut(tp, zc(px, py, H - 1, H + tt + 1, 2.75))
    for (xx, yy) in TJ.circle_pts(xs, 0, s['W2'] / 2, 4, math.pi / 4):
        mp.cut(tp, zc(xx, yy, H + tt - 8.0, H + tt + 1, 2.0))
    st = mp.new_occ(arm2.component, 'Second arm structure')
    mp.add_body(st, bp, 'Base skin', 'Aluminum')
    mp.add_body(st, tp, 'Top skin', 'Aluminum')
    w = L['wall2']
    mp.add_body(st, mp.box(x0, x1, hw - w, hw, 0.0, H + tt - sk), 'Wall left', 'Aluminum')
    mp.add_body(st, mp.box(x0, x1, -hw, -hw + w, 0.0, H + tt - sk), 'Wall right', 'Aluminum')
    mp.add_body(st, mp.box(x0, x0 + w, -hw + w, hw - w, 0.0, H + tt - sk), 'Back wall', 'Aluminum')
    return st


# ---------- joints ----------
def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def local(moving, point):
    """A world point in the moving occurrence's component frame, for a top-level occurrence placed by translation."""
    if moving.assemblyContext is None:
        t = moving.transform2.translation
        return (point[0] - t.x * 10, point[1] - t.y * 10, point[2] - t.z * 10)
    return point


def revolute_z(root, moving, fixed, point, name):
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.ZAxisJointDirection, Jd.XAxisJointDirection, Jd.YAxisJointDirection):
        sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
        sk.isVisible = False
        pt = sk.sketchPoints.add(P3(*[v * CM for v in local(moving, point)])).createForAssemblyContext(moving)
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.05
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][k] * r0[c][k] for k in range(3)) for c in range(3)] for a in range(3)]
        wv = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        if abs(wv[2]) > 0.02 and abs(wv[0]) < 0.02 and abs(wv[1]) < 0.02:
            j.name = name
            return j, (1 if wv[2] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about z' % moving.name)


def rigid(root, moving, fixed, point, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(*[v * CM for v in local(moving, point)])).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def find(occ, name):
    for o in occ.childOccurrences:
        if o.component.name == name:
            return o
    for o in occ.childOccurrences:
        r = find(o, name)
        if r is not None:
            return r
    return None


_last = {}


def pose(q1, q2, j3=0.0, j4=0.0):
    """J1 and J2 in degrees from the arms lying along +X, J3 down in mm, J4 in degrees."""
    J = _last['joints']
    J['J1'][0].jointMotion.rotationValue = J['J1'][1] * math.radians(q1)
    J['J2'][0].jointMotion.rotationValue = J['J2'][1] * math.radians(q2)
    J['J3'][0].jointMotion.slideValue = -J['J3'][1] * j3 * CM
    J['J4'][0].jointMotion.rotationValue = J['J4'][1] * math.radians(j4)
    adsk.doEvents()


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        wds = o.component.name.split()
        if wds[-1].isdigit():
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


def build(size1=45):
    size1 = int(size1)
    if size1 not in (40, 45):
        raise ValueError('The J1 unit is a CSG-40 or a CSG-45 (Chapter 38); %d was asked.' % size1)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    g = build_parts(root, size1)
    g['base'].isGrounded = True
    base, arm1, arm2 = g['base'], g['arm1'], g['arm2']
    # the modules' and the arm's end's own moving groups: the inputs and the nuts' sides stay with their stators here
    for occ, names in ((base, ('J1 input',)), (arm1, ('J2 input',)),
                       (arm2, ('Screw side', 'Spline side', 'J3 motor side', 'J4 motor side'))):
        for nm in names:
            o = find(occ, nm)
            (o.nativeObject or o).isGroundToParent = True
    J = {}
    J['J1'] = revolute_z(root, arm1, base, (0.0, 0.0, g['H']['z1']), 'J1')
    J['J2'] = revolute_z(root, arm2, arm1, (L1, 0.0, g['H']['arm2']), 'J2')
    out1, out2 = find(base, 'J1 output'), find(arm1, 'J2 output')
    rigid(root, out1, arm1, (0.0, 0.0, 0.0), 'J1 output to the first arm')
    rigid(root, out2, arm2, (L1, 0.0, 0.0), 'J2 output to the second arm')
    car, ts, end = find(arm2, 'J3 carriage'), find(arm2, 'Tool shaft'), find(arm2, 'Arm end')
    J['J3'] = TJ.slider_z(root, car, end, (sd.LAY37['x_s'], 0, 0), 'J3')
    J['J4'] = TJ.revolute_z(root, ts, car, (sd.LAY37['x_s'], 0, 0), 'J4')
    _last['joints'] = J
    _last['groups'] = g
    for occ, col in ((base, 'Paint - Enamel Glossy (Grey)'), (arm1, 'Paint - Enamel Glossy (Grey)'),
                     (arm2, 'Paint - Enamel Glossy (Grey)')):
        mp.colour(des, occ, col)
    app.activeViewport.fit()
    pl = parts_list(root)
    return ('SCARA put together: J1 CSG-%d-50-2UH, J2 CSG-20-50-2UH, SP20 ball screw spline.\nParts: %d components; %s.\n\n[%s]'
            % (size1, sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('size1', 'J1 unit size (CSG)', 45.0)]


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
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'SCARA robot', 'The SCARA of Chapter 38')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ScaraRobot could not start:\n' + traceback.format_exc())
