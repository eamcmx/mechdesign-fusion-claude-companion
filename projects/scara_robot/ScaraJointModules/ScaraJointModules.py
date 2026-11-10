# =============================================================================
# ScaraJointModules.py  -  Chapter 36 of "Mechanical Design in Autodesk Fusion with Claude": the SCARA's J1 and J2
# modules. Each is a servo motor with its holding brake on a module plate, coupled to the wave generator of a Harmonic
# Drive CSG-2UH housed unit (strain wave gear with its cross roller output bearing): J1 a CSG-40-50-2UH, J2 a
# CSG-20-50-2UH. The unit is drawn from the catalogue's outline (Table 28: A, B, E, O, Q, S, T, U) with its parts
# inside at concept level: the housing with the circular spline, the cross rollers, the output flange, the
# flexspline's cup and the elliptical wave generator. The sizing is lib/scara_design.modules(). Written from
# PROMPT.txt beside it; checked by checks/scara_modules_check.py.
#
#   INPUTS        the dialog: the J1 unit's size (the J2 unit's is 20)
#   DERIVED       scara_design.modules(): each unit's average torque, life, speeds, the motors, the bearing
#   HARD RULES    build() refuses a J1 unit whose repeated peak, average torque, input speeds or life fall short
#   CONVENTIONS   each module along +Z from the housing's input face, the J1 module at the origin and the J2 module
#                 400 mm along X; in each, three subassemblies: stator (grounded in the module), output, input;
#                 As-built revolute joints about the module's axis and a Motion Link of 50 turns in to one turn out,
#                 the other way
#   ACCEPTANCE    the message; no interference at rest or with the output turned; the units' outline dimensions
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
import scara_design as sd       # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(sd)
importlib.reload(mp)

VERSION = 'ScaraJointModules 1.0 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
RATIO = 50
X_J2 = 400.0                                   # the J2 module's place along X in this design (mm)
BOLT = {40: (8.0, 9.0), 20: (5.0, 5.5), 32: (6.0, 6.6)}   # the case bolts: size, hole (Table 29)
PLATE_T = {1: 12.0, 2: 8.0}                    # the module plate's thickness (mm)
MOTOR_BOLT = {1: 8.0, 2: 5.0}


def design(size1=40):
    o = sd.modules()
    o['size1'] = int(size1)
    o['sel1'] = o['j1'].get(int(size1)) or sd.hd_select(
        sd.leg_pattern(sd.brief(), (400.0, -150.0), (400.0, 150.0), 2.0, 0.2, 1, rest=0.05), int(size1))
    return o


def refuse(o):
    s, u = o['sel1'], sd.CSG[o['size1']]
    short = []
    if s['Tpk'] > u['Trep']:
        short.append('a repeated peak of %.1f N m against its limit of %d' % (s['Tpk'], u['Trep']))
    if s['Tav'] > u['Tav']:
        short.append('an average torque of %.1f N m against its limit of %d' % (s['Tav'], u['Tav']))
    if s['ni_max'] > u['nmax']:
        short.append('an input speed of %.0f rev/min against %d' % (s['ni_max'], u['nmax']))
    if s['Lh'] < sd.brief()['life_h']:
        short.append('a life of %.0f h against %.0f' % (s['Lh'], sd.brief()['life_h']))
    if short:
        raise ValueError('The CSG-%d-50-2UH falls short at J1: %s.' % (o['size1'], '; '.join(short)))


# ---------- geometry (world mm; each module's axis is Z through (x0, 0)) ----------
def along_z(x, y, z0, z1, r, r_in=0.0):
    return mp.tube((x, y, z0), (x, y, z1), r, r_in) if r_in > 0 else mp.cyl((x, y, z0), (x, y, z1), r)


def part(parent, name, tb, material='Steel'):
    o = mp.new_occ(parent.component, name)
    mp.add_body(o, tb, name, material)
    return o


def ellipse_part(parent, name, x0, a, b_, z0, z1, bore):
    """An elliptical cylinder about the axis (x0, 0) from z0 to z1, its long axis along X, with a round bore: sketched
    and extruded in a component of its own (the temporary-body manager makes no ellipse)."""
    o = mp.new_occ(parent.component, name)
    comp = o.component
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z0 * CM))
    sk = comp.sketches.add(comp.constructionPlanes.add(ci))
    sk.isVisible = False
    c = sk.modelToSketchSpace(P3(x0 * CM, 0, z0 * CM))
    ma = sk.modelToSketchSpace(P3((x0 + a) * CM, 0, z0 * CM))
    mi = sk.modelToSketchSpace(P3(x0 * CM, b_ * CM, z0 * CM))
    sk.sketchCurves.sketchEllipses.add(c, ma, mi)
    sk.sketchCurves.sketchCircles.addByCenterRadius(c, bore * CM)
    prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, VI((z1 - z0) * CM))
    body = ext.add(inp).bodies.item(0)
    body.name = name
    mp.set_material(body, 'Steel')
    return o


def unit_geometry(size):
    """The positions inside the unit, from its outline: the rollers, the output flange's hub, the cup."""
    u = sd.CSG[size]
    A, B, E, Q = u['A'], u['B'], u['E'], u['Q']
    d_r = 0.06 * A                                  # the rollers' diameter, drawn (not the maker's)
    r_b = Q / 2 - 0.707 * d_r - 0.3                 # the rollers' circle
    z_b = E - 0.707 * d_r - 1.0
    r_h = r_b - 0.707 * d_r - 0.3                   # the output flange's hub inside the rollers
    z_h0 = z_b - 0.707 * d_r - 1.0                  # the hub's lower face
    r_fo = Q / 2 - 1.0                              # the flexspline's cup
    wall = 0.02 * A
    return dict(A=A, B=B, E=E, Q=Q, O=u['O'], S=u['S'], T=u['T'], U=u['U'], d_r=d_r, r_b=r_b, z_b=z_b, r_h=r_h,
                z_h0=z_h0, r_fo=r_fo, r_fi=r_fo - wall, z_wg=(2.0, 2.0 + 0.35 * (z_h0 - 4.0)))


def module(root, name, size, joint, x0):
    """One joint module: its stator (housing, rollers, module plate, motor with brake, bolts), its output (flange,
    flexspline) and its input (wave generator, motor shaft)."""
    g = unit_geometry(size)
    m = sd.MOTOR[joint]
    pt = PLATE_T[joint]
    mod = mp.new_occ(root, name, (0.0, 0.0, 0.0))
    st = mp.new_occ(mod.component, 'J%d stator' % joint)
    out = mp.new_occ(mod.component, 'J%d output' % joint)
    inp = mp.new_occ(mod.component, 'J%d input' % joint)
    for o in (out, inp):
        o.isGroundToParent = False
    bs, bh = BOLT[size]
    u = sd.CSG[size]
    # the housing: the circular spline and the bearing's outer ring, with the case bolts' holes
    h = along_z(x0, 0, 0.0, g['E'], g['A'] / 2, g['Q'] / 2)
    for k in range(u['nc']):
        t = 2 * math.pi * (k + 0.5) / u['nc']
        mp.cut(h, along_z(x0 + u['b'] / 2 * math.cos(t), u['b'] / 2 * math.sin(t), -1, g['E'] + 1, bh / 2))
    part(st, 'CSG-%d-50-2UH housing' % size, h)
    # the cross rollers: one row, their axes alternately at +45 and -45 deg in the plane through the unit's axis
    n = int(2 * math.pi * g['r_b'] / (1.45 * g['d_r']))
    rl = None
    for k in range(n):
        t = 2 * math.pi * k / n
        c = (x0 + g['r_b'] * math.cos(t), g['r_b'] * math.sin(t), g['z_b'])
        s = 1 if k % 2 == 0 else -1
        e = (math.cos(t) * 0.7071 * s, math.sin(t) * 0.7071 * s, 0.7071)
        hl = g['d_r'] / 2 * 0.98
        r = mp.cyl(tuple(c[i] - e[i] * hl for i in range(3)), tuple(c[i] + e[i] * hl for i in range(3)), g['d_r'] / 2 * 0.98)
        rl = r if rl is None else mp.union(rl, r)
    part(st, 'Cross rollers %d' % size, rl)
    # the module plate under the housing, the motor under it
    F = m['flange']
    side = max(g['A'], F) + 30.0
    pl = mp.box(x0 - side / 2, x0 + side / 2, -side / 2, side / 2, -pt, 0.0)
    mp.cut(pl, along_z(x0, 0, -pt - 1, 1, max(g['U'] / 2 + 3.0, 0.3 * g['Q'])))
    for k in range(u['nc']):
        t = 2 * math.pi * (k + 0.5) / u['nc']
        mp.cut(pl, along_z(x0 + u['b'] / 2 * math.cos(t), u['b'] / 2 * math.sin(t), -pt - 1, 1, bs / 2))   # tapped
    mb = MOTOR_BOLT[joint]
    corners = [(x0 + sx * (F / 2 - mb), sy * (F / 2 - mb)) for sx in (1, -1) for sy in (1, -1)]
    for cx, cy in corners:
        mp.cut(pl, along_z(cx, cy, -pt - 1, 1, mb / 2))
    part(st, 'Module plate J%d' % joint, pl, 'Aluminum')
    z_f = -pt - 8.0
    mo = mp.new_occ(st.component, 'Servo motor J%d' % joint)
    fl = mp.box(x0 - F / 2, x0 + F / 2, -F / 2, F / 2, z_f, -pt)
    mp.cut(fl, along_z(x0, 0, z_f - 1, -pt + 1, g['U'] / 2 + 0.5))
    for cx, cy in corners:
        mp.cut(fl, along_z(cx, cy, z_f - 1, -pt + 1, mb / 2 + 0.3))
    mp.add_body(mo, fl, 'Flange', 'Aluminum')
    zb = z_f - m['L'] * 0.7
    mp.add_body(mo, along_z(x0, 0, zb, z_f, m['D'] / 2), 'Motor', 'Aluminum')
    mp.add_body(mo, along_z(x0, 0, z_f - m['L'], zb, m['D'] / 2), 'Brake', 'Aluminum')
    for k, (bx, by) in enumerate(u_bolts(x0, u)):
        part(st, 'Case bolt M%d %d' % (bs, k + 1), mp.cap_screw(bs, g['E'] + 0.8 * pt, (bx, by, g['E']), (0, 0, -1)))
    for k, (cx, cy) in enumerate(corners):
        part(st, 'Motor bolt M%d %d' % (mb, k + 1), mp.cap_screw(mb, 8.0 + 0.8 * pt, (cx, cy, z_f), (0, 0, 1)))
    # the output: the flange with its hub inside the rollers, and the flexspline's cup bolted under it
    of = along_z(x0, 0, g['E'], g['B'], g['O'] / 2, g['S'] / 2)
    mp.union(of, along_z(x0, 0, g['z_h0'], g['E'] + 0.01, g['r_h'], g['S'] / 2))
    part(out, 'Output flange %d' % size, of)
    cup = along_z(x0, 0, 2.0, g['z_h0'] - 2.0 + 0.01, g['r_fo'], g['r_fi'])
    mp.union(cup, along_z(x0, 0, g['z_h0'] - 2.0, g['z_h0'], g['r_fo'], g['S'] / 2))
    part(out, 'Flexspline %d' % size, cup)
    # the input: the elliptical wave generator on the motor's shaft
    a = g['r_fi'] - 0.4
    ellipse_part(inp, 'Wave generator %d' % size, x0, a, a * 0.985, g['z_wg'][0], g['z_wg'][1], g['U'] / 2)
    part(inp, 'Motor shaft J%d' % joint, along_z(x0, 0, z_f, g['z_wg'][1] - 1.0, g['U'] / 2))
    return mod, st, out, inp, g


def u_bolts(x0, u):
    return [(x0 + u['b'] / 2 * math.cos(2 * math.pi * (k + 0.5) / u['nc']),
             u['b'] / 2 * math.sin(2 * math.pi * (k + 0.5) / u['nc'])) for k in range(u['nc'])]


# ---------- joints ----------
def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_z(root, moving, fixed, point, name):
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.ZAxisJointDirection, Jd.XAxisJointDirection, Jd.YAxisJointDirection):
        sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
        sk.isVisible = False
        pt = sk.sketchPoints.add(P3(*[v * CM for v in point])).createForAssemblyContext(moving)
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


_last = {}


def build(size1=40):
    o = design(size1)
    refuse(o)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design_ = adsk.fusion.Design.cast(app.activeProduct)
    design_.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design_.rootComponent
    joints = {}
    for joint, size, x0 in ((1, int(size1), 0.0), (2, 20, X_J2)):
        mod, st, out, inp, g = module(root, 'J%d module' % joint, size, joint, x0)
        mod.isGrounded = True
        px = {x.component.name: x for x in mod.childOccurrences}
        P = lambda k: px['J%d %s' % (joint, k)]
        jo, so = revolute_z(root, P('output'), P('stator'), (x0, 0, 0), 'J%d output turns' % joint)
        ji, si = revolute_z(root, P('input'), P('stator'), (x0, 0, 0), 'J%d input turns' % joint)
        li = root.motionLinks.createInput(ji, jo)
        li.valueOne = VS('%d deg' % (360 * RATIO))
        li.valueTwo = VS('360 deg')
        li.isReversed = (so * si) > 0                              # the output turns the other way (CS fixed)
        root.motionLinks.add(li).name = 'J%d: 50 turns in, one out' % joint
        joints[joint] = (jo, so, ji, si)
        for oc, col in ((P('stator'), 'Paint - Enamel Glossy (Grey)'), (P('output'), 'Paint - Enamel Glossy (Blue)'),
                        (P('input'), 'Paint - Enamel Glossy (Yellow)')):
            mp.colour(design_, oc, col)
    _last['joints'] = joints
    app.activeViewport.fit()
    s1, s2 = o['sel1'], o['j2'][20]
    pl = parts_list(root)
    return ('SCARA joint modules built.\nJ1: CSG-%d-50-2UH, average torque %.1f N m (limit %d), life %.0f h, input up '
            'to %.0f rev/min; motor %.2f N m peak, %.2f RMS.\nJ2: CSG-20-50-2UH, average torque %.1f N m (limit %d), '
            'life %.0f h on radial moves, input up to %.0f rev/min.\n\nParts: %d components; %s.\n\n[%s]'
            % (o['size1'], s1['Tav'], sd.CSG[o['size1']]['Tav'], s1['Lh'], s1['ni_max'], o['j1_motor']['peak'],
               o['j1_motor']['rms'], s2['Tav'], sd.CSG[20]['Tav'], s2['Lh'], s2['ni_max'], sum(pl.values()),
               ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('size1', 'J1 unit size (CSG)', 40.0)]


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
        cid = 'ScaraJointModulesCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'SCARA joint modules', 'The J1 and J2 modules of Chapter 36')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ScaraJointModules could not start:\n' + traceback.format_exc())
