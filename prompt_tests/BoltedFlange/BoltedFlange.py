# =============================================================================
# BoltedFlange.py - two round flange plates clamped together by hexagon bolts and nuts on a bolt circle, assembled
# with rigid As-built Joints. The script reports the preload, the tightening torque and the safety factors of the
# joint under an external tensile load (property class 8.8, metric coarse thread, steel plates).
#
# Inputs (a command dialog): bolt size, number of bolts N, bolt circle diameter BCD, plate thicknesses t1 and t2,
# external tensile load F_ext. Every length is typed in mm; the Fusion API works in cm.
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_common():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py; return common/."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('common/companion_paths.py not found above ' + os.path.abspath(__file__))
        d = up


_common = _companion_common()
if _common not in sys.path:
    sys.path.insert(0, _common)
import companion_paths  # noqa: E402,F401  (puts every lib/ folder of the companion on sys.path)

VERSION = 'BoltedFlange v1.0 (prompt test, 2026-11-11)'

CM = 0.1                      # mm -> cm
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

# metric coarse thread data, mm: d, tensile stress area At, clearance hole d_h (ISO 273 medium),
# hexagon across flats s and head height k (ISO 4017), nut height m (ISO 4032)
THREADS = {
    'M5': dict(d=5.0, At=14.2, dh=5.5, s=8.0, k=3.5, m=4.7),
    'M6': dict(d=6.0, At=20.1, dh=6.6, s=10.0, k=4.0, m=5.2),
    'M8': dict(d=8.0, At=36.6, dh=9.0, s=13.0, k=5.3, m=6.8),
    'M10': dict(d=10.0, At=58.0, dh=11.0, s=16.0, k=6.4, m=8.4),
    'M12': dict(d=12.0, At=84.3, dh=13.5, s=18.0, k=7.5, m=10.8),
}
SIZES = ('M5', 'M6', 'M8', 'M10', 'M12')
SP = 580.0                    # proof stress of property class 8.8, MPa
E = 207000.0                  # steel, MPa
K_TORQUE = 0.2                # torque coefficient

DEFAULTS = dict(size='M8', N=4, BCD=60.0, t1=10.0, t2=10.0, F_ext=20000.0)

CMD_ID = 'mdfcBoltedFlangePromptTest'
_handlers = []
_standalone = False           # True when Fusion runs the script itself (it passes a context); see run()
_last = None


# ---------- the joint's numbers ----------
def joint(size, N, t1, t2, F_ext):
    """The bolted joint of one bolt size, N bolts, plates t1 and t2 (mm) and the external load F_ext (N).
    Forces in N, stiffnesses in N/mm, the torque T in N.m."""
    th = THREADS[size]
    d, At = th['d'], th['At']
    l = t1 + t2                                   # grip, mm
    Fp = At * SP                                  # proof load
    Fi = 0.75 * Fp                                # preload
    T = K_TORQUE * Fi * d / 1000.0                # tightening torque, N.m
    kb = At * E / l                               # bolt stiffness, fully threaded bolt
    km = 0.5774 * math.pi * E * d / (2.0 * math.log(5.0 * (0.5774 * l + 0.5 * d) / (0.5774 * l + 2.5 * d)))
    C = kb / (kb + km)                            # joint constant
    P = F_ext / N                                 # external load per bolt
    Fb = Fi + C * P                               # bolt load
    Fm = Fi - (1.0 - C) * P                       # clamping force left
    np_ = Fp / Fb                                 # yield (proof) factor
    n0 = Fi / (P * (1.0 - C)) if P > 0 else float('inf')   # factor against separation
    return dict(size=size, d=d, At=At, l=l, Fp=Fp, Fi=Fi, T=T, kb=kb, km=km, C=C, P=P, Fb=Fb, Fm=Fm,
                np=np_, n0=n0)


def design_values(size, N, BCD, t1, t2, F_ext):
    """Check the inputs against the hard constraints and return the joint and the part sizes (mm).
    Raises ValueError when a constraint is broken."""
    if size not in THREADS:
        raise ValueError('bolt size %r is not one of %s' % (size, ', '.join(SIZES)))
    if int(N) != N or N < 1:
        raise ValueError('the number of bolts must be a whole number of at least 1, not %s' % N)
    N = int(N)
    for name, v in (('BCD', BCD), ('t1', t1), ('t2', t2), ('F_ext', F_ext)):
        if not v > 0:
            raise ValueError('%s must be positive, not %s' % (name, v))
    th = THREADS[size]
    chord = BCD * math.sin(math.pi / N)
    if chord < 2.0 * th['s']:
        raise ValueError('no room for a socket between neighbouring bolts: BCD sin(pi / N) = %g sin(%g deg) = '
                         '%.1f mm < 2 s = %g mm' % (BCD, 180.0 / N, chord, 2.0 * th['s']))
    j = joint(size, N, t1, t2, F_ext)
    if not j['n0'] > 1.0:
        raise ValueError('the joint separates: n0 = Fi / (P (1 - C)) = %.3f, it must be above 1' % j['n0'])
    if not j['np'] > 1.0:
        raise ValueError('the bolts pass their proof load: np = Fp / Fb = %.3f, it must be above 1' % j['np'])
    if not th['dh'] > th['d']:
        raise ValueError('the clearance hole %g mm is not larger than the bolt %g mm' % (th['dh'], th['d']))
    g = dict(th)
    g.update(N=N, BCD=BCD, t1=t1, t2=t2, F_ext=F_ext,
             D_plate=BCD + 4.0 * th['dh'],                 # plate diameter
             L_bolt=j['l'] + th['m'] + 2.0)                # bolt length under the head
    return j, g


def report(j, g):
    return ('%s\n\n'
            'Bolted flange: %d x %s 8.8 on a %g mm bolt circle, plates %g + %g mm (diameter %g mm), '
            'bolts %g mm long, F_ext = %g N\n\n'
            'At = %.1f mm^2\n'
            'Fp = At Sp = %.0f N\n'
            'Fi = 0.75 Fp = %.0f N\n'
            'T = K Fi d = %.2f N.m\n'
            'kb = At E / l = %.0f N/mm\n'
            'km = %.0f N/mm\n'
            'C = kb / (kb + km) = %.5f\n'
            'P = F_ext / N = %.0f N\n'
            'Fb = Fi + C P = %.0f N\n'
            'Fm = Fi - (1 - C) P = %.0f N\n'
            'np = Fp / Fb = %.3f\n'
            'n0 = Fi / (P (1 - C)) = %.3f'
            % (VERSION, g['N'], j['size'], g['BCD'], g['t1'], g['t2'], g['D_plate'], g['L_bolt'], g['F_ext'],
               j['At'], j['Fp'], j['Fi'], j['T'], j['kb'], j['km'], j['C'], j['P'], j['Fb'], j['Fm'],
               j['np'], j['n0']))


# ---------- geometry helpers (mm in, cm to the API) ----------
def hexagon(sk, s, phase=0.0):
    """A regular hexagon across flats s, centred on the sketch origin: one closed polyline whose six lines share
    their sketch points."""
    R = s / math.sqrt(3.0)                       # corner radius
    pts = [P3(R * math.cos(phase + math.pi / 6.0 + i * math.pi / 3.0) * CM,
              R * math.sin(phase + math.pi / 6.0 + i * math.pi / 3.0) * CM, 0.0) for i in range(6)]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first
    for i in range(2, 6):
        prev = lines.addByTwoPoints(prev.endSketchPoint, pts[i])
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)


def profile_with_loops(sk, n_loops):
    for p in sk.profiles:
        if p.profileLoops.count == n_loops:
            return p
    raise RuntimeError('no profile with %d loops in sketch %s' % (n_loops, sk.name))


def extrude(comp, prof, dist_mm, op, bodies=None):
    """Extrude a profile dist_mm along +z of its sketch (negative: along -z)."""
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    direction = (adsk.fusion.ExtentDirections.PositiveExtentDirection if dist_mm > 0
                 else adsk.fusion.ExtentDirections.NegativeExtentDirection)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(abs(dist_mm) * CM)), direction)
    if bodies:
        inp.participantBodies = bodies
    return ext.add(inp)


def new_occ(root, name, x=0.0, y=0.0, z=0.0, angle=0.0):
    """A new component at (x, y, z) mm, turned by `angle` (rad) about z."""
    mat = adsk.core.Matrix3D.create()
    if angle:
        mat.setToRotation(angle, V3(0, 0, 1), P3(0, 0, 0))
    mat.translation = V3(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def plate(root, name, g, z_dir, t):
    """A round plate with N clearance holes on the bolt circle, from z = 0 to z_dir * t."""
    occ = new_occ(root, name)
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name
    circles = sk.sketchCurves.sketchCircles
    circles.addByCenterRadius(P3(0, 0, 0), g['D_plate'] / 2.0 * CM)
    r = g['BCD'] / 2.0
    for i in range(g['N']):
        a = 2.0 * math.pi * i / g['N']
        circles.addByCenterRadius(P3(r * math.cos(a) * CM, r * math.sin(a) * CM, 0), g['dh'] / 2.0 * CM)
    f = extrude(comp, profile_with_loops(sk, g['N'] + 1), z_dir * t,
                adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    f.bodies.item(0).name = name
    return occ


def bolt(root, name, g, x, y, a):
    """A hexagon head bolt: its origin at the head's underside on top of plate 2 (z = t2), the head above, a plain
    shank of diameter d down through both plates and the nut, 2 mm past it."""
    occ = new_occ(root, name, x, y, g['t2'], a)
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name + ' head'
    hexagon(sk, g['s'])
    head = extrude(comp, profile_with_loops(sk, 1), g['k'], adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = head.bodies.item(0)
    sk2 = comp.sketches.add(comp.xYConstructionPlane)
    sk2.name = name + ' shank'
    sk2.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), g['d'] / 2.0 * CM)
    shank = extrude(comp, profile_with_loops(sk2, 1), -g['L_bolt'], adsk.fusion.FeatureOperations.JoinFeatureOperation,
                    [body])
    shank.bodies.item(0).name = name
    return occ


def nut(root, name, g, x, y, a):
    """A hexagon nut with a hole of diameter d: its origin on its top face, against the underside of plate 1
    (z = -t1), the nut below."""
    occ = new_occ(root, name, x, y, -g['t1'], a)
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name
    hexagon(sk, g['s'])
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), g['d'] / 2.0 * CM)
    f = extrude(comp, profile_with_loops(sk, 2), -g['m'], adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    f.bodies.item(0).name = name
    return occ


def rigid(root, occ, to, name):
    inp = root.asBuiltJoints.createInput(occ, to, None)
    inp.setAsRigidJointMotion()
    jt = root.asBuiltJoints.add(inp)
    jt.name = name
    return jt


# ---------- the build ----------
def build(size=DEFAULTS['size'], N=DEFAULTS['N'], BCD=DEFAULTS['BCD'], t1=DEFAULTS['t1'], t2=DEFAULTS['t2'],
          F_ext=DEFAULTS['F_ext']):
    """Check the constraints, then build the flange in a new Fusion design; returns the report text.
    Raises ValueError (before any document is made) when the joint is refused."""
    global _last
    j, g = design_values(size, N, BCD, t1, t2, F_ext)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    root = des.rootComponent

    p1 = plate(root, 'Plate 1', g, -1.0, g['t1'])          # z = -t1 .. 0
    p1.isGrounded = True
    p2 = plate(root, 'Plate 2', g, 1.0, g['t2'])           # z = 0 .. t2

    r = g['BCD'] / 2.0
    bolts, nuts = [], []
    for i in range(1, g['N'] + 1):
        a = 2.0 * math.pi * (i - 1) / g['N']
        x, y = r * math.cos(a), r * math.sin(a)
        bolts.append(bolt(root, 'Bolt %d' % i, g, x, y, a))
        nuts.append(nut(root, 'Nut %d' % i, g, x, y, a))

    rigid(root, p2, p1, 'Plate 2 to Plate 1')
    for i, o in enumerate(bolts, 1):
        rigid(root, o, p1, 'Bolt %d to Plate 1' % i)
    for i, o in enumerate(nuts, 1):
        rigid(root, o, p1, 'Nut %d to Plate 1' % i)

    app.activeViewport.fit()
    text = report(j, g)
    _last = dict(joint=j, geometry=g, text=text)
    return text


# ---------- the dialog ----------
class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            cmd = args.command
            cmd.isRepeatable = False
            ins = cmd.commandInputs
            dd = ins.addDropDownCommandInput('size', 'Bolt size', adsk.core.DropDownStyles.TextListDropDownStyle)
            for s in SIZES:
                dd.listItems.add(s, s == DEFAULTS['size'])
            ins.addIntegerSpinnerCommandInput('N', 'Number of bolts N', 1, 72, 1, DEFAULTS['N'])
            ins.addValueInput('BCD', 'Bolt circle diameter BCD', 'mm', VI(DEFAULTS['BCD'] * CM))
            ins.addValueInput('t1', 'Plate 1 thickness t1', 'mm', VI(DEFAULTS['t1'] * CM))
            ins.addValueInput('t2', 'Plate 2 thickness t2', 'mm', VI(DEFAULTS['t2'] * CM))
            ins.addValueInput('F_ext', 'External tensile load F_ext (N)', '', VI(DEFAULTS['F_ext']))
            ex = _Execute()
            cmd.execute.add(ex)
            _handlers.append(ex)
            de = _Destroy()
            cmd.destroy.add(de)
            _handlers.append(de)
        except Exception:
            ui.messageBox('BoltedFlange: the dialog failed\n' + traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            ins = args.command.commandInputs
            size = ins.itemById('size').selectedItem.name
            N = ins.itemById('N').value
            BCD = ins.itemById('BCD').value / CM                 # cm -> mm
            t1 = ins.itemById('t1').value / CM
            t2 = ins.itemById('t2').value / CM
            F_ext = ins.itemById('F_ext').value
            try:
                text = build(size=size, N=N, BCD=BCD, t1=t1, t2=t2, F_ext=F_ext)
            except ValueError as e:
                ui.messageBox('%s\n\nRefused: %s' % (VERSION, e), 'Bolted flange')
                return
            ui.messageBox(text, 'Bolted flange')
        except Exception:
            ui.messageBox('BoltedFlange failed\n' + traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _standalone:
            adsk.terminate()


def run(context):
    global _standalone
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        # Fusion passes a context when it runs the script itself; then the script must stay loaded while its dialog
        # is open and end when the dialog closes.
        _standalone = context is not None
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Bolted flange',
                                                        'Two flange plates bolted together on a bolt circle')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if _standalone:
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('BoltedFlange failed\n' + traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
