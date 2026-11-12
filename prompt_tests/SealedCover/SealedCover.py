# =============================================================================
# SealedCover.py  -  a pressure housing closed by a bolted cover and sealed by an O-ring in a face groove.
#
# The O-ring is drawn at its free size, resting on the groove bottom against the groove's outside wall (the
# pressure pushes it outward), so the part of it that stands above the housing's top face lies inside the cover:
# Fusion's interference check then measures the squeeze volume. The message reports the squeeze, the gland fill,
# the pressure force on the cover and the clamp margin of the cover bolts.
#
#   Housing   (grounded) cylinder z = -20 .. 0, pressure cavity D_i - 6 x 10 deep, face groove D_i .. D_o x L deep,
#             N blind holes 5 mm x 10 deep on the bolt circle
#   Cover     z = 0 .. 8, N clearance holes 5.5 mm
#   O-ring    a torus revolved from its circular section about z
#   Bolt i    M5: hexagon head 8 across flats x 3.5 on the cover, 5 mm shank down to z = -10,
#             at 360 (i - 1) / N degrees on the bolt circle
#   Joints    As-built, rigid: Cover, O-ring and each Bolt to Housing
#
# All sizes in the formulas are millimetres, MPa and newtons; the Fusion API works in centimetres (CM = 0.1).
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _common_dir():
    """Walk up from this script's folder to the first folder that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % __file__)
        d = up


_COMMON = _common_dir()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402  (puts every lib/ folder of the companion on sys.path)

VERSION = 'SealedCover v1.0 (prompt test, 2026-11-11)'
CMD_ID = 'mdfcSealedCoverPromptTest'

CM = 0.1                                   # mm -> cm
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

W_CHOICES = (1.78, 2.62, 3.53)             # O-ring cross-sections offered, mm
DEFAULTS = dict(W=2.62, d1=45.0, L=1.95, G=3.5, p=1.0, N=4)

# cover bolts: M5, property class 8.8 (ISO 898-1 proof strength 580 MPa), stress area 14.2 mm^2
PROOF_MPA = 580.0
AS_M5 = 14.2
F_I = 0.75 * AS_M5 * PROOF_MPA             # preload of one bolt, N (6177 N)
D_SHANK = 5.0                              # shank and blind hole, mm
D_CLEAR = 5.5                              # clearance hole in the cover, mm
HEX_AF = 8.0                               # head across flats, mm
HEX_H = 3.5                                # head height, mm

# the blocks, mm
Z_HOUSING = -20.0                          # housing from here up to z = 0
CAVITY_DEPTH = 10.0
HOLE_DEPTH = 10.0                          # blind holes, and the shank ends at z = -10
COVER_T = 8.0                              # cover from z = 0 to 8

# limits (HARD CONSTRAINTS of the prompt)
SQUEEZE_MIN, SQUEEZE_MAX = 20.0, 30.0      # percent, Parker O-Ring Handbook, Design Chart 4-3 (face seals)
FILL_MIN, FILL_MAX = 60.0, 85.0            # percent, Parker section 3.7
NC_MIN = 2.0


# ---------- the seal, by the formulas of the prompt ----------
def seal(W, d1, L, G, p, N):
    """Every derived number of the seal, from the inputs (mm, MPa, count)."""
    c = W - L                                          # squeeze, mm
    squeeze = 100.0 * c / W
    Do = d1 + 2.0 * W                                  # groove outside diameter = the O-ring's outside diameter
    Di = Do - 2.0 * G                                  # groove inside diameter
    fill = 100.0 * (math.pi * W * W / 4.0) / (G * L)   # gland fill, percent
    r = W / 2.0
    Rc = d1 / 2.0 + W / 2.0                            # section centre radius
    zc = -L + W / 2.0                                  # section centre height (resting on the groove bottom)
    if c <= 0.0:
        A = 0.0                                        # nothing stands above the groove
    elif c >= W:
        A = math.pi * r * r
    else:
        A = r * r * math.acos((r - c) / r) - (r - c) * math.sqrt(2.0 * r * c - c * c)
    V = A * 2.0 * math.pi * Rc                         # squeeze volume, mm^3 (Pappus: the segment's centroid is at R_c)
    Fp = p * math.pi * Do ** 2 / 4.0                   # pressure force on the cover, N
    nc = N * F_I / Fp if Fp > 0 else float('inf')      # clamp margin
    Db = math.ceil(round(Do + 12.0, 9))                # bolt circle, rounded up to a whole mm
    Dh = Db + 16                                       # housing and cover diameter
    return dict(W=W, d1=d1, L=L, G=G, p=p, N=N, c=c, squeeze=squeeze, Do=Do, Di=Di, fill=fill, r=r, Rc=Rc, zc=zc,
                A=A, V=V, Fp=Fp, Fi=F_I, nc=nc, Db=Db, Dh=Dh, Dcav=Di - 6.0)


def _refuse_if_out(s):
    """Raise ValueError, before anything is built, for inputs the prompt does not accept."""
    W, d1, L, G, p, N = s['W'], s['d1'], s['L'], s['G'], s['p'], s['N']
    if not any(abs(W - w) < 1e-6 for w in W_CHOICES):
        raise ValueError('O-ring cross-section W = %g mm is not one of 1.78, 2.62, 3.53 mm.' % W)
    if d1 <= 0 or L <= 0 or G <= 0 or p <= 0:
        raise ValueError('d1, L, G and p must all be positive (d1 %g mm, L %g mm, G %g mm, p %g MPa).' % (d1, L, G, p))
    if N < 1 or abs(N - round(N)) > 1e-9:
        raise ValueError('The number of cover bolts N must be a whole number of at least 1 (N = %g).' % N)
    if not SQUEEZE_MIN <= s['squeeze'] <= SQUEEZE_MAX:
        raise ValueError('Squeeze %.2f percent (c = W - L = %.2f mm) is outside 20 to 30 percent '
                         '(Parker O-Ring Handbook, Design Chart 4-3, face seals).' % (s['squeeze'], s['c']))
    if not FILL_MIN <= s['fill'] <= FILL_MAX:
        raise ValueError('Gland fill %.2f percent is outside 60 to 85 percent (Parker, section 3.7): '
                         'section pi W^2 / 4 = %.3f mm^2 in a groove G L = %.3f mm^2.'
                         % (s['fill'], math.pi * W * W / 4.0, G * L))
    if s['nc'] < NC_MIN:
        raise ValueError('Clamp margin n_c = N F_i / F_p = %.2f is below 2: %d M5 8.8 bolts at F_i = %.0f N against '
                         'F_p = %.1f N.' % (s['nc'], int(round(N)), F_I, s['Fp']))
    # no part may touch the O-ring except the groove and the cover
    if s['Di'] / 2.0 >= d1 / 2.0:
        raise ValueError('The O-ring (inside diameter %.2f mm) would touch the groove\'s inside wall (D_i = %.2f mm).'
                         % (d1, s['Di']))
    if s['Dcav'] <= 0:
        raise ValueError('The pressure cavity D_i - 6 = %.2f mm has no size.' % s['Dcav'])
    if s['Db'] / 2.0 - D_CLEAR / 2.0 <= s['Do'] / 2.0:
        raise ValueError('The bolt holes would cut into the groove.')
    n = int(round(N))
    corner = HEX_AF / math.cos(math.radians(30.0))          # the head across corners
    if n >= 2 and s['Db'] * math.sin(math.pi / n) <= corner:
        raise ValueError('%d bolt heads (%.2f mm across corners) do not fit on the %d mm bolt circle without touching.'
                         % (n, corner, s['Db']))


# ---------- Fusion helpers ----------
def _new_occ(root, name, t=(0.0, 0.0, 0.0)):
    """A new component in the root, its origin at the translation t (mm)."""
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(t[0] * CM, t[1] * CM, t[2] * CM)
    occ = root.occurrences.addNewComponent(m)
    occ.component.name = name
    return occ


def _extrude(comp, sk, profiles, depth_mm, up, op, bodies=None):
    """Extrude `profiles` of the sketch `sk` (square to z) by depth_mm, towards +z if up, else towards -z."""
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profiles, op)
    normal = sk.xDirection.crossProduct(sk.yDirection)
    along = (normal.z > 0) == up
    d = (adsk.fusion.ExtentDirections.PositiveExtentDirection if along
         else adsk.fusion.ExtentDirections.NegativeExtentDirection)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(depth_mm * CM)), d)
    if bodies is not None:
        inp.participantBodies = bodies
    return ext.add(inp)


def _profile_most_loops(sk):
    best = None
    for pr in sk.profiles:
        if best is None or pr.profileLoops.count > best.profileLoops.count:
            best = pr
    return best


def _bolt_xy(s, i):
    """Bolt i (1 .. N) on the bolt circle at 360 (i - 1) / N degrees, mm."""
    a = 2.0 * math.pi * (i - 1) / int(round(s['N']))
    R = s['Db'] / 2.0
    return R * math.cos(a), R * math.sin(a)


def _housing(root, s):
    occ = _new_occ(root, 'Housing')
    comp = occ.component
    cut = adsk.fusion.FeatureOperations.CutFeatureOperation
    # the cylinder, z = -20 .. 0
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Housing outline'
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), s['Dh'] / 2.0 * CM)
    f = _extrude(comp, sk, sk.profiles.item(0), -Z_HOUSING, False,
                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = f.bodies.item(0)
    body.name = 'Housing'
    # the pressure cavity, D_i - 6 x 10 deep
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Pressure cavity'
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), s['Dcav'] / 2.0 * CM)
    _extrude(comp, sk, sk.profiles.item(0), CAVITY_DEPTH, False, cut, [body])
    # the face groove, D_i .. D_o, L deep, in the top face
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Face groove'
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), s['Di'] / 2.0 * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), s['Do'] / 2.0 * CM)
    ring = [pr for pr in sk.profiles if pr.profileLoops.count == 2][0]
    _extrude(comp, sk, ring, s['L'], False, cut, [body])
    # N blind holes, 5 mm x 10 deep, on the bolt circle
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Bolt holes'
    n = int(round(s['N']))
    for i in range(1, n + 1):
        x, y = _bolt_xy(s, i)
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), D_SHANK / 2.0 * CM)
    holes = adsk.core.ObjectCollection.create()
    for pr in sk.profiles:
        holes.add(pr)
    _extrude(comp, sk, holes, HOLE_DEPTH, False, cut, [body])
    occ.isGrounded = True
    return occ


def _cover(root, s):
    occ = _new_occ(root, 'Cover')
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Cover outline'
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), s['Dh'] / 2.0 * CM)
    n = int(round(s['N']))
    for i in range(1, n + 1):
        x, y = _bolt_xy(s, i)
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), D_CLEAR / 2.0 * CM)
    f = _extrude(comp, sk, _profile_most_loops(sk), COVER_T, True,
                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    f.bodies.item(0).name = 'Cover'
    return occ


def _oring(root, s):
    occ = _new_occ(root, 'O-ring')
    comp = occ.component
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = 'O-ring section'
    centre = sk.modelToSketchSpace(P3(s['Rc'] * CM, 0, s['zc'] * CM))
    sk.sketchCurves.sketchCircles.addByCenterRadius(centre, s['r'] * CM)
    revs = comp.features.revolveFeatures
    inp = revs.createInput(sk.profiles.item(0), comp.zConstructionAxis,
                           adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setAngleExtent(False, VS('360 deg'))
    f = revs.add(inp)
    f.bodies.item(0).name = 'O-ring'
    return occ


def _bolt(root, s, i):
    """Bolt i: its own frame at the underside of the head (on the cover, z = 8), on the bolt circle."""
    x, y = _bolt_xy(s, i)
    occ = _new_occ(root, 'Bolt %d' % i, (x, y, COVER_T))
    comp = occ.component
    # the shank, 5 mm, from the head down to z = -10
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Shank'
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), D_SHANK / 2.0 * CM)
    f = _extrude(comp, sk, sk.profiles.item(0), COVER_T + HOLE_DEPTH, False,
                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = f.bodies.item(0)
    # the hexagon head, 8 across flats, 3.5 high, joined to the shank
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Hexagon head'
    rc = HEX_AF / 2.0 / math.cos(math.radians(30.0))
    pts = [P3(rc * math.cos(math.radians(30.0 + 60.0 * k)) * CM, rc * math.sin(math.radians(30.0 + 60.0 * k)) * CM, 0)
           for k in range(6)]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first
    for k in range(2, 6):
        prev = lines.addByTwoPoints(prev.endSketchPoint, pts[k])
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
    _extrude(comp, sk, sk.profiles.item(0), HEX_H, True, adsk.fusion.FeatureOperations.JoinFeatureOperation, [body])
    comp.bRepBodies.item(0).name = 'Bolt %d' % i
    return occ


def _rigid(root, occ, housing):
    ji = root.asBuiltJoints.createInput(occ, housing, None)
    ji.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(ji)
    j.name = '%s to Housing' % occ.component.name
    return j


def _report(s):
    return ('%s\n\n'
            'O-ring W = %.2f mm, d1 = %g mm; groove L = %g mm deep, G = %g mm wide; p = %g MPa; %d bolts M5 8.8\n\n'
            'squeeze c = W - L = %.2f mm, %.2f percent of W (20 to 30)\n'
            'groove D_o = %.2f mm, D_i = %.2f mm\n'
            'gland fill %.2f percent (60 to 85)\n'
            'squeeze volume V = %.3f mm^3 (the O-ring inside the cover, as the interference check finds it)\n'
            'pressure force F_p = %.1f N; preload F_i = %.0f N per bolt\n'
            'clamp margin n_c = N F_i / F_p = %.2f (at least 2)\n\n'
            'bolt circle %d mm, housing and cover %d mm'
            % (VERSION, s['W'], s['d1'], s['L'], s['G'], s['p'], int(round(s['N'])), s['c'], s['squeeze'], s['Do'],
               s['Di'], s['fill'], s['V'], s['Fp'], s['Fi'], s['nc'], s['Db'], s['Dh']))


# ---------- the build ----------
def build(W=DEFAULTS['W'], d1=DEFAULTS['d1'], L=DEFAULTS['L'], G=DEFAULTS['G'], p=DEFAULTS['p'], N=DEFAULTS['N']):
    """Check the inputs, then build the sealed cover in a new design. Returns the report; raises ValueError (and
    builds nothing) when a hard constraint is broken."""
    s = seal(float(W), float(d1), float(L), float(G), float(p), N)
    _refuse_if_out(s)
    s['N'] = int(round(N))
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    housing = _housing(root, s)
    cover = _cover(root, s)
    oring = _oring(root, s)
    bolts = [_bolt(root, s, i) for i in range(1, s['N'] + 1)]
    for o in [cover, oring] + bolts:
        _rigid(root, o, housing)
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return _report(s)


# ---------- the dialog ----------
_handlers = []
_waiting = False          # True when Fusion ran this script itself, so that it must be told when to end


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            W = float(ins.itemById('W').selectedItem.name)
            d1 = round(ins.itemById('d1').value * 10.0, 9)       # cm -> mm
            L = round(ins.itemById('L').value * 10.0, 9)
            G = round(ins.itemById('G').value * 10.0, 9)
            p = ins.itemById('p').value
            N = ins.itemById('N').value
            msg = build(W=W, d1=d1, L=L, G=G, p=p, N=N)
            ui.messageBox(msg, 'Sealed cover')
        except ValueError as e:
            ui.messageBox('Refused, nothing built:\n%s\n\n%s' % (e, VERSION), 'Sealed cover')
        except Exception:
            ui.messageBox('Sealed cover failed:\n%s' % traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        global _waiting
        if _waiting:
            _waiting = False
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            dd = ins.addDropDownCommandInput('W', 'O-ring cross-section W (mm)',
                                             adsk.core.DropDownStyles.TextListDropDownStyle)
            for w in W_CHOICES:
                dd.listItems.add('%.2f' % w, abs(w - DEFAULTS['W']) < 1e-9, '')
            ins.addValueInput('d1', 'O-ring inside diameter d1', 'mm', VS('%g mm' % DEFAULTS['d1']))
            ins.addValueInput('L', 'Groove depth L', 'mm', VS('%g mm' % DEFAULTS['L']))
            ins.addValueInput('G', 'Groove width G', 'mm', VS('%g mm' % DEFAULTS['G']))
            ins.addValueInput('p', 'Pressure in the housing p (MPa)', '', VI(DEFAULTS['p']))
            ins.addIntegerSpinnerCommandInput('N', 'Number of cover bolts N', 1, 36, 1, DEFAULTS['N'])
            on_execute = _Execute()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('Sealed cover dialog failed:\n%s' % traceback.format_exc())


def run(context):
    global _waiting
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, 'Sealed cover', 'A pressure housing, bolted cover and face O-ring drawn at its free size.')
        on_created = _Created()
        cdef.commandCreated.add(on_created)
        _handlers.append(on_created)
        if context is not None:            # run by Fusion: stay alive until the dialog closes
            _waiting = True
            adsk.autoTerminate(False)
        cdef.execute()
    except Exception:
        if ui:
            ui.messageBox('Sealed cover failed:\n%s' % traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        d = ui.commandDefinitions.itemById(CMD_ID)
        if d:
            d.deleteMe()
    except Exception:
        pass
