# =============================================================================
# SealedCover.py  -  Chapter 10 of "Mechanical Design in Autodesk Fusion with Claude": a pressure housing closed by a
# bolted cover and sealed by an O-ring in a face groove. The O-ring is drawn at its free size, so Fusion's interference
# check measures its squeeze. Written from PROMPT.txt beside it; checked by checks/sealed_cover_check.py.
#
#   INPUTS        the dialog: W, d1, L, G, p, N
#   DERIVED       seal(): c, squeeze, D_o, D_i, gland fill, R_c, the squeeze volume V, F_p, F_i, n_c
#   HARD RULES    build() refuses a squeeze outside 20-30 %, a gland fill outside 60-85 %, a clamp margin below 2
#   CONVENTIONS   a new design; every part created where it belongs; the O-ring a revolved torus; hexagon heads as
#                 closed polylines; cm inside the API; As-built rigid joints
#   ACCEPTANCE    the message with the defaults; one overlap, O-ring with cover, of V; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'SealedCover 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
SECTIONS = (1.78, 2.62, 3.53)              # O-ring cross-sections, mm
FI_M5 = 0.75 * 14.2 * 580.0                # preload of an M5 8.8 bolt, N (Chapter 8)
H_HOUSING, T_COVER, CAVITY = 20.0, 8.0, 10.0
BOLT_D, HOLE_D, HEX_S, HEX_K, ENGAGE = 5.0, 5.5, 8.0, 3.5, 10.0


def seal(W, d1, L, G, p, N):
    c = W - L
    Do = d1 + 2 * W
    Di = Do - 2 * G
    fill = (math.pi * W ** 2 / 4) / (G * L)
    r, Rc = W / 2, d1 / 2 + W / 2
    A = r ** 2 * math.acos((r - c) / r) - (r - c) * math.sqrt(2 * r * c - c ** 2) if 0 < c < W else 0.0
    Fp = p * math.pi * Do ** 2 / 4
    return dict(c=c, squeeze=100 * c / W, Do=Do, Di=Di, fill=100 * fill, Rc=Rc, zc=-L + W / 2, A=A,
                V=A * 2 * math.pi * Rc, Fp=Fp, Fi=FI_M5, nc=N * FI_M5 / Fp)


def place(root, name, x=0.0, y=0.0):
    """A new component created where it belongs: the position goes in the creation matrix."""
    mat = adsk.core.Matrix3D.create()
    mat.translation = V3(x * CM, y * CM, 0)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def extrude(comp, profiles, length_mm, op=NEW, body=None, down=False):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profiles, op)
    side = adsk.fusion.ExtentDirections.NegativeExtentDirection if down else adsk.fusion.ExtentDirections.PositiveExtentDirection
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(length_mm * CM)), side)
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def all_profiles(sk):
    col = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        col.add(p)
    return col


def hexagon(sk, s):
    """A hexagon of across-flats s centred on the sketch origin: one closed polyline with shared points."""
    r = s / math.sqrt(3.0)
    pts = [P3(r * math.cos(math.radians(60 * i)) * CM, r * math.sin(math.radians(60 * i)) * CM, 0) for i in range(6)]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for p in pts[2:]:
        prev = lines.addByTwoPoints(prev, p).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def rigid(root, moving, fixed, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    inp.setAsRigidJointMotion()
    root.asBuiltJoints.add(inp).name = name


def colour(design, occ, name):
    """An appearance from Fusion's library; the build does not depend on it."""
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


def build(W=2.62, d1=45.0, L=1.95, G=3.5, p=1.0, N=4):
    N = int(round(N))
    if min(abs(W - w) for w in SECTIONS) > 1e-6:
        raise ValueError('Cross-section %.2f mm: choose one of %s mm.' % (W, ', '.join('%.2f' % w for w in SECTIONS)))
    s = seal(W, d1, L, G, p, N)
    if not 20.0 <= s['squeeze'] <= 30.0:
        raise ValueError('Squeeze %.2f percent: a face seal needs 20 to 30 percent (Parker, Design Chart 4-3).' % s['squeeze'])
    if not 60.0 <= s['fill'] <= 85.0:
        raise ValueError('Gland fill %.2f percent: keep it between 60 and 85 percent (Parker, 3.7).' % s['fill'])
    if s['nc'] < 2.0:
        raise ValueError('The bolts clamp only n_c = N F_i / F_p = %.2f times the pressure force: add bolts.' % s['nc'])
    BCD = math.ceil(s['Do'] + 12.0 - 1e-9)
    Dc = BCD + 16.0
    R = BCD / 2.0
    holes = [(R * math.cos(2 * math.pi * i / N), R * math.sin(2 * math.pi * i / N)) for i in range(N)]

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    housing = place(root, 'Housing')
    c = housing.component
    sk = c.sketches.add(plane_at(c, -H_HOUSING))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), Dc / 2 * CM)
    body = extrude(c, sk.profiles.item(0), H_HOUSING).bodies.item(0)
    body.name = 'Housing'
    sk = c.sketches.add(c.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (s['Di'] - 6.0) / 2 * CM)
    extrude(c, sk.profiles.item(0), CAVITY, CUT, body, down=True)                      # the pressure cavity
    sk = c.sketches.add(c.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), s['Do'] / 2 * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), s['Di'] / 2 * CM)
    ring = [q for q in sk.profiles if q.profileLoops.count == 2][0]
    extrude(c, ring, L, CUT, body, down=True)                                          # the face groove
    sk = c.sketches.add(c.xYConstructionPlane)
    for x, y in holes:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), BOLT_D / 2 * CM)
    extrude(c, all_profiles(sk), ENGAGE, CUT, body, down=True)                         # the bolts' holes
    housing.isGrounded = True

    cover = place(root, 'Cover')
    c = cover.component
    sk = c.sketches.add(c.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), Dc / 2 * CM)
    for x, y in holes:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), HOLE_D / 2 * CM)
    prof = max(list(sk.profiles), key=lambda q: q.areaProperties().area)
    extrude(c, prof, T_COVER).bodies.item(0).name = 'Cover'

    oring = place(root, 'O-ring')
    c = oring.component
    sk = c.sketches.add(c.xZConstructionPlane)
    centre = sk.modelToSketchSpace(P3(s['Rc'] * CM, 0, s['zc'] * CM))                  # resting on the groove bottom
    sk.sketchCurves.sketchCircles.addByCenterRadius(centre, W / 2 * CM)
    rev = c.features.revolveFeatures
    inp = rev.createInput(sk.profiles.item(0), c.zConstructionAxis, NEW)
    inp.setAngleExtent(False, VI(2 * math.pi))
    rev.add(inp).bodies.item(0).name = 'O-ring'

    for i, (x, y) in enumerate(holes):
        bolt = place(root, 'Bolt %d' % (i + 1), x, y)
        bc = bolt.component
        sk = bc.sketches.add(plane_at(bc, T_COVER))
        hexagon(sk, HEX_S)
        b = extrude(bc, sk.profiles.item(0), HEX_K).bodies.item(0)
        b.name = 'Bolt %d' % (i + 1)
        sk = bc.sketches.add(plane_at(bc, -ENGAGE))
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), BOLT_D / 2 * CM)
        extrude(bc, sk.profiles.item(0), ENGAGE + T_COVER, JOIN, b)
        rigid(root, bolt, housing, 'Bolt %d' % (i + 1))
    rigid(root, cover, housing, 'Cover on housing')
    rigid(root, oring, housing, 'O-ring in groove')
    colour(design, housing, 'Aluminum - Satin')
    colour(design, cover, 'Paint - Enamel Glossy (Blue)')
    colour(design, oring, 'Paint - Enamel Glossy (Black)')
    app.activeViewport.fit()

    return ('Sealed cover built: O-ring %.2f x %.2f mm in a face groove %.2f deep and %.2f wide; %d M5 bolts on a %d mm '
            'circle.\n\n'
            'Squeeze c = W - L = %.2f mm = %.2f percent\n'
            'Groove D_o = %.2f mm, D_i = %.2f mm; gland fill %.2f percent\n'
            'Squeeze volume V = %.3f mm^3 (Interference should find it between the O-ring and the cover)\n'
            'Pressure force F_p = %.1f N at %.2f MPa; clamp margin n_c = N F_i / F_p = %.2f\n\n[%s]'
            % (d1, W, L, G, N, BCD, s['c'], s['squeeze'], s['Do'], s['Di'], s['fill'], s['V'], s['Fp'], p, s['nc'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            prm = dict(W=float(ins.itemById('W').selectedItem.name), d1=ins.itemById('d1').value / CM,
                       L=ins.itemById('L').value / CM, G=ins.itemById('G').value / CM, p=ins.itemById('p').value,
                       N=ins.itemById('N').value)
            ui.messageBox(build(**prm), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('W', 'O-ring cross-section (mm)', adsk.core.DropDownStyles.TextListDropDownStyle)
        for w in SECTIONS:
            dd.listItems.add('%.2f' % w, abs(w - 2.62) < 1e-9)
        ins.addValueInput('d1', 'O-ring inside diameter', 'mm', VI(45.0 * CM))
        ins.addValueInput('L', 'Groove depth', 'mm', VI(1.95 * CM))
        ins.addValueInput('G', 'Groove width', 'mm', VI(3.5 * CM))
        ins.addValueInput('p', 'Pressure in the housing (MPa)', '', VI(1.0))
        ins.addIntegerSpinnerCommandInput('N', 'Number of cover bolts', 3, 16, 1, 4)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'SealedCoverCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Sealed cover', 'A cover sealed by an O-ring in a face groove')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Sealed cover could not start:\n' + traceback.format_exc())
