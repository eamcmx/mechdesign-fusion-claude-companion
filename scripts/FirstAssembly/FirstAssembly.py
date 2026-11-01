# =============================================================================
# FirstAssembly.py  -  Chapter 2 of "Mechanical Design in Autodesk Fusion with Claude": a block with a shaft that
# turns in a hole through it. Written from PROMPT.txt beside it; checked by checks/first_assembly_check.py.
#
#   INPUTS        FIELDS (the dialog)
#   DERIVED       build(): hole diameter, the shaft's z range
#   HARD RULES    build() refuses a wall under 3 mm and a shaft shorter than the block
#   CONVENTIONS   a new design; two components, one body each; the shaft created at (x, y, 0); cm inside the API;
#                 an As-built revolute joint about z
#   ACCEPTANCE    the message; shaft at (15, 0); volumes 3927.0 and 46365.7 mm^3; no interference; x = 22 refused
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'FirstAssembly 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
CLEAR = 0.2                                # mm, the hole is this much larger than the shaft
WALL = 3.0                                 # mm, the least block material round the hole
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

FIELDS = [('L_b', 'Block length', 60.0), ('W_b', 'Block width', 40.0), ('H_b', 'Block height', 20.0),
          ('d', 'Shaft diameter', 10.0), ('L_s', 'Shaft length', 50.0), ('x', 'Shaft position x', 15.0),
          ('y', 'Shaft position y', 0.0)]


def place(root, name, x=0.0, y=0.0, z=0.0):
    """A new component created where it belongs: the position goes in the creation matrix."""
    mat = adsk.core.Matrix3D.create()
    mat.translation = V3(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    """The component's XY plane, or a plane parallel to it at height z_mm."""
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def extrude(comp, profile, length_mm, op=adsk.fusion.FeatureOperations.NewBodyFeatureOperation, body=None):
    """Extrude a profile up (+z) by length_mm; op says whether it makes, joins or cuts a body."""
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def build(L_b=60.0, W_b=40.0, H_b=20.0, d=10.0, L_s=50.0, x=15.0, y=0.0):
    # HARD CONSTRAINTS, checked before anything is drawn
    r_hole = (d + CLEAR) / 2.0
    wall = min(L_b / 2 - abs(x) - r_hole, W_b / 2 - abs(y) - r_hole)
    if wall < WALL:
        raise ValueError('Only %.1f mm of block left round the hole; at least %.0f mm is needed.' % (wall, WALL))
    if L_s <= H_b:
        raise ValueError('The shaft (%.1f mm) must be longer than the block is high (%.1f mm).' % (L_s, H_b))

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    # the block: a rectangle on the plane z = -H_b, extruded up to z = 0, then the hole cut through it
    block = place(root, 'Block')
    bc = block.component
    sk = bc.sketches.add(plane_at(bc, -H_b))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-L_b / 2 * CM, -W_b / 2 * CM, 0), P3(L_b / 2 * CM, W_b / 2 * CM, 0))
    body = extrude(bc, sk.profiles.item(0), H_b).bodies.item(0)
    body.name = 'Block'
    hs = bc.sketches.add(plane_at(bc, -H_b))
    hs.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), r_hole * CM)
    extrude(bc, hs.profiles.item(0), H_b, adsk.fusion.FeatureOperations.CutFeatureOperation, body)
    block.isGrounded = True

    # the shaft: created at (x, y, 0), drawn at its own origin, from z = -H_b up to L_s - H_b
    shaft = place(root, 'Shaft', x, y, 0.0)
    sc = shaft.component
    ss = sc.sketches.add(plane_at(sc, -H_b))
    ss.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d / 2 * CM)
    sbody = extrude(sc, ss.profiles.item(0), L_s).bodies.item(0)
    sbody.name = 'Shaft'

    # the joint: As-built, revolute about z through the shaft's centre (a point at its own origin)
    jp = sc.sketches.add(sc.xYConstructionPlane)
    jp.isVisible = False
    pt = jp.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(shaft)
    ji = root.asBuiltJoints.createInput(shaft, block, adsk.fusion.JointGeometry.createByPoint(pt))
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    root.asBuiltJoints.add(ji).name = 'Shaft turns'
    app.activeViewport.fit()

    t = shaft.transform2.translation
    return ('Block and shaft built.\n\n'
            'Shaft at (%.2f, %.2f) mm, diameter %.1f mm, %.1f mm long\n'
            'Shaft volume  %.1f mm^3\nBlock volume  %.1f mm^3\n\n'
            'Drive Joints on "Shaft turns" to turn it.\n\n[%s]'
            % (t.x / CM, t.y / CM, d, L_s, sbody.volume / CM ** 3, body.volume / CM ** 3, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            p = {key: inputs.itemById(key).value / CM for key, _, _ in FIELDS}
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        inputs = cmd.commandInputs
        for key, label, d in FIELDS:
            inputs.addValueInput(key, label, 'mm', VI(d * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'FirstAssemblyCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Block and shaft', 'A block with a shaft that turns in it')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Block and shaft could not start:\n' + traceback.format_exc())
