# =============================================================================
# BearingSeat.py  -  Chapter 8 of "Mechanical Design in Autodesk Fusion with Claude": a deep groove ball bearing seated
# on a shaft and in a housing; the balls orbit at the cage's speed; the rating life is reported. Written from
# PROMPT.txt beside it; checked by checks/bearing_seat_check.py.
#
#   INPUTS        the dialog: bearing, n, Fr, L10h_req
#   DERIVED       rating(): L10, L10h, s0; geometry(): dm, Dw, Z, the ring diameters, the cage speed ratio
#   HARD RULES    build() refuses a life below the required one and balls that would touch each other
#   CONVENTIONS   a new design; five components created where they belong; cm inside the API; As-built Joints;
#                 a Motion Link from the shaft to the balls at n_cage / n
#   ACCEPTANCE    the message; 6002 and 61902 at 500 N; inner ring 90, balls 35.234 degrees; no interference; refusal
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'BearingSeat 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

# SKF data: d, D, B in mm; C, C0 in kN
BEARINGS = {'6002': (15.0, 32.0, 9.0, 5.85, 2.85), '61902': (15.0, 28.0, 7.0, 4.36, 2.24)}


def rating(bearing, n, Fr):
    d, D, B, C, C0 = BEARINGS[bearing]
    P = Fr
    L10 = (C * 1000.0 / P) ** 3                     # million revolutions
    L10h = 1e6 * L10 / (60.0 * n)                   # hours
    s0 = C0 * 1000.0 / P
    return L10, L10h, s0


def geometry(bearing):
    d, D, B, C, C0 = BEARINGS[bearing]
    dm = (d + D) / 2.0
    Dw = 0.3 * (D - d)
    Z = int(math.floor(math.pi * dm / (1.25 * Dw)))
    return dict(dm=dm, Dw=Dw, Z=Z, ring_in_od=dm - Dw - 0.1, ring_out_id=dm + Dw + 0.1, k=0.5 * (1.0 - Dw / dm))


def place(root, name):
    """Every part of this assembly sits on the z axis at the origin: create it there."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def ring(comp, z0, height, d_in, d_out, name):
    """A ring (annulus) from z0 up by height, inside and outside diameters in mm; returns its body."""
    sk = comp.sketches.add(plane_at(comp, z0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d_out / 2 * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d_in / 2 * CM)
    prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, VI(height * CM))
    body = ext.add(inp).bodies.item(0)
    body.name = name
    return body


def joint_point(occ):
    sk = occ.component.sketches.add(occ.component.xYConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(occ)


def as_built(root, moving, fixed, revolute, name):
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(joint_point(moving)))
    if revolute:
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def build(bearing='6002', n=1000.0, Fr=500.0, L10h_req=10000.0):
    if bearing not in BEARINGS:
        raise ValueError('Bearing %s is not in the table: %s.' % (bearing, ', '.join(BEARINGS)))
    d, D, B, C, C0 = BEARINGS[bearing]
    L10, L10h, s0 = rating(bearing, n, Fr)
    g = geometry(bearing)
    if L10h < L10h_req:
        P_ok = C * 1000.0 / (L10h_req * 60.0 * n / 1e6) ** (1.0 / 3.0)
        raise ValueError('Rating life L10h = %.0f h is below the required %.0f h. The %s reaches %.0f h at %.0f rpm only up to '
                         'Fr = %.0f N.' % (L10h, L10h_req, bearing, L10h_req, n, P_ok))
    if g['dm'] * math.sin(math.pi / g['Z']) <= g['Dw']:
        raise ValueError('%d balls of %.2f mm touch each other on a pitch diameter of %.2f mm.' % (g['Z'], g['Dw'], g['dm']))

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    ext = lambda comp: comp.features.extrudeFeatures

    # housing: a block with the bore D and the shoulder bore D - 4 below it
    housing = place(root, 'Housing')
    hc = housing.component
    side = D + 16.0
    sk = hc.sketches.add(plane_at(hc, -3.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-side / 2 * CM, -side / 2 * CM, 0), P3(side / 2 * CM, side / 2 * CM, 0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (D - 4.0) / 2 * CM)
    prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    inp = ext(hc).createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, VI((B + 3.0) * CM))
    hb = ext(hc).add(inp).bodies.item(0)
    hb.name = 'Housing'
    sk = hc.sketches.add(hc.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), D / 2 * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (D - 4.0) / 2 * CM)
    prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    inp = ext(hc).createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setDistanceExtent(False, VI(B * CM))
    inp.participantBodies = [hb]
    ext(hc).add(inp)
    housing.isGrounded = True

    # the bearing: outer ring, inner ring, balls (one body)
    outer = place(root, 'Outer ring')
    ring(outer.component, 0.0, B, g['ring_out_id'], D, 'Outer ring')
    inner = place(root, 'Inner ring')
    ring(inner.component, 0.0, B, d, g['ring_in_od'], 'Inner ring')
    balls = place(root, 'Balls')
    tbm = adsk.fusion.TemporaryBRepManager.get()
    rb = g['dm'] / 2.0
    ball = None
    for i in range(g['Z']):
        a = 2 * math.pi * i / g['Z']
        s = tbm.createSphere(P3(rb * math.cos(a) * CM, rb * math.sin(a) * CM, B / 2 * CM), g['Dw'] / 2 * CM)
        if ball is None:
            ball = s
        else:
            tbm.booleanOperation(ball, s, adsk.fusion.BooleanTypes.UnionBooleanType)
    bf = balls.component.features.baseFeatures.add()
    bf.startEdit()
    balls.component.bRepBodies.add(ball, bf)
    bf.finishEdit()
    bf.bodies.item(0).name = 'Balls'

    # the shaft, with the shoulder the inner ring sits on
    shaft = place(root, 'Shaft')
    sc = shaft.component
    sk = sc.sketches.add(plane_at(sc, -10.0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (d + 3.0) / 2 * CM)
    inp = ext(sc).createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, VI(10.0 * CM))
    sb = ext(sc).add(inp).bodies.item(0)
    sb.name = 'Shaft'
    sk = sc.sketches.add(sc.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d / 2 * CM)
    inp = ext(sc).createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    inp.setDistanceExtent(False, VI((B + 20.0) * CM))
    inp.participantBodies = [sb]
    ext(sc).add(inp)

    # joints, and the balls carried round at the cage's speed
    as_built(root, outer, housing, False, 'Outer ring in housing')
    j_shaft = as_built(root, shaft, housing, True, 'Shaft turns')
    as_built(root, inner, shaft, False, 'Inner ring on shaft')
    j_balls = as_built(root, balls, housing, True, 'Balls orbit')
    li = root.motionLinks.createInput(j_shaft, j_balls)
    li.valueOne = VS('360 deg')
    li.valueTwo = VS('%.6f deg' % (360.0 * g['k']))
    li.isReversed = False
    root.motionLinks.add(li).name = 'Cage speed'
    app.activeViewport.fit()

    return ('Bearing %s seated.\n\n'
            'd %.0f, D %.0f, B %.0f mm; C %.2f kN, C0 %.2f kN\n'
            'P = Fr = %.0f N at n = %.0f rpm\n'
            'L10 = (C / P)^3 = %.1f million revolutions; L10h = %.0f h (required %.0f h)\n'
            's0 = C0 / P0 = %.2f\n'
            'Simplified internals: dm %.2f mm, Dw %.2f mm, Z = %d balls\n'
            'Cage speed n_cage = n / 2 * (1 - Dw / dm) = %.5f n = %.1f rpm\n\n[%s]'
            % (bearing, d, D, B, C, C0, Fr, n, L10, L10h, L10h_req, s0, g['dm'], g['Dw'], g['Z'], g['k'], g['k'] * n, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(bearing=ins.itemById('bearing').selectedItem.name, n=ins.itemById('n').value,
                     Fr=ins.itemById('Fr').value, L10h_req=ins.itemById('L10h_req').value)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('bearing', 'Bearing', adsk.core.DropDownStyles.TextListDropDownStyle)
        for b in BEARINGS:
            dd.listItems.add(b, b == '6002')
        ins.addValueInput('n', 'Shaft speed (rpm)', '', VI(1000.0))
        ins.addValueInput('Fr', 'Radial load (N)', '', VI(500.0))
        ins.addValueInput('L10h_req', 'Required rating life (h)', '', VI(10000.0))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'BearingSeatCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Bearing seat', 'A ball bearing on a shaft and in a housing')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Bearing seat could not start:\n' + traceback.format_exc())
