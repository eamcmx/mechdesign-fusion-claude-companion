# =============================================================================
# CamFollower.py  -  Chapter 23 of "Mechanical Design in Autodesk Fusion with Claude": a disc cam and a translating
# roller follower. The cam's outline is generated from the follower's motion law: the roller centre's path in the
# cam's frame, offset by the roller radius. Written from PROMPT.txt beside it; checked by checks/cam_check.py.
#
#   INPUTS        the dialog: R_b, R_r, h, beta1, dwell, beta2
#   DERIVED       cam(): s(theta), the pressure angle's largest value, the pitch curve's smallest radius of
#                 curvature; outline(): the cam's outline, 1 point per degree
#   HARD RULES    build() refuses more than 360 deg of motion, a pressure angle above 30 deg, an undercut
#   CONVENTIONS   a new design; parts at the origin, drawn in place; the outline one closed polyline; cm inside the
#                 API; a revolute and a slider As-built Joint and no Motion Link (a cam's ratio is not constant)
#   ACCEPTANCE    the message; at 36 steps of 10 deg no interference and a 0.05 mm gap measured by Fusion
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'CamFollower 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
GAP = 0.05                                 # mm, the running gap between roller and cam
Z0, T = 1.0, 8.0                           # mm, the cam's and the follower's level and thickness
BORE, STEM = 10.0, 6.0                     # mm, the cam's bore and the stem's width
PHI_MAX = 30.0                             # deg


def lift(h, b1, dw, b2):
    """s(theta) in mm, theta in radians: cycloidal rise over b1, dwell, cycloidal return over b2, dwell."""
    def s(t):
        t = t % (2 * math.pi)
        if t < b1:
            u = t / b1
            return h * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
        if t < b1 + dw:
            return h
        if t < b1 + dw + b2:
            u = (t - b1 - dw) / b2
            return h * (1 - (u - math.sin(2 * math.pi * u) / (2 * math.pi)))
        return 0.0
    return s


def cam(Rb, Rr, h, beta1, dwell, beta2):
    s = lift(h, math.radians(beta1), math.radians(dwell), math.radians(beta2))
    Rp = Rb + Rr
    d1 = lambda t, e=1e-6: (s(t + e) - s(t - e)) / (2 * e)
    d2 = lambda t, e=1e-4: (s(t + e) - 2 * s(t) + s(t - e)) / (e * e)
    phi, rho_convex, rho_concave = 0.0, float('inf'), float('inf')
    for i in range(36000):
        t = 2 * math.pi * i / 36000
        r, r1, r2 = Rp + s(t), d1(t), d2(t)
        phi = max(phi, abs(math.degrees(math.atan2(r1, r))))
        den = r * r + 2 * r1 * r1 - r * r2
        if abs(den) > 1e-12:
            rho = (r * r + r1 * r1) ** 1.5 / den
            if rho > 0:
                rho_convex = min(rho_convex, rho)
            else:
                rho_concave = min(rho_concave, -rho)
    return dict(s=s, Rp=Rp, phi=phi, rho=rho_convex, rho_concave=rho_concave)


def outline(c, Rr):
    """The cam's outline in its own frame: the roller centre's path (R_p + s)(sin t, cos t) for a cam turned
    anticlockwise by t, offset towards the centre by the roller radius and the gap; one point per degree."""
    pitch = [((c['Rp'] + c['s'](math.radians(k))) * math.sin(math.radians(k)),
              (c['Rp'] + c['s'](math.radians(k))) * math.cos(math.radians(k))) for k in range(360)]
    n = len(pitch)
    area = sum(pitch[i][0] * pitch[(i + 1) % n][1] - pitch[(i + 1) % n][0] * pitch[i][1] for i in range(n)) / 2
    d = Rr + GAP
    out = []
    for i in range(n):
        (x0, y0), (x1, y1) = pitch[i - 1], pitch[(i + 1) % n]
        L = math.hypot(x1 - x0, y1 - y0)
        nx, ny = -(y1 - y0) / L, (x1 - x0) / L                      # left normal: inward for an anticlockwise curve
        if area < 0:
            nx, ny = -nx, -ny
        out.append((pitch[i][0] + d * nx, pitch[i][1] + d * ny))
    return out


# ---------- Fusion building blocks (mm at the interface) ----------
def new_part(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def extrude(comp, prof, length_mm, op=NEW, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def polyline(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0] * CM, pts[0][1] * CM, 0), P3(pts[1][0] * CM, pts[1][1] * CM, 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0] * CM, q[1] * CM, 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def joint(root, moving, fixed, kind, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    if kind == 'revolute':
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsSliderJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the cam and follower ----------
def build(Rb=25.0, Rr=6.0, h=10.0, beta1=120.0, dwell=60.0, beta2=120.0):
    if beta1 + dwell + beta2 > 360.0 + 1e-9:
        raise ValueError('The motion takes %.0f + %.0f + %.0f = %.0f deg: more than one turn.'
                         % (beta1, dwell, beta2, beta1 + dwell + beta2))
    c = cam(Rb, Rr, h, beta1, dwell, beta2)
    if c['phi'] > PHI_MAX:
        raise ValueError('The pressure angle reaches %.1f deg: above %.0f deg. Use a longer rise or a larger base circle.'
                         % (c['phi'], PHI_MAX))
    if c['rho'] <= Rr:
        raise ValueError('The pitch curve\'s radius of curvature falls to %.2f mm, not more than the roller\'s %.1f mm: '
                         'the cam would be undercut.' % (c['rho'], Rr))

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    top = c['Rp'] + h + 20
    ground = new_part(root, 'Ground')
    g = ground.component
    sk = g.sketches.add(plane_at(g, -6.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-(c['Rp'] + h + 6) * CM, -(c['Rp'] + h + 6) * CM, 0),
                                                     P3((c['Rp'] + h + 6) * CM, top * CM, 0))
    extrude(g, sk.profiles.item(0), 6.0).bodies.item(0).name = 'Ground'
    ground.isGrounded = True

    cm = new_part(root, 'Cam')
    k = cm.component
    sk = k.sketches.add(plane_at(k, Z0))
    polyline(sk, outline(c, Rr))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), BORE / 2 * CM)
    extrude(k, [p for p in sk.profiles if p.profileLoops.count == 2][0], T).bodies.item(0).name = 'Cam'

    fol = new_part(root, 'Follower')
    f = fol.component
    sk = f.sketches.add(plane_at(f, Z0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, c['Rp'] * CM, 0), Rr * CM)
    body = extrude(f, sk.profiles.item(0), T).bodies.item(0)
    body.name = 'Follower'
    sk = f.sketches.add(plane_at(f, Z0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-STEM / 2 * CM, c['Rp'] * CM, 0), P3(STEM / 2 * CM, top * CM, 0))
    extrude(f, sk.profiles.item(0), T, JOIN, body)

    joint(root, cm, ground, 'revolute', 'Cam turns')
    joint(root, fol, ground, 'slider', 'Follower slides')
    colour(design, ground, 'Paint - Enamel Glossy (Grey)')
    colour(design, cm, 'Paint - Enamel Glossy (Yellow)')
    colour(design, fol, 'Paint - Enamel Glossy (Blue)')
    app.activeViewport.fit()

    return ('Cam and roller follower built: base circle %.1f mm, roller %.1f mm, lift %.1f mm.\n\n'
            'Cycloidal rise over %.0f deg, dwell %.0f deg, cycloidal return over %.0f deg\n'
            'Largest pressure angle %.2f deg; the pitch curve\'s smallest radius of curvature %.2f mm\n\n'
            'Turn "Cam turns" and set "Follower slides" to s(theta): Fusion cannot link them.\n\n[%s]'
            % (Rb, Rr, h, beta1, dwell, beta2, c['phi'], c['rho'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(Rb=ins.itemById('Rb').value / CM, Rr=ins.itemById('Rr').value / CM, h=ins.itemById('h').value / CM,
                     beta1=math.degrees(ins.itemById('beta1').value), dwell=math.degrees(ins.itemById('dwell').value),
                     beta2=math.degrees(ins.itemById('beta2').value))
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('Rb', 'Base circle radius', 'mm', VI(25.0 * CM))
        ins.addValueInput('Rr', 'Roller radius', 'mm', VI(6.0 * CM))
        ins.addValueInput('h', 'Lift', 'mm', VI(10.0 * CM))
        ins.addValueInput('beta1', 'Rise angle', 'deg', VI(math.radians(120.0)))
        ins.addValueInput('dwell', 'Dwell at the top', 'deg', VI(math.radians(60.0)))
        ins.addValueInput('beta2', 'Return angle', 'deg', VI(math.radians(120.0)))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'CamFollowerCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Cam and follower', 'A disc cam and a roller follower')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Cam and follower could not start:\n' + traceback.format_exc())
