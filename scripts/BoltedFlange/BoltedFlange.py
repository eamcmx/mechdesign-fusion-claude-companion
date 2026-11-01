# =============================================================================
# BoltedFlange.py  -  Chapter 8 of "Mechanical Design in Autodesk Fusion with Claude": two flange plates clamped by
# hexagon bolts and nuts on a bolt circle; preload, tightening torque and the joint's safety factors under an external
# tensile load. Written from PROMPT.txt beside it; checked by checks/bolted_flange_check.py.
#
#   INPUTS        the dialog: bolt size, N, BCD, t1, t2, F_ext
#   DERIVED       joint(): thread and hexagon data, Fp, Fi, T, kb, km, C, P, Fb, Fm, np, n0
#   HARD RULES    build() refuses bolts too close for a socket, a joint that separates, bolts past their proof load
#   CONVENTIONS   a new design; plates, bolts and nuts created where they belong; hexagons as closed polylines;
#                 cm inside the API; As-built rigid joints
#   ACCEPTANCE    the message (M8, 4 bolts, 20 kN); positions; no interference; two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'BoltedFlange 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
E_STEEL = 207000.0                         # MPa
SP_88 = 580.0                              # MPa, proof stress of property class 8.8 (ISO 898-1)
K_TORQUE = 0.2                             # torque coefficient when the finish is not stated
PROTRUDE = 2.0                             # mm of shank past the nut

# d: (At mm^2, clearance hole d_h ISO 273 medium, across flats s and head height k ISO 4017, nut height m ISO 4032)
THREADS = {'M5': (5, 14.2, 5.5, 8.0, 3.5, 4.7), 'M6': (6, 20.1, 6.6, 10.0, 4.0, 5.2), 'M8': (8, 36.6, 9.0, 13.0, 5.3, 6.8),
           'M10': (10, 58.0, 11.0, 16.0, 6.4, 8.4), 'M12': (12, 84.3, 13.5, 18.0, 7.5, 10.8)}


def joint(size, N, t1, t2, F_ext):
    d, At, dh, s, k, m = THREADS[size]
    l = t1 + t2
    Fp = At * SP_88
    Fi = 0.75 * Fp
    T = K_TORQUE * Fi * d / 1000.0                       # N.m
    kb = At * E_STEEL / l
    km = 0.5774 * math.pi * E_STEEL * d / (2.0 * math.log(5.0 * (0.5774 * l + 0.5 * d) / (0.5774 * l + 2.5 * d)))
    C = kb / (kb + km)
    P = F_ext / N
    Fb = Fi + C * P
    Fm = Fi - (1.0 - C) * P
    return dict(d=d, At=At, dh=dh, s=s, k=k, m=m, l=l, Fp=Fp, Fi=Fi, T=T, kb=kb, km=km, C=C, P=P, Fb=Fb, Fm=Fm,
                np=Fp / Fb, n0=Fi / (P * (1.0 - C)) if P > 0 else float('inf'))


def place(root, name, x=0.0, y=0.0, z=0.0):
    """A new component created where it belongs: the position goes in the creation matrix."""
    mat = adsk.core.Matrix3D.create()
    mat.translation = V3(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def extrude(comp, profile, length_mm, op=adsk.fusion.FeatureOperations.NewBodyFeatureOperation, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


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


def plate(root, name, z0, t, Dp, R, N, dh):
    occ = place(root, name)
    c = occ.component
    sk = c.sketches.add(plane_at(c, z0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), Dp / 2 * CM)
    for i in range(N):
        a = 2 * math.pi * i / N
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(R * math.cos(a) * CM, R * math.sin(a) * CM, 0), dh / 2 * CM)
    prof = max(list(sk.profiles), key=lambda p: p.areaProperties().area)
    extrude(c, prof, t).bodies.item(0).name = name
    return occ


def rigid(root, moving, fixed, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    inp.setAsRigidJointMotion()
    root.asBuiltJoints.add(inp).name = name


def build(size='M8', N=4, BCD=60.0, t1=10.0, t2=10.0, F_ext=20000.0):
    if size not in THREADS:
        raise ValueError('Bolt size %s is not in the table: %s.' % (size, ', '.join(THREADS)))
    N = int(round(N))
    j = joint(size, N, t1, t2, F_ext)
    chord = BCD * math.sin(math.pi / N)
    if chord < 2 * j['s']:
        raise ValueError('%d bolts on a %.0f mm circle are %.1f mm apart: less than %.0f mm, two across-flats, leaves no room '
                         'for a socket.' % (N, BCD, chord, 2 * j['s']))
    if j['n0'] <= 1.0:
        raise ValueError('The joint separates: n0 = Fi / (P (1 - C)) = %.3f. Use more or larger bolts.' % j['n0'])
    if j['np'] <= 1.0:
        raise ValueError('The bolts pass their proof load: np = %.3f.' % j['np'])
    R = BCD / 2.0
    Dp = BCD + 4.0 * j['dh']
    d, s, k, m = j['d'], j['s'], j['k'], j['m']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    p1 = plate(root, 'Plate 1', -t1, t1, Dp, R, N, j['dh'])
    p1.isGrounded = True
    p2 = plate(root, 'Plate 2', 0.0, t2, Dp, R, N, j['dh'])
    rigid(root, p2, p1, 'Plate 2 to plate 1')
    z_bottom = -t1 - m - PROTRUDE
    for i in range(N):
        a = 2 * math.pi * i / N
        x, y = R * math.cos(a), R * math.sin(a)
        bolt = place(root, 'Bolt %d' % (i + 1), x, y)
        bc = bolt.component
        sk = bc.sketches.add(plane_at(bc, t2))
        hexagon(sk, s)
        body = extrude(bc, sk.profiles.item(0), k).bodies.item(0)
        body.name = 'Bolt %d' % (i + 1)
        sk = bc.sketches.add(plane_at(bc, z_bottom))
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d / 2 * CM)
        extrude(bc, sk.profiles.item(0), t2 - z_bottom, adsk.fusion.FeatureOperations.JoinFeatureOperation, body)
        nut = place(root, 'Nut %d' % (i + 1), x, y)
        nc = nut.component
        sk = nc.sketches.add(plane_at(nc, -t1 - m))
        hexagon(sk, s)
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d / 2 * CM)
        prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
        extrude(nc, prof, m).bodies.item(0).name = 'Nut %d' % (i + 1)
        rigid(root, bolt, p1, 'Bolt %d' % (i + 1))
        rigid(root, nut, p1, 'Nut %d' % (i + 1))
    app.activeViewport.fit()

    return ('Bolted flange built: %d x %s, property class 8.8, on a %.0f mm bolt circle; grip %.0f mm.\n\n'
            'At = %.1f mm^2, Fp = At Sp = %.0f N, preload Fi = 0.75 Fp = %.0f N\n'
            'Tightening torque T = K Fi d = %.2f N.m (K = 0.2)\n'
            'kb = %.0f N/mm, km = %.0f N/mm, joint constant C = %.5f\n'
            'Load per bolt P = %.0f N: bolt load Fb = %.0f N, clamping left Fm = %.0f N\n'
            'Yield factor np = %.3f, factor against separation n0 = %.3f\n\n[%s]'
            % (N, size, BCD, j['l'], j['At'], j['Fp'], j['Fi'], j['T'], j['kb'], j['km'], j['C'], j['P'], j['Fb'], j['Fm'],
               j['np'], j['n0'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(size=ins.itemById('size').selectedItem.name, N=ins.itemById('N').value,
                     BCD=ins.itemById('BCD').value / CM, t1=ins.itemById('t1').value / CM, t2=ins.itemById('t2').value / CM,
                     F_ext=ins.itemById('F_ext').value)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('size', 'Bolt size', adsk.core.DropDownStyles.TextListDropDownStyle)
        for name in THREADS:
            dd.listItems.add(name, name == 'M8')
        ins.addIntegerSpinnerCommandInput('N', 'Number of bolts', 3, 24, 1, 4)
        ins.addValueInput('BCD', 'Bolt circle diameter', 'mm', VI(60.0 * CM))
        ins.addValueInput('t1', 'Plate 1 thickness', 'mm', VI(10.0 * CM))
        ins.addValueInput('t2', 'Plate 2 thickness', 'mm', VI(10.0 * CM))
        ins.addValueInput('F_ext', 'External tensile load (N)', '', VI(20000.0))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'BoltedFlangeCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Bolted flange', 'Two plates bolted on a bolt circle')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Bolted flange could not start:\n' + traceback.format_exc())
