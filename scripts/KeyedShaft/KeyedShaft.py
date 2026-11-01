# =============================================================================
# KeyedShaft.py  -  Chapter 6 of "Mechanical Design in Autodesk Fusion with Claude": a shaft turning in a bearing
# block, with a hub fixed to it by a DIN 6885 parallel key. Written from PROMPT.txt beside it; checked by
# checks/keyed_shaft_check.py.
#
#   INPUTS        FIELDS (the dialog)
#   DERIVED       lookup_key() (DIN 6885), the hub's sizes, the force on the key and the three stresses
#   HARD RULES    build() refuses D outside the table, a key longer than the hub, and a key that is overstressed
#   CONVENTIONS   a new design; four components, one body each, created where they belong; cm inside the API;
#                 As-built Joints: shaft revolute on the block, key and hub rigid to the shaft
#   ACCEPTANCE    the message; key sizes and stresses for D = 20 and 30; no interference at 0 and 90 degrees;
#                 shaft, key and hub turn together; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'KeyedShaft 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
BLOCK_H = 20.0                             # mm, height of the bearing block
TOP_GAP = 5.0                              # mm, from the hub's top face to the shaft's end

# DIN 6885 parallel keys: (D above, D up to, b, h, t1, t2), mm
DIN6885 = [(10, 12, 4, 4, 2.5, 1.8), (12, 17, 5, 5, 3.0, 2.3), (17, 22, 6, 6, 3.5, 2.8), (22, 30, 8, 7, 4.0, 3.3),
           (30, 38, 10, 8, 5.0, 3.3), (38, 44, 12, 8, 5.0, 3.3), (44, 50, 14, 9, 5.5, 3.8), (50, 58, 16, 10, 6.0, 4.3)]

# (id, label, default, unit)
FIELDS = [('D', 'Shaft diameter', 20.0, 'mm'), ('L_shaft', 'Shaft length', 80.0, 'mm'), ('L_hub', 'Hub length', 25.0, 'mm'),
          ('t_wall', 'Hub wall', 8.0, 'mm'), ('L_key', 'Key length', 18.0, 'mm'), ('T', 'Torque to transmit (N.m)', 30.0, 'num'),
          ('tau_allow', 'Allowable shear stress (MPa)', 60.0, 'num'), ('sigma_allow', 'Allowable crushing stress (MPa)', 100.0, 'num')]


def lookup_key(D):
    for lo, hi, b, h, t1, t2 in DIN6885:
        if lo < D <= hi:
            return b, h, t1, t2
    raise ValueError('A shaft of %.1f mm is outside the DIN 6885 table used here (above 10 mm, up to 58 mm).' % D)


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


def extrude(comp, profile, length_mm, op=NEW, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def rect(sk, x0, y0, x1, y1):
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(x0 * CM, y0 * CM, 0), P3(x1 * CM, y1 * CM, 0))


def ring_profile(sk):
    """The profile with two loops: the ring between two circles, not the hole inside."""
    return [p for p in sk.profiles if p.profileLoops.count == 2][0]


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


def build(D=20.0, L_shaft=80.0, L_hub=25.0, t_wall=8.0, L_key=18.0, T=30.0, tau_allow=60.0, sigma_allow=100.0):
    # DERIVED, and the HARD CONSTRAINTS checked before anything is drawn
    b, h, t1, t2 = lookup_key(D)
    if L_key > L_hub:
        raise ValueError('The key (%.1f mm) is longer than the hub (%.1f mm).' % (L_key, L_hub))
    R = D / 2.0
    F = 2.0 * T * 1000.0 / D                      # N, with T in N.m and D in mm
    tau = F / (b * L_key)                         # MPa = N/mm^2
    sigma = F / ((h - t1) * L_key)
    tau_shaft = 16.0 * T * 1000.0 / (math.pi * D ** 3)
    over = []
    if tau > tau_allow:
        over.append('key shear stress %.1f MPa is over %.0f MPa by %.1f MPa' % (tau, tau_allow, tau - tau_allow))
    if sigma > sigma_allow:
        over.append('crushing stress %.1f MPa is over %.0f MPa by %.1f MPa' % (sigma, sigma_allow, sigma - sigma_allow))
    if over:
        raise ValueError('The key is overstressed: ' + '; '.join(over) + '. Use a longer key or a larger shaft.')
    z_top = L_shaft - TOP_GAP                      # the hub's top face
    z_hub = z_top - L_hub                          # the hub's bottom face
    z_mid = (z_top + z_hub) / 2.0
    side = max(40.0, D + 20.0)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    # bearing block, grounded, with its bore
    block = place(root, 'Bearing block')
    bc = block.component
    sk = bc.sketches.add(bc.xYConstructionPlane)
    rect(sk, -side / 2, -side / 2, side / 2, side / 2)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (D + 0.2) / 2 * CM)
    bb = extrude(bc, ring_profile(sk), BLOCK_H).bodies.item(0)
    bb.name = 'Bearing block'
    block.isGrounded = True

    # shaft, with its slot on the +y side, centred on the hub
    shaft = place(root, 'Shaft')
    scomp = shaft.component
    sk = scomp.sketches.add(scomp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), R * CM)
    sb = extrude(scomp, sk.profiles.item(0), L_shaft).bodies.item(0)
    sb.name = 'Shaft'
    slot = L_key + 1.0
    sk = scomp.sketches.add(plane_at(scomp, z_mid - slot / 2))
    rect(sk, -b / 2, R - t1, b / 2, R + 1.0)
    extrude(scomp, sk.profiles.item(0), slot, CUT, sb)

    # key: t1 of its height in the shaft's slot, the rest in the hub's slot
    key = place(root, 'Key')
    kc = key.component
    sk = kc.sketches.add(plane_at(kc, z_mid - L_key / 2))
    rect(sk, -b / 2, R - t1, b / 2, R - t1 + h)
    extrude(kc, sk.profiles.item(0), L_key).bodies.item(0).name = 'Key'

    # hub: a ring with its slot through its whole length
    hub = place(root, 'Hub')
    hc = hub.component
    sk = hc.sketches.add(plane_at(hc, z_hub))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (R + t_wall) * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (D + 0.1) / 2 * CM)
    hb = extrude(hc, ring_profile(sk), L_hub).bodies.item(0)
    hb.name = 'Hub'
    sk = hc.sketches.add(plane_at(hc, z_hub))
    rect(sk, -b / 2, R - 1.0, b / 2, R + t2)
    extrude(hc, sk.profiles.item(0), L_hub, CUT, hb)

    # joints: the shaft turns in the block; key and hub are locked to the shaft
    as_built(root, shaft, block, True, 'Shaft turns')
    as_built(root, key, shaft, False, 'Key in shaft')
    as_built(root, hub, shaft, False, 'Hub on shaft')
    app.activeViewport.fit()

    return ('Keyed shaft built.\n\n'
            'Shaft %.1f mm: DIN 6885 key %g x %g mm, shaft slot t1 = %g mm, hub slot t2 = %g mm\n'
            'Hub: bore %.2f mm, outside diameter %.1f mm, %.1f mm long\n\n'
            'Torque %.1f N.m: force on the key F = 2T/D = %.0f N\n'
            '  key shear stress     tau   = F / (b L)        = %.2f MPa (allowed %.0f)\n'
            '  crushing stress      sigma = F / ((h - t1) L) = %.2f MPa (allowed %.0f)\n'
            '  shaft torsion stress 16T / (pi D^3)           = %.2f MPa\n\n[%s]'
            % (D, b, h, t1, t2, D + 0.1, D + 2 * t_wall, L_hub, T, F, tau, tau_allow, sigma, sigma_allow, tau_shaft, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            p = {key: inputs.itemById(key).value / (CM if unit == 'mm' else 1.0) for key, _, _, unit in FIELDS}
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        cmd.setDialogInitialSize(420, 320)
        inputs = cmd.commandInputs
        for key, label, d, unit in FIELDS:
            if unit == 'mm':
                inputs.addValueInput(key, label, 'mm', VI(d * CM))
            else:
                inputs.addValueInput(key, label, '', VI(d))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'KeyedShaftCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Keyed shaft', 'A shaft in a bearing block with a keyed hub')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Keyed shaft could not start:\n' + traceback.format_exc())
