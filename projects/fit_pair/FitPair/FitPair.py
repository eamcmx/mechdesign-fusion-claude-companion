# =============================================================================
# FitPair.py  -  Chapter 7 of "Mechanical Design in Autodesk Fusion with Claude": a pin in a hole, each made at a chosen
# limit of its ISO 286 tolerance, so that the fit can be measured in Fusion. Written from PROMPT.txt beside it;
# checked by checks/fit_pair_check.py.
#
#   INPUTS        FIELDS and CHOICES (the dialog)
#   DERIVED       limits(): IT grades, fundamental deviations, the four limits, the clearances and the kind of fit
#   HARD RULES    limits() refuses D outside the table and a class not in it; the model is made at the chosen limits
#   CONVENTIONS   a new design; two components created where they belong; cm inside the API (um -> mm -> cm);
#                 an As-built Joint, revolute for a clearance fit, rigid otherwise
#   ACCEPTANCE    the message; 20 H7/g6, 20 H7/p6 (22.010 mm^3 of overlap), 15 H7/k6; two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'FitPair 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
UM = 0.001                                 # micrometres to millimetres
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
STICK_OUT = 5.0                            # mm, how far the pin stands out of each face of the block

# ISO 286, in micrometres: (above, up to, IT6, IT7, {letter: fundamental deviation})
# f, g, h give the upper deviation es of the shaft; k, n, p give its lower deviation ei.
ISO286 = [(3, 6, 8, 12, {'f': -10, 'g': -4, 'h': 0, 'k': 1, 'n': 8, 'p': 12}),
          (6, 10, 9, 15, {'f': -13, 'g': -5, 'h': 0, 'k': 1, 'n': 10, 'p': 15}),
          (10, 18, 11, 18, {'f': -16, 'g': -6, 'h': 0, 'k': 1, 'n': 12, 'p': 18}),
          (18, 30, 13, 21, {'f': -20, 'g': -7, 'h': 0, 'k': 2, 'n': 15, 'p': 22}),
          (30, 50, 16, 25, {'f': -25, 'g': -9, 'h': 0, 'k': 2, 'n': 17, 'p': 26})]
SHAFTS = ['f7', 'g6', 'h6', 'k6', 'n6', 'p6']
CONDITIONS = ['maximum material', 'minimum material']

FIELDS = [('D', 'Nominal size', 20.0), ('L', 'Engaged length', 20.0)]


def limits(D, shaft='g6', hole='H7'):
    """The four limits in mm and the clearances in um, for hole H7 and one of the shafts in the table."""
    row = [r for r in ISO286 if r[0] < D <= r[1]]
    if not row:
        raise ValueError('A size of %.1f mm is outside the ISO 286 table used here (above 3 mm, up to 50 mm).' % D)
    if hole != 'H7' or shaft not in SHAFTS:
        raise ValueError('The class %s/%s is not in the table: hole H7 with a shaft %s.' % (hole, shaft, ', '.join(SHAFTS)))
    _, _, it6, it7, dev = row[0]
    EI, ES = 0, it7
    letter, grade = shaft[0], int(shaft[1])
    it = it6 if grade == 6 else it7
    if letter in 'fgh':
        es = dev[letter]
        ei = es - it
    else:
        ei = dev[letter]
        es = ei + it
    c_max, c_min = ES - ei, EI - es
    kind = 'clearance' if c_min >= 0 else ('interference' if c_max <= 0 else 'transition')
    return dict(EI=EI, ES=ES, ei=ei, es=es, c_max=c_max, c_min=c_min, kind=kind)


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


def extrude(comp, profile, length_mm):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    return ext.add(inp)


def build(D=20.0, L=20.0, shaft='g6', condition='maximum material'):
    f = limits(D, shaft)
    maxmat = condition == 'maximum material'
    d_hole = D + (f['EI'] if maxmat else f['ES']) * UM
    d_pin = D + (f['es'] if maxmat else f['ei']) * UM
    gap = (d_hole - d_pin) / UM                    # um, at the chosen condition (negative = interference)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    # the block: a square of side 2D with the hole, extruded L along z, from -L/2 to L/2
    block = place(root, 'Block')
    bc = block.component
    sk = bc.sketches.add(plane_at(bc, -L / 2))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-D * CM, -D * CM, 0), P3(D * CM, D * CM, 0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d_hole / 2 * CM)
    prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    extrude(bc, prof, L).bodies.item(0).name = 'Block'
    block.isGrounded = True

    # the pin: a cylinder along z, standing out of both faces
    pin = place(root, 'Pin')
    pc = pin.component
    sk = pc.sketches.add(plane_at(pc, -L / 2 - STICK_OUT))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d_pin / 2 * CM)
    extrude(pc, sk.profiles.item(0), L + 2 * STICK_OUT).bodies.item(0).name = 'Pin'

    # a clearance fit turns; a transition or interference fit is fixed
    jp = pc.sketches.add(pc.xYConstructionPlane)
    jp.isVisible = False
    pt = jp.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(pin)
    ji = root.asBuiltJoints.createInput(pin, block, adsk.fusion.JointGeometry.createByPoint(pt))
    if gap >= 0:
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        jname = 'Pin turns (clearance)'
    else:
        ji.setAsRigidJointMotion()
        jname = 'Pin fixed (interference)'
    root.asBuiltJoints.add(ji).name = jname
    app.activeViewport.fit()

    return ('Fit %g H7/%s built at %s.\n\n'
            'Hole %.3f to %.3f mm (EI %+d, ES %+d um)\n'
            'Pin  %.3f to %.3f mm (ei %+d, es %+d um)\n'
            'Clearance from %+d to %+d um (negative = interference): a %s fit\n'
            'As built: hole %.3f, pin %.3f, %s %d um on the diameter; joint "%s"\n\n[%s]'
            % (D, shaft, condition, D + f['EI'] * UM, D + f['ES'] * UM, f['EI'], f['ES'],
               D + f['ei'] * UM, D + f['es'] * UM, f['ei'], f['es'], f['c_min'], f['c_max'], f['kind'],
               d_hole, d_pin, 'clearance' if gap >= 0 else 'interference', abs(round(gap)), jname, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            p = {key: inputs.itemById(key).value / CM for key, _, _ in FIELDS}
            p['shaft'] = inputs.itemById('shaft').selectedItem.name
            p['condition'] = inputs.itemById('condition').selectedItem.name
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        inputs = cmd.commandInputs
        inputs.addValueInput('D', 'Nominal size', 'mm', VI(20.0 * CM))
        inputs.addTextBoxCommandInput('hole', 'Hole class', 'H7', 1, True)
        dd = inputs.addDropDownCommandInput('shaft', 'Shaft class', adsk.core.DropDownStyles.TextListDropDownStyle)
        for s in SHAFTS:
            dd.listItems.add(s, s == 'g6')
        dc = inputs.addDropDownCommandInput('condition', 'Condition', adsk.core.DropDownStyles.TextListDropDownStyle)
        for c in CONDITIONS:
            dc.listItems.add(c, c == 'maximum material')
        inputs.addValueInput('L', 'Engaged length', 'mm', VI(20.0 * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'FitPairCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'ISO 286 fit', 'A pin in a hole at a chosen limit of its fit')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ISO 286 fit could not start:\n' + traceback.format_exc())
