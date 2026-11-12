# =============================================================================
# FitPair.py  -  a pin in a hole through a block, each made at a chosen limit of its ISO 286 tolerance (hole basis,
# H7), so that the fit can be seen and measured in Fusion. A pair with clearance at the chosen limits is joined with
# a revolute As-built Joint about z (the pin turns); a pair with interference is joined rigid (the pin is fixed).
#
#   limits(D, shaft)       the ISO 286 deviations (um), the largest and smallest clearance and the kind of fit
#   sizes(f, condition)    the hole's and the pin's diameters (mm) at maximum or minimum material
#   build(...)             a new Fusion design: Block (grounded) and Pin, one body each, and the joint
#   run(context)           the dialog; OK builds and shows the message
# =============================================================================
import os
import sys
import traceback

import adsk.core
import adsk.fusion

VERSION = 'FitPair v1.0 (prompt test)'
CMD_ID = 'mdfc_FitPair_prompt_test'
CM = 0.1             # the API works in cm: 1 mm = 0.1 cm
UM = 0.001           # 1 um = 0.001 mm


def _companion():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py, put common/ on
    sys.path and import it. The prompt names no library, so nothing else is imported from it."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            common = os.path.join(d, 'common')
            if common not in sys.path:
                sys.path.insert(0, common)
            import companion_paths
            return companion_paths
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up


CP = _companion()

# ---------- ISO 286, in micrometres ----------
# (above, up to, IT6, IT7, es of f, g, h, ei of k, n, p)
TABLE = (
    (3, 6, 8, 12, -10, -4, 0, 1, 8, 12),
    (6, 10, 9, 15, -13, -5, 0, 1, 10, 15),
    (10, 18, 11, 18, -16, -6, 0, 1, 12, 18),
    (18, 30, 13, 21, -20, -7, 0, 2, 15, 22),
    (30, 50, 16, 25, -25, -9, 0, 2, 17, 26),
)
SHAFTS = ('f7', 'g6', 'h6', 'k6', 'n6', 'p6')
HOLE = 'H7'
CONDITIONS = ('maximum material', 'minimum material')


def limits(D, shaft):
    """The H7 hole's and the shaft's deviations (um) for nominal size D (mm), the largest and smallest clearance (um,
    negative = interference) and the kind of fit. Refuses a D outside the table and a shaft class not in it."""
    try:
        D = float(D)
    except (TypeError, ValueError):
        raise ValueError('D must be a size in mm, not %r' % (D,))
    row = None
    for r in TABLE:
        if r[0] < D <= r[1]:
            row = r
            break
    if row is None:
        raise ValueError('D = %g mm is outside the ISO 286 table (above 3 mm up to 50 mm)' % D)
    if shaft not in SHAFTS:
        raise ValueError('shaft class %r is not in the table (%s)' % (shaft, ', '.join(SHAFTS)))
    lo, hi, it6, it7 = row[:4]
    dev = dict(zip(SHAFTS, row[4:]))
    it_shaft = it7 if shaft[1:] == '7' else it6
    EI, ES = 0, it7                                    # H7: EI = 0, ES = IT7
    if shaft[0] in 'fgh':                              # the table gives es; ei = es - IT
        es = dev[shaft]
        ei = es - it_shaft
    else:                                              # k, n, p: the table gives ei; es = ei + IT6
        ei = dev[shaft]
        es = ei + it_shaft
    c_max = ES - ei                                    # largest hole, smallest shaft
    c_min = EI - es                                    # smallest hole, largest shaft
    if c_min >= 0:
        kind = 'clearance'
    elif c_max <= 0:
        kind = 'interference'
    else:
        kind = 'transition'
    return dict(D=D, shaft=shaft, range=(lo, hi), IT6=it6, IT7=it7, EI=EI, ES=ES, ei=ei, es=es,
                c_max=c_max, c_min=c_min, kind=kind)


def sizes(f, condition):
    """The hole's and the pin's diameters (mm) at the chosen condition, and the clearance between them (um,
    negative = interference on the diameter)."""
    if condition not in CONDITIONS:
        raise ValueError('condition %r is not one of %s' % (condition, ' or '.join(CONDITIONS)))
    mmc = condition == 'maximum material'
    hole_um = f['EI'] if mmc else f['ES']              # smallest hole / largest hole
    pin_um = f['es'] if mmc else f['ei']               # largest pin / smallest pin
    return f['D'] + hole_um * UM, f['D'] + pin_um * UM, hole_um - pin_um


# ---------- the model ----------
def _P(x, y, z=0.0):
    return adsk.core.Point3D.create(x * CM, y * CM, z * CM)


def _new_occ(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def _extrude_symmetric(comp, profile, length, name):
    """A new body from the profile, `length` mm long in all, half on each side of the sketch plane z = 0."""
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(length * CM), True)
    feat = ext.add(inp)
    body = feat.bodies.item(0)
    body.name = name
    return body


def _block_body(comp, D, d_hole, L):
    """A square plate 2 D a side and L thick, from z = -L/2 to L/2, with the hole of diameter d_hole along z."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Plate and hole'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(_P(-D, -D), _P(D, D))
    sk.sketchCurves.sketchCircles.addByCenterRadius(_P(0, 0), d_hole / 2.0 * CM)
    prof = None
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        if p.profileLoops.count == 2:                  # the square less the circle
            prof = p
    if prof is None:
        raise RuntimeError('the plate sketch has no profile with a hole in it')
    return _extrude_symmetric(comp, prof, L, 'Block')


def _pin_body(comp, d_pin, length):
    """A cylinder of diameter d_pin along z, `length` mm long, centred on z = 0."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Pin'
    sk.sketchCurves.sketchCircles.addByCenterRadius(_P(0, 0), d_pin / 2.0 * CM)
    return _extrude_symmetric(comp, sk.profiles.item(0), length, 'Pin')


def _joint(root, pin, block, turns):
    """An As-built Joint between Pin and Block on the pin's axis: revolute about z if the pair has clearance at the
    chosen limits, rigid otherwise."""
    body = pin.bRepBodies.item(0)                      # a proxy in the root's context
    face = None
    for i in range(body.faces.count):
        f = body.faces.item(i)
        if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
            face = f
            break
    geo = adsk.fusion.JointGeometry.createByNonPlanarFace(face, adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
    ji = root.asBuiltJoints.createInput(pin, block, geo)
    if turns:
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        ji.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(ji)
    j.name = 'Pin in hole'
    return j


def _gap_text(gap):
    if gap > 0:
        return '%d um clearance on the diameter' % gap
    if gap < 0:
        return '%d um interference on the diameter' % -gap
    return 'line to line (0 um)'


def message(f, condition, L, d_hole, d_pin, gap, turns):
    """What was built: the four limits, the largest and smallest clearance, the kind of fit and the version tag."""
    D = f['D']
    lines = [
        VERSION,
        '%g %s/%s, engaged length %g mm, made at %s' % (D, HOLE, f['shaft'], L, condition),
        'hole %s: %.3f to %.3f mm (EI %+d um, ES %+d um)' % (HOLE, D + f['EI'] * UM, D + f['ES'] * UM, f['EI'], f['ES']),
        'pin %s: %.3f to %.3f mm (ei %+d um, es %+d um)' % (f['shaft'], D + f['ei'] * UM, D + f['es'] * UM,
                                                            f['ei'], f['es']),
        'largest clearance %d um, smallest clearance %d um (negative = interference)' % (f['c_max'], f['c_min']),
        '%s %s fit (IT6 = %d um, IT7 = %d um for D above %g up to %g mm)' % ('an' if f['kind'][0] in 'aeiou' else 'a', f['kind'], f['IT6'], f['IT7'],
                                                                         f['range'][0], f['range'][1]),
        'modelled: hole %.3f mm, pin %.3f mm, %s' % (d_hole, d_pin, _gap_text(gap)),
        'joint: revolute about z, the pin turns' if turns else 'joint: rigid, the pin is fixed',
    ]
    return '\n'.join(lines)


def build(D=20.0, shaft='g6', condition='maximum material', L=20.0):
    """A new Fusion design with the pair made at the chosen limits. Returns the message. Raises ValueError, before
    any document is made, for a D outside the table, a shaft class not in it, an unknown condition or L <= 0."""
    f = limits(D, shaft)
    d_hole, d_pin, gap = sizes(f, condition)
    try:
        L = float(L)
    except (TypeError, ValueError):
        raise ValueError('L must be a length in mm, not %r' % (L,))
    if not L > 0:
        raise ValueError('the engaged length L must be more than 0 mm (L = %g)' % L)
    D = f['D']
    turns = gap >= 0

    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    block = _new_occ(root, 'Block')
    _block_body(block.component, D, d_hole, L)
    block.isGrounded = True
    pin = _new_occ(root, 'Pin')
    _pin_body(pin.component, d_pin, L + 2 * 5.0)       # from z = -L/2 - 5 to L/2 + 5
    _joint(root, pin, block, turns)
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return message(f, condition, L, d_hole, d_pin, gap, turns)


# ---------- the dialog ----------
_handlers = []
_standalone = False          # True when Fusion runs this script; False when another script calls run(None)


def _read(ins):
    """(D, shaft, condition, L) from the dialog, sizes in mm."""
    for k in ('D', 'L'):
        if not ins.itemById(k).isValidExpression:
            raise ValueError('%s is not a valid length' % k)
    D = round(ins.itemById('D').value / CM, 9)
    L = round(ins.itemById('L').value / CM, 9)
    shaft = ins.itemById('shaft').selectedItem.name
    condition = ins.itemById('condition').selectedItem.name
    return D, shaft, condition, L


def _derived(ins):
    """The derived values shown read-only in the dialog."""
    try:
        D, shaft, condition, L = _read(ins)
        f = limits(D, shaft)
        d_hole, d_pin, gap = sizes(f, condition)
    except ValueError as e:
        return 'Refused: %s' % e
    return '<br>'.join([
        'IT6 %d um, IT7 %d um' % (f['IT6'], f['IT7']),
        'hole %s: %.3f to %.3f mm' % (HOLE, D + f['EI'] * UM, D + f['ES'] * UM),
        'pin %s: %.3f to %.3f mm' % (shaft, D + f['ei'] * UM, D + f['es'] * UM),
        'clearance %d to %d um: %s fit' % (f['c_min'], f['c_max'], f['kind']),
        'made at: hole %.3f, pin %.3f mm, %s' % (d_hole, d_pin, _gap_text(gap)),
        'joint: %s' % ('revolute' if gap >= 0 else 'rigid'),
    ])


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            ins = cmd.commandInputs
            ins.addValueInput('D', 'Nominal size D', 'mm', adsk.core.ValueInput.createByString('20 mm'))
            hole = ins.addStringValueInput('hole', 'Hole class (hole basis)', HOLE)
            hole.isReadOnly = True
            dd = ins.addDropDownCommandInput('shaft', 'Shaft class', adsk.core.DropDownStyles.TextListDropDownStyle)
            for s in SHAFTS:
                dd.listItems.add(s, s == 'g6')
            dc = ins.addDropDownCommandInput('condition', 'Condition', adsk.core.DropDownStyles.TextListDropDownStyle)
            for c in CONDITIONS:
                dc.listItems.add(c, c == 'maximum material')
            ins.addValueInput('L', 'Engaged length L', 'mm', adsk.core.ValueInput.createByString('20 mm'))
            ins.addTextBoxCommandInput('derived', 'Derived', _derived(ins), 7, True)
            for event, handler in ((cmd.execute, _Execute()), (cmd.inputChanged, _Changed()),
                                   (cmd.destroy, _Destroy())):
                event.add(handler)
                _handlers.append(handler)
        except Exception:
            ui.messageBox('FitPair: the dialog failed\n%s' % traceback.format_exc())


class _Changed(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.input.parentCommand.commandInputs
            if args.input.id != 'derived':
                ins.itemById('derived').formattedText = _derived(ins)
        except Exception:
            pass


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            D, shaft, condition, L = _read(args.command.commandInputs)
            ui.messageBox(build(D=D, shaft=shaft, condition=condition, L=L), 'FitPair')
        except ValueError as e:
            ui.messageBox('%s\nRefused: %s' % (VERSION, e), 'FitPair')
        except Exception:
            ui.messageBox('FitPair failed\n%s' % traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            if _standalone:
                adsk.terminate()
        except Exception:
            pass


def run(context):
    global _standalone
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        _standalone = context is not None
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Fit pair (ISO 286)',
                                                            'A pin in a hole, each at a chosen ISO 286 limit')
        on_created = _Created()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)
        cmd_def.execute()
        if _standalone:
            adsk.autoTerminate(False)          # keep the script alive for the dialog's events
    except Exception:
        if ui:
            ui.messageBox('FitPair failed\n%s' % traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
    except Exception:
        pass
