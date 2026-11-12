# =============================================================================
# BearingSeat.py  -  a deep groove ball bearing seated on a shaft and in a housing (Chapter 8).
#
# Run it: a new design opens with a dialog (bearing 6002 or 61902, shaft speed n, radial load Fr, required life
# L10h_req). OK checks the rating life and the ball spacing, then builds five components on the z axis (Housing,
# Outer ring, Inner ring, Balls, Shaft) and joins them with As-built Joints and one Motion Link, so that Drive
# Joints on "Shaft turns" turns the inner ring with the shaft and carries the balls round at the cage's speed.
#
# Every size below is in millimetres; the API works in centimetres, so each is multiplied by CM on its way in.
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


# ---------- the companion's libraries, found through common/companion_paths.py ----------
def _companion_paths():
    here = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(here, 'common', 'companion_paths.py')):
            common = os.path.join(here, 'common')
            if common not in sys.path:
                sys.path.insert(0, common)
            import companion_paths
            return companion_paths
        up = os.path.dirname(here)
        if up == here:
            return None                 # not inside the companion: this script needs none of its libraries
        here = up


CP = _companion_paths()

VERSION = 'BearingSeat v1.0 (prompt test, 2026-11-11)'
CMD_ID = 'mdfcBearingSeatPromptTest'
CM = 0.1                                   # mm to cm

# SKF catalogue data: sizes in mm, load ratings in N (C 5.85 kN = 5850 N)
BEARINGS = {
    '6002':  dict(d=15.0, D=32.0, B=9.0, C=5850.0, C0=2850.0),
    '61902': dict(d=15.0, D=28.0, B=7.0, C=4360.0, C0=2240.0),
}
DEFAULTS = dict(bearing='6002', n=1000.0, Fr=500.0, L10h_req=10000.0)


# ---------- the design calculation ----------
def geometry(bearing):
    """Catalogue sizes and the simplified internal geometry of the bearing. k = n_cage / n."""
    if bearing not in BEARINGS:
        raise ValueError('bearing %r is not one of %s' % (bearing, ', '.join(sorted(BEARINGS))))
    b = BEARINGS[bearing]
    d, D = b['d'], b['D']
    dm = (d + D) / 2.0                                   # pitch diameter
    Dw = 0.3 * (D - d)                                   # ball diameter
    Z = int(math.floor(math.pi * dm / (1.25 * Dw)))      # number of balls
    return dict(name=bearing, d=d, D=D, B=b['B'], C=b['C'], C0=b['C0'],
                dm=dm, Dw=Dw, Z=Z,
                in_od=dm - Dw - 0.1,                     # inner ring outside diameter
                out_id=dm + Dw + 0.1,                    # outer ring inside diameter
                pitch=dm * math.sin(math.pi / Z),        # centre distance of two neighbouring balls
                k=0.5 * (1.0 - Dw / dm))                 # n_cage / n, outer ring held, no contact angle


def rating(bearing, n, Fr):
    """Basic rating life of a ball bearing under a pure radial load: (L10 in million revolutions, L10h in hours,
    static safety factor s0)."""
    g = geometry(bearing)
    P = Fr                                               # pure radial load: P = Fr
    P0 = Fr                                              # and P0 = Fr
    L10 = (g['C'] / P) ** 3
    L10h = 1.0e6 * L10 / (60.0 * n)
    s0 = g['C0'] / P0
    return L10, L10h, s0


def load_for_life(bearing, n, L10h_req):
    """The load P at which the bearing just reaches L10h_req at n rpm."""
    g = geometry(bearing)
    L10_req = L10h_req * 60.0 * n / 1.0e6
    return g['C'] / L10_req ** (1.0 / 3.0)


# ---------- temporary bodies, world millimetres, all on the z axis ----------
def _tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def _pt(x, y, z):
    return adsk.core.Point3D.create(x * CM, y * CM, z * CM)


def _boolean(a, b, kind):
    if not _tbm().booleanOperation(a, b, kind):
        raise RuntimeError('a boolean of temporary bodies failed')
    return a


def _union(a, b):
    return _boolean(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)


def _cut(a, b):
    return _boolean(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)


def _cyl(z0, z1, r):
    """A cylinder of radius r on the z axis from z0 to z1."""
    return _tbm().createCylinderOrCone(_pt(0, 0, z0), r * CM, _pt(0, 0, z1), r * CM)


def _tube(z0, z1, r_out, r_in):
    return _cut(_cyl(z0, z1, r_out), _cyl(z0 - 1.0, z1 + 1.0, r_in))


def _square_block(side, z0, z1):
    centre = _pt(0, 0, (z0 + z1) / 2.0)
    box = adsk.core.OrientedBoundingBox3D.create(centre, adsk.core.Vector3D.create(1, 0, 0),
                                                 adsk.core.Vector3D.create(0, 1, 0),
                                                 side * CM, side * CM, (z1 - z0) * CM)
    return _tbm().createBox(box)


def _sphere(x, y, z, r):
    return _tbm().createSphere(_pt(x, y, z), r * CM)


def _new_occ(root, name):
    """A new component in the root, its origin on the world origin, so its bodies sit where they belong."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def _add_body(occ, tb, name):
    comp = occ.component
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


# ---------- joints ----------
def _rigid(root, occ_one, occ_two, name):
    inp = root.asBuiltJoints.createInput(occ_one, occ_two, None)
    inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def _revolute_z(root, occ_one, occ_two, name):
    """Revolute about the z axis through the origin of occ_two (the housing, which sits on the world origin)."""
    origin = occ_two.component.originConstructionPoint.createForAssemblyContext(occ_two)
    geo = adsk.fusion.JointGeometry.createByPoint(origin)
    inp = root.asBuiltJoints.createInput(occ_one, occ_two, geo)
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


# ---------- the design ----------
def _new_design():
    """A new Fusion design document, parametric, able to hold components (not a part design)."""
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    try:
        if design.designIntent == adsk.fusion.DesignIntentTypes.PartDesignIntentType:
            design.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
    except AttributeError:
        pass                                             # a Fusion without design intents: every design is hybrid
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    return design


def _assemble(design, g):
    root = design.rootComponent
    d, D, B = g['d'], g['D'], g['B']

    housing = _new_occ(root, 'Housing')
    outer = _new_occ(root, 'Outer ring')
    inner = _new_occ(root, 'Inner ring')
    balls = _new_occ(root, 'Balls')
    shaft = _new_occ(root, 'Shaft')

    # Housing: a square block D + 16 from z = -3 to B, bore D from z = 0 to B, bore D - 4 below it (the shoulder)
    hb = _square_block(D + 16.0, -3.0, B)
    _cut(hb, _cyl(0.0, B + 1.0, D / 2.0))
    _cut(hb, _cyl(-4.0, 1.0, (D - 4.0) / 2.0))
    _add_body(housing, hb, 'Housing')

    # Outer ring: D outside, dm + Dw + 0.1 inside, z = 0 to B
    _add_body(outer, _tube(0.0, B, D / 2.0, g['out_id'] / 2.0), 'Outer ring')

    # Inner ring: dm - Dw - 0.1 outside, d inside, z = 0 to B
    _add_body(inner, _tube(0.0, B, g['in_od'] / 2.0, d / 2.0), 'Inner ring')

    # Balls: Z balls of diameter Dw, centres at radius dm / 2 and height B / 2, one body
    rb = g['dm'] / 2.0
    bb = None
    for i in range(g['Z']):
        a = 2.0 * math.pi * i / g['Z']
        s = _sphere(rb * math.cos(a), rb * math.sin(a), B / 2.0, g['Dw'] / 2.0)
        bb = s if bb is None else _union(bb, s)
    _add_body(balls, bb, 'Balls')

    # Shaft: d from z = 0 to B + 20 (run down to -10 inside the shoulder), shoulder d + 3 from z = -10 to 0
    sb = _cyl(-10.0, B + 20.0, d / 2.0)
    _union(sb, _cyl(-10.0, 0.0, (d + 3.0) / 2.0))
    _add_body(shaft, sb, 'Shaft')

    housing.isGrounded = True

    # As-built joints only, and one Motion Link: shaft to balls at the cage's speed
    _rigid(root, outer, housing, 'Outer ring in housing')
    j_shaft = _revolute_z(root, shaft, housing, 'Shaft turns')
    _rigid(root, inner, shaft, 'Inner ring on shaft')
    j_balls = _revolute_z(root, balls, housing, 'Balls orbit')

    ml = root.motionLinks.createInput(j_shaft, j_balls)
    ml.valueOne = adsk.core.ValueInput.createByString('360 deg')
    ml.valueTwo = adsk.core.ValueInput.createByString('%.9f deg' % (360.0 * g['k']))
    ml.isReversed = False                                # same direction
    link = root.motionLinks.add(ml)
    link.name = 'Balls at cage speed'

    try:
        adsk.core.Application.get().activeViewport.fit()
    except Exception:
        pass


def build(bearing=DEFAULTS['bearing'], n=DEFAULTS['n'], Fr=DEFAULTS['Fr'], L10h_req=DEFAULTS['L10h_req'],
          design=None):
    """Check the bearing and build the assembly. Raises ValueError (before any document is made) when a hard
    constraint is broken. design: the design to build into; None makes a new design document. Returns the message."""
    g = geometry(bearing)
    if n <= 0 or Fr <= 0 or L10h_req <= 0:
        raise ValueError('n, Fr and L10h_req must be positive (n %g rpm, Fr %g N, L10h_req %g h)' % (n, Fr, L10h_req))
    L10, L10h, s0 = rating(bearing, n, Fr)
    if L10h < L10h_req:
        P_max = load_for_life(bearing, n, L10h_req)
        raise ValueError('%s refused at Fr = %.0f N and n = %.0f rpm: L10h = %.0f h, less than the required %.0f h. '
                         'It would just reach %.0f h at P = Fr = %.1f N.'
                         % (bearing, Fr, n, L10h, L10h_req, L10h_req, P_max))
    if not g['pitch'] > g['Dw']:
        raise ValueError('%s refused: the balls touch each other, dm sin(pi / Z) = %.3f mm is not more than Dw = %.3f mm'
                         % (bearing, g['pitch'], g['Dw']))
    if design is None:
        design = _new_design()
    _assemble(design, g)
    n_cage = n * g['k']
    return ('%s\n\n'
            'Bearing %s (SKF): d %.0f mm, D %.0f mm, B %.0f mm, C %.2f kN, C0 %.2f kN\n'
            'Shaft speed n %.0f rpm, radial load Fr %.0f N\n'
            'P = Fr = %.0f N, P0 = Fr = %.0f N\n'
            'L10 = (C / P)^3 = %.1f million revolutions\n'
            'L10h = 10^6 L10 / (60 n) = %.0f h (required %.0f h)\n'
            's0 = C0 / P0 = %.2f\n'
            'dm %.2f mm, Dw %.2f mm, Z = %d balls (dm sin(pi / Z) = %.2f mm > Dw)\n'
            'n_cage = n / 2 (1 - Dw / dm) = %.1f rpm, n_cage / n = %.5f\n\n'
            'Drive Joints on "Shaft turns": the inner ring turns with the shaft, the balls at %.5f of its angle.'
            % (VERSION, bearing, g['d'], g['D'], g['B'], g['C'] / 1000.0, g['C0'] / 1000.0, n, Fr, Fr, Fr,
               L10, L10h, L10h_req, s0, g['dm'], g['Dw'], g['Z'], g['pitch'], n_cage, g['k'], g['k']))


# ---------- the dialog ----------
_handlers = []
_state = {'design': None, 'standalone': False}


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            ins = cmd.commandInputs
            dd = ins.addDropDownCommandInput('bearing', 'Bearing', adsk.core.DropDownStyles.TextListDropDownStyle)
            for name in ('6002', '61902'):
                dd.listItems.add(name, name == DEFAULTS['bearing'], '')
            ins.addValueInput('n', 'Shaft speed n (rpm)', '', adsk.core.ValueInput.createByReal(DEFAULTS['n']))
            ins.addValueInput('Fr', 'Radial load Fr (N)', '', adsk.core.ValueInput.createByReal(DEFAULTS['Fr']))
            ins.addValueInput('L10h_req', 'Required rating life L10h_req (h)', '',
                              adsk.core.ValueInput.createByReal(DEFAULTS['L10h_req']))
            on_execute = _Execute()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui.messageBox('BearingSeat: the dialog failed\n' + traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            ins = args.command.commandInputs
            bearing = ins.itemById('bearing').selectedItem.name
            n = ins.itemById('n').value
            Fr = ins.itemById('Fr').value
            L10h_req = ins.itemById('L10h_req').value
            design = _state['design']
            if design is None or not design.isValid:
                design = adsk.fusion.Design.cast(app.activeProduct)
            msg = build(bearing=bearing, n=n, Fr=Fr, L10h_req=L10h_req, design=design)
            ui.messageBox(msg, 'Bearing seat')
        except ValueError as e:
            ui.messageBox('%s\n\nNothing built: %s' % (VERSION, e), 'Bearing seat')
        except Exception:
            ui.messageBox('BearingSeat failed\n' + traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _state['standalone']:
            adsk.terminate()


def run(context):
    """Opens a new design and the dialog in it. A document cannot be made inside a command's events, so it is made
    here, before the command starts; build() called directly makes its own."""
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        _handlers.clear()
        _state['standalone'] = context is not None       # run by Fusion itself, not called from another script
        _state['design'] = _new_design()
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Bearing seat',
                                                       'A deep groove ball bearing on a shaft and in a housing')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        if _state['standalone']:
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('BearingSeat failed\n' + traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
