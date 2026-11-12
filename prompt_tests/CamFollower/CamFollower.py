# -*- coding: utf-8 -*-
"""CamFollower.py: a disc cam with a translating roller follower, built in a new Fusion design.

The cam's outline comes from a cycloidal rise, a dwell at the top, a cycloidal return and a dwell at the bottom. The
roller's centre runs on the pitch curve; the cam's outline is that curve offset towards the cam's centre by the roller
radius plus a 0.05 mm running gap, one point per degree, one closed polyline. The script reports the cam's largest
pressure angle and the pitch curve's smallest radius of curvature, and refuses a design whose angles add up to more
than 360 deg, whose pressure angle passes 30 deg or whose pitch curve would undercut the cam.

Three components, one body each, drawn in place at theta = 0:
    Ground    a plate from z = -6 to 0, grounded
    Cam       z = 1 to 9, the outline with a 10 mm bore; As-built revolute joint 'Cam turns' about z on Ground
    Follower  z = 1 to 9, the roller round (0, R_p) and a 6 mm stem up +y; As-built slider 'Follower slides' along y
There is no Motion Link: the cam's lift is not a constant ratio of its turn. Whoever turns the cam sets the follower
to s(theta) with cam(...)['s'].

Sizes are in mm and angles in degrees everywhere in this file except where the Fusion API is called (cm, radians).
"""
import json
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    """The first folder above this script that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % os.path.dirname(os.path.abspath(__file__)))
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402,F401  (puts every lib/ folder on sys.path)
import mdfc_parts as P  # noqa: E402

VERSION = 'CamFollower v1.0 (2026-11-11)'

CMD_ID = 'mdfcCamFollowerDialog'
EVENT_ID = 'mdfcCamFollowerBuild'

DEFAULTS = dict(R_b=25.0, R_r=6.0, h=10.0, beta1=120.0, dwell=60.0, beta2=120.0)

GAP = 0.05            # running gap between the roller and the cam (mm)
BORE_D = 10.0         # the cam's bore (diameter, mm)
STEM_W = 6.0          # the follower's stem, wide (mm)
STEM_L = 40.0         # the stem's length above the roller's centre (mm); the prompt gives no length
PLATE_MARGIN = 10.0   # the ground plate reaches this far past the cam and the stem (mm); the prompt gives no size
Z_PLATE = (-6.0, 0.0)
Z_PART = (1.0, 9.0)
PHI_LIMIT = 30.0      # largest pressure angle allowed (deg)
N_SCAN = 36000        # samples per turn for the largest pressure angle and the smallest radius of curvature

TWO_PI = 2.0 * math.pi


# ---------------------------------------------------------------------------------------------------------------
# The motion law and the cam's numbers (no Fusion needed)
# ---------------------------------------------------------------------------------------------------------------
def motion_law(h, beta1, dwell, beta2):
    """s(theta) and its first two derivatives for a cam turned by theta (radians): cycloidal rise over beta1, dwell
    at h, cycloidal return over beta2, dwell at 0 for the rest of the turn. Returns f(theta) -> (s, s', s'') in mm,
    mm/rad and mm/rad^2."""
    b1, d, b2 = math.radians(beta1), math.radians(dwell), math.radians(beta2)
    t_top, t_back = b1 + d, b1 + d + b2

    def f(theta):
        t = theta % TWO_PI
        if t < b1:
            u = t / b1
            return (h * (u - math.sin(TWO_PI * u) / TWO_PI),
                    h / b1 * (1.0 - math.cos(TWO_PI * u)),
                    TWO_PI * h / b1 ** 2 * math.sin(TWO_PI * u))
        if t < t_top:
            return h, 0.0, 0.0
        if t < t_back:
            u = (t - t_top) / b2
            return (h * (1.0 - (u - math.sin(TWO_PI * u) / TWO_PI)),
                    -h / b2 * (1.0 - math.cos(TWO_PI * u)),
                    -TWO_PI * h / b2 ** 2 * math.sin(TWO_PI * u))
        return 0.0, 0.0, 0.0
    return f


def cam(R_b, R_r, h, beta1, dwell, beta2):
    """Everything derived from the six inputs: the lift s(theta) (theta in radians, s in mm), the largest pressure
    angle phi (deg), the pitch curve's smallest radius of curvature rho (mm), the pitch curve and the cam's outline
    (one point per degree, in the cam's own frame, mm). Raises ValueError when the angles do not fit in one turn."""
    for name, v in (('R_b', R_b), ('R_r', R_r), ('h', h), ('beta1', beta1), ('beta2', beta2)):
        if not v > 0:
            raise ValueError('%s must be larger than 0 (it is %g)' % (name, v))
    if dwell < 0:
        raise ValueError('the dwell at the top cannot be negative (it is %g deg)' % dwell)
    total = beta1 + dwell + beta2
    if total > 360.0 + 1e-9:
        raise ValueError('beta1 + dwell + beta2 = %g + %g + %g = %g deg, more than the 360 deg of one turn'
                         % (beta1, dwell, beta2, total))
    law = motion_law(h, beta1, dwell, beta2)
    R_p = R_b + R_r

    phi, phi_at = 0.0, 0.0
    rho, rho_at = float('inf'), 0.0
    concave = False
    for k in range(N_SCAN):
        th = TWO_PI * k / N_SCAN
        s, ds, dds = law(th)
        r = R_p + s
        p = abs(math.degrees(math.atan(ds / r)))
        if p > phi:
            phi, phi_at = p, math.degrees(th)
        den = r * r + 2.0 * ds * ds - r * dds
        if den > 0.0:
            rc = (r * r + ds * ds) ** 1.5 / den
            if rc < rho:
                rho, rho_at = rc, math.degrees(th)
        else:
            concave = True          # the pitch curve is flat or concave here: a roller cannot undercut it

    pitch, outline = [], []
    off = R_r + GAP
    for k in range(360):
        th = math.radians(k)
        s, ds, _ = law(th)
        r = R_p + s
        sn, cs = math.sin(th), math.cos(th)
        px, py = r * sn, r * cs                              # the roller's centre in the cam's frame
        tx, ty = ds * sn + r * cs, ds * cs - r * sn          # d(pitch)/d(theta)
        L = math.hypot(tx, ty)
        nx, ny = ty / L, -tx / L                             # unit normal towards the cam's centre
        pitch.append((px, py))
        outline.append((px + off * nx, py + off * ny))
    return dict(s=lambda theta: law(theta)[0], law=law, phi=phi, phi_at=phi_at, rho=rho, rho_at=rho_at,
                concave=concave, R_p=R_p, pitch=pitch, outline=outline,
                r_min=min(math.hypot(x, y) for x, y in outline),
                r_max=max(math.hypot(x, y) for x, y in outline),
                inputs=dict(R_b=R_b, R_r=R_r, h=h, beta1=beta1, dwell=dwell, beta2=beta2))


def refuse(c):
    """Raises ValueError when the cam breaks a hard constraint of the design."""
    R_r = c['inputs']['R_r']
    if c['phi'] > PHI_LIMIT:
        raise ValueError('the pressure angle reaches %.1f deg (at theta = %.1f deg), more than the %g deg allowed'
                         % (c['phi'], c['phi_at'], PHI_LIMIT))
    if not c['rho'] > R_r:
        raise ValueError("the pitch curve's smallest radius of curvature is %.2f mm (at theta = %.1f deg), not larger "
                         "than the roller radius %g mm: the cam would be undercut" % (c['rho'], c['rho_at'], R_r))
    if not c['r_min'] > BORE_D / 2.0:
        raise ValueError("the cam's outline comes within %.2f mm of its centre, inside the %g mm bore"
                         % (c['r_min'], BORE_D))


# ---------------------------------------------------------------------------------------------------------------
# The model in Fusion
# ---------------------------------------------------------------------------------------------------------------
def _new_design():
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    intent_types = getattr(adsk.fusion, 'DesignIntentTypes', None)
    if intent_types is not None and hasattr(des, 'designIntent'):
        if des.designIntent == intent_types.PartDesignIntentType:     # a part design cannot hold components
            des.designIntent = intent_types.HybridDesignIntentType
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    return des


def _cam_body(occ, c):
    """The cam: the outline (one closed polyline) less the bore, extruded from z = 1 to 9 in its component."""
    comp = occ.component
    CM = P.CM
    z0, z1 = Z_PART
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, P.VI(z0 * CM))
    plane = comp.constructionPlanes.add(pin)
    plane.name = 'Cam z = 1'
    sk = comp.sketches.add(plane)
    sk.name = 'Cam outline'
    sk.isComputeDeferred = True
    lines = sk.sketchCurves.sketchLines
    pts = [sk.modelToSketchSpace(P.P3(x * CM, y * CM, z0 * CM)) for x, y in c['outline']]
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P.P3(0, 0, z0 * CM)), BORE_D / 2.0 * CM)
    sk.isComputeDeferred = False
    prof = None
    for i in range(sk.profiles.count):
        if sk.profiles.item(i).profileLoops.count == 2:     # the ring between the outline and the bore
            prof = sk.profiles.item(i)
    if prof is None:
        raise RuntimeError('the cam sketch gave no profile between the outline and the bore')
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    normal = sk.xDirection.crossProduct(sk.yDirection)
    inp.setDistanceExtent(False, P.VI((z1 - z0) * CM * (1 if normal.z > 0 else -1)))
    feat = ext.add(inp)
    body = feat.bodies.item(0)
    body.name = 'Cam'
    return body


def _origin_geometry(root, ground):
    """A joint geometry at the origin, on Ground (its origin point in the assembly), else the root's origin."""
    g = None
    try:
        g = adsk.fusion.JointGeometry.createByPoint(
            ground.component.originConstructionPoint.createForAssemblyContext(ground))
    except Exception:
        g = None
    if g is None:
        g = adsk.fusion.JointGeometry.createByPoint(root.originConstructionPoint)
    return g


def _slider(root, ground, fol, swap):
    abj = root.asBuiltJoints
    one, two = (ground, fol) if swap else (fol, ground)
    ji = abj.createInput(one, two, _origin_geometry(root, ground))
    ji.setAsSliderJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
    j = abj.add(ji)
    j.name = 'Follower slides'
    return j


def _slides_up(j, fol):
    """True when a positive slide moves the follower along +y (tried with 1 mm, then put back)."""
    j.jointMotion.slideValue = 0.1
    y = fol.transform2.translation.y
    j.jointMotion.slideValue = 0.0
    return y > -0.05


def build(R_b=25.0, R_r=6.0, h=10.0, beta1=120.0, dwell=60.0, beta2=120.0):
    """Builds the cam and follower in a new design and returns the message to show. Raises ValueError (before any
    document is made) when the design is refused."""
    c = cam(R_b, R_r, h, beta1, dwell, beta2)
    refuse(c)
    des = _new_design()
    root = des.rootComponent
    R_p = c['R_p']
    z0, z1 = Z_PART

    ground = P.new_occ(root, 'Ground')
    cam_occ = P.new_occ(root, 'Cam')
    fol = P.new_occ(root, 'Follower')
    ground.isGrounded = True

    # Ground: a plate under the cam and the follower's whole stroke
    reach = c['r_max'] + PLATE_MARGIN
    P.add_body(ground, P.box(-reach, reach, -reach, max(reach, R_p + h + STEM_L + PLATE_MARGIN), *Z_PLATE), 'Plate')

    # Cam
    _cam_body(cam_occ, c)

    # Follower at theta = 0 (s = 0): the roller round (0, R_p) and the stem up +y, one body
    roller = P.cyl((0.0, R_p, z0), (0.0, R_p, z1), R_r)
    stem = P.box(-STEM_W / 2.0, STEM_W / 2.0, R_p, R_p + STEM_L, z0, z1)
    P.add_body(fol, P.union(roller, stem), 'Roller and stem')

    # As-built joints: the cam turns about z on Ground, the follower slides along y on Ground; no Motion Link
    abj = root.asBuiltJoints
    ji = abj.createInput(cam_occ, ground, _origin_geometry(root, ground))
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    jc = abj.add(ji)
    jc.name = 'Cam turns'
    jf = _slider(root, ground, fol, swap=False)
    if not _slides_up(jf, fol):
        jf.deleteMe()
        jf = _slider(root, ground, fol, swap=True)
        if not _slides_up(jf, fol):
            raise RuntimeError('the follower slider does not move the follower along +y')

    try:
        adsk.core.Application.get().activeViewport.fit()
    except Exception:
        pass

    i = c['inputs']
    rest = 360.0 - (i['beta1'] + i['dwell'] + i['beta2'])
    return ('%s\n'
            'Disc cam with a translating roller follower.\n'
            'Largest pressure angle: %.2f deg (at theta = %.1f deg), limit %g deg.\n'
            "Smallest radius of curvature of the pitch curve: %.2f mm (at theta = %.1f deg), roller radius %g mm.\n"
            'R_b = %g mm, R_r = %g mm, R_p = %g mm, lift h = %g mm.\n'
            'Cycloidal rise %g deg, dwell %g deg, cycloidal return %g deg, dwell %g deg.\n'
            "Turn 'Cam turns' and set 'Follower slides' to s(theta): there is no Motion Link."
            % (VERSION, c['phi'], c['phi_at'], PHI_LIMIT, c['rho'], c['rho_at'], i['R_r'],
               i['R_b'], i['R_r'], R_p, i['h'], i['beta1'], i['dwell'], i['beta2'], rest))


# ---------------------------------------------------------------------------------------------------------------
# The dialog. A document cannot be made inside a command's events, so OK fires a custom event and build() runs
# from there.
# ---------------------------------------------------------------------------------------------------------------
_handlers = []
_state = {'standalone': False, 'executed': False}

_INPUTS = (('R_b', 'Base circle radius R_b', 'mm'),
           ('R_r', 'Roller radius R_r', 'mm'),
           ('h', 'Lift h', 'mm'),
           ('beta1', 'Rise angle beta1', 'deg'),
           ('dwell', 'Dwell at the top', 'deg'),
           ('beta2', 'Return angle beta2', 'deg'))


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            for key, label, unit in _INPUTS:
                ins.addValueInput(key, label, unit, adsk.core.ValueInput.createByString('%g %s' % (DEFAULTS[key], unit)))
            h = _Execute()
            cmd.execute.add(h)
            _handlers.append(h)
            h = _Destroy()
            cmd.destroy.add(h)
            _handlers.append(h)
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('%s: the dialog failed\n%s' % (VERSION, traceback.format_exc()))


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.command.commandInputs
            v = {}
            for key, _, unit in _INPUTS:
                val = ins.itemById(key).value                 # cm or radians
                v[key] = val * 10.0 if unit == 'mm' else math.degrees(val)
            app = adsk.core.Application.get()
            if not app.fireCustomEvent(EVENT_ID, json.dumps(v)):      # handled after this event returns
                raise RuntimeError('the build event could not be fired')
            _state['executed'] = True
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('%s failed\n%s' % (VERSION, traceback.format_exc()))


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _state['standalone'] and not _state['executed']:
            adsk.terminate()


class _Build(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            v = json.loads(args.additionalInfo)
            msg = build(**v)
            ui.messageBox(msg)
        except ValueError as e:
            ui.messageBox('%s\nRefused: %s' % (VERSION, e))
        except Exception:
            ui.messageBox('%s failed\n%s' % (VERSION, traceback.format_exc()))
        finally:
            if _state['standalone']:
                adsk.terminate()


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        _state['standalone'] = context is not None       # Fusion passes a context; a check calling run(None) does not
        _state['executed'] = False
        try:
            app.unregisterCustomEvent(EVENT_ID)
        except Exception:
            pass
        evt = app.registerCustomEvent(EVENT_ID)
        h = _Build()
        evt.add(h)
        _handlers.append(h)
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Disc cam and roller follower',
                                                        'Builds a disc cam with a translating roller follower')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if _state['standalone']:
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('%s failed\n%s' % (VERSION, traceback.format_exc()))


def stop(context):
    try:
        app = adsk.core.Application.get()
        app.unregisterCustomEvent(EVENT_ID)
        cd = app.userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
