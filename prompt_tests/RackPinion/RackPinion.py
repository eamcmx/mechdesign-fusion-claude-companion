# -*- coding: utf-8 -*-
"""RackPinion.py: a rack and pinion on a base plate, built from a command dialog.

A spur pinion turns on a fixed shaft and a straight rack slides under it. As-built joints hold the shaft to the
plate (rigid), let the pinion turn on the shaft (revolute about z) and let the rack slide on the plate (slider along
x, limited to its travel). A Motion Link ties 360 deg of the pinion to pi m z of the rack, so Drive Joints on the
pinion moves the rack: anticlockwise seen from above moves the rack towards +x.

World frame (mm): the plate's top face is z = 0, the rack's pitch line is y = 0 with its teeth pointing +y, the
pinion's centre is (0, r) and its tooth 0 points straight down (-y) at a gap of the rack centred on x = 0. The
pinion and the rack run from z = 1 to z = 1 + b. The Fusion API works in cm and radians; every size here is in mm
and degrees until it is handed to the API.
"""
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


# ---------- the companion's libraries, found through common/companion_paths.py ----------
def _companion_root(start):
    """The first folder above `start` that holds common/companion_paths.py."""
    d = start
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % start)
        d = up


_COMMON = os.path.join(_companion_root(os.path.dirname(os.path.abspath(__file__))), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402,F401  (puts every lib/ on sys.path; this script needs no library)


VERSION = 'RackPinion prompt-test v1 (2026-11-12)'
CMD_ID = 'mdfcPromptTestRackPinion'
TITLE = 'Rack and pinion'

CM = 0.1                    # mm to the API's cm
Z0 = 1.0                    # the pinion and the rack start 1 mm above the plate's top face
BACKLASH = 0.2              # mm: each member's tooth is p/2 - 0.1 thick on its pitch line
RACK_BODY = 8.0             # mm of rack body below the roots
SHAFT_D = 8.0               # mm
BORE_CLEAR = 0.1            # mm on the diameter: the pinion's bore runs on the shaft
COLLAR_D = 14.0             # mm, the collar under the pinion
SHAFT_ABOVE = 2.0           # mm of shaft above the pinion's top face
PLATE_T = 6.0               # mm, the plate's thickness below z = 0
PLATE_MARGIN = 10.0         # mm of plate beyond the rack's travel and the pinion's tips

P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString


# ---------- design calculations (mm and degrees, no Fusion) ----------
def inv(a):
    """The involute function, a in radians."""
    return math.tan(a) - a


def rack_pinion(m, z, n_r, alpha_deg):
    """Every derived value of the prompt, in mm: r, p, per_turn, per_deg, eps (contact ratio) and limit (the travel
    either way), with the radii and the tooth thickness the geometry needs."""
    al = math.radians(alpha_deg)
    r = m * z / 2.0
    rb = r * math.cos(al)
    ra = r + m
    rf = r - 1.25 * m
    p = math.pi * m
    eps = (math.sqrt(ra * ra - rb * rb) - r * math.sin(al) + m / math.sin(al)) / (math.pi * m * math.cos(al))
    return dict(m=m, z=z, n_r=n_r, alpha=alpha_deg, al=al, r=r, rb=rb, ra=ra, rf=rf, p=p,
                per_turn=math.pi * m * z, per_deg=r * math.pi / 180.0, eps=eps,
                limit=(n_r / 2.0 - 2.0) * p, s=p / 2.0 - BACKLASH / 2.0)


def check_inputs(m, z, n_r, alpha, b):
    """The prompt's hard constraints first, then what the geometry needs to exist. Raises ValueError."""
    if int(z) != z or int(n_r) != n_r:
        raise ValueError('the tooth counts must be whole numbers (z = %s, N_r = %s)' % (z, n_r))
    if z < 18:
        raise ValueError('z = %d refused: a pinion of fewer than 18 teeth is undercut at a 20 deg pressure angle' % z)
    if n_r < 8:
        raise ValueError('N_r = %d refused: a rack of fewer than 8 teeth cannot travel and keep whole teeth in '
                         'mesh' % n_r)
    if m <= 0 or b <= 0:
        raise ValueError('the module and the face width must be positive (m = %s mm, b = %s mm)' % (m, b))
    if not 5.0 <= alpha <= 35.0:
        raise ValueError('pressure angle %s deg refused: give 5 to 35 deg' % alpha)


def tooth_numbers(k):
    """Widths the outlines need: the rack tooth's half width at its tip and root, the pinion tooth's half angle at
    the tip, and the radii of the tooth's foot and of the collar."""
    t = math.tan(k['al'])
    m = k['m']
    w_tip = k['s'] / 2.0 - m * t
    w_root = k['s'] / 2.0 + 1.25 * m * t
    beta = k['s'] / (2.0 * k['r'])                          # the pinion tooth's half angle on its pitch circle
    psi_tip = beta + inv(k['al']) - inv(math.acos(k['rb'] / k['ra']))
    r_in = k['rf'] - 0.25 * m                                # the tooth outline closes inside the root disc
    bore_r = (SHAFT_D + BORE_CLEAR) / 2.0
    collar_r = min(COLLAR_D / 2.0, k['r'] - m - 0.5)         # clear of the rack's underside at z = 1
    return dict(w_tip=w_tip, w_root=w_root, beta=beta, psi_tip=psi_tip, r_in=r_in, bore_r=bore_r,
                collar_r=collar_r)


def check_geometry(k):
    g = tooth_numbers(k)
    if g['w_tip'] <= 0.05:
        raise ValueError('the rack teeth come to a point at this module and pressure angle')
    if k['p'] - 2.0 * g['w_root'] <= 0.05:
        raise ValueError('the rack teeth meet at their roots at this module and pressure angle')
    if g['psi_tip'] <= 0.0:
        raise ValueError('the pinion teeth come to a point at this module and pressure angle')
    if g['r_in'] <= g['bore_r'] + 1.0 or g['collar_r'] <= g['bore_r'] + 0.5:
        raise ValueError('the pinion is too small for the %.0f mm shaft and its collar' % SHAFT_D)
    return g


def pinion_tooth(k, phase, n=12):
    """One closed outline (mm, the pinion's own frame, centre at the origin) of the tooth centred on the angle
    `phase`: involute flanks from the base circle (or the root, if that is larger) to the tip, radial below the base
    circle, a tip land on r + m, closed by a chord just inside the root circle so that it overlaps the root disc."""
    g = tooth_numbers(k)
    rb, ra, rf, al = k['rb'], k['ra'], k['rf'], k['al']
    r_lo = max(rb, rf)

    def psi(rho):                                            # the tooth's half angle at radius rho
        return g['beta'] + inv(al) - inv(math.acos(min(1.0, rb / rho)))

    radii = [r_lo + (ra - r_lo) * i / n for i in range(n + 1)]
    flank = [(rho, psi(rho)) for rho in radii]               # (radius, half angle) from the foot to the tip
    pol = [(g['r_in'], -flank[0][1])]
    pol += [(rho, -a) for rho, a in flank]
    a_tip = flank[-1][1]
    pol += [(ra, -a_tip + 2.0 * a_tip * j / 4.0) for j in range(1, 4)]
    pol += [(rho, a) for rho, a in reversed(flank)]
    pol += [(g['r_in'], flank[0][1])]
    return [(rho * math.cos(a + phase), rho * math.sin(a + phase)) for rho, a in pol]


def rack_outline(k):
    """The rack's whole outline (mm, the rack's own frame = the world at rest), teeth and body, as one closed
    polygon: N_r teeth, each in a pitch from one gap centre to the next, so that a gap is centred on x = 0."""
    g = tooth_numbers(k)
    m, p, n_r = k['m'], k['p'], k['n_r']
    y_tip, y_root = m, -1.25 * m
    y_bot = y_root - RACK_BODY
    k0 = -(n_r // 2)
    x_l, x_r = k0 * p, (k0 + n_r) * p
    pts = [(x_l, y_bot), (x_l, y_root)]
    for j in range(n_r):
        xc = (k0 + j + 0.5) * p
        pts += [(xc - g['w_root'], y_root), (xc - g['w_tip'], y_tip), (xc + g['w_tip'], y_tip),
                (xc + g['w_root'], y_root)]
    pts += [(x_r, y_root), (x_r, y_bot)]
    return pts, (x_l, x_r, y_bot)


# ---------- Fusion helpers ----------
def _new_occ(root, name, x, y, z):
    mat = adsk.core.Matrix3D.create()
    mat.translation = adsk.core.Vector3D.create(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def _sketch(comp, z_mm, name):
    """A sketch on a plane square to z at z_mm (the component's own frame)."""
    if abs(z_mm) < 1e-12:
        plane = comp.xYConstructionPlane
    else:
        ci = comp.constructionPlanes.createInput()
        ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
        plane = comp.constructionPlanes.add(ci)
    sk = comp.sketches.add(plane)
    sk.name = name
    return sk


def _pt(sk, x, y, z_mm):
    return sk.modelToSketchSpace(P3(x * CM, y * CM, z_mm * CM))


def _polyline(sk, pts, z_mm):
    """One closed polyline of sketch lines, each line starting on the SketchPoint the last one ended on."""
    lines = sk.sketchCurves.sketchLines
    q = [_pt(sk, x, y, z_mm) for x, y in pts]
    first = lines.addByTwoPoints(q[0], q[1])
    prev = first.endSketchPoint
    for p in q[2:]:
        prev = lines.addByTwoPoints(prev, p).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def _circle(sk, x, y, z_mm, rad):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(sk, x, y, z_mm), rad * CM)


def _profile(sk, loops=1):
    """The largest profile of the sketch with this many loops."""
    best, area = None, -1.0
    for pr in sk.profiles:
        if pr.profileLoops.count == loops:
            a = pr.areaProperties().area
            if a > area:
                best, area = pr, a
    if best is None:
        raise RuntimeError('sketch %s has no profile with %d loop(s)' % (sk.name, loops))
    return best


def _extrude(comp, sk, profile, height_mm, op, body=None):
    """Extrude towards +z of the component by height_mm, whichever way the sketch's normal points."""
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile, op)
    normal = sk.xDirection.crossProduct(sk.yDirection)
    inp.setDistanceExtent(False, VI(height_mm * CM * (1.0 if normal.z > 0 else -1.0)))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation


def _one_body(comp, name):
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s has %d bodies, not one' % (comp.name, comp.bRepBodies.count))
    body = comp.bRepBodies.item(0)
    body.name = name
    return body


# ---------- the four parts ----------
def _build_plate(occ, k, g):
    comp = occ.component
    pts, (x_l, x_r, y_bot) = rack_outline(k)
    x0, x1 = x_l - k['limit'] - PLATE_MARGIN, x_r + k['limit'] + PLATE_MARGIN
    y0, y1 = y_bot - PLATE_MARGIN, k['r'] + k['ra'] + PLATE_MARGIN
    sk = _sketch(comp, -PLATE_T, 'Plate outline')
    sk.sketchCurves.sketchLines.addTwoPointRectangle(_pt(sk, x0, y0, -PLATE_T), _pt(sk, x1, y1, -PLATE_T))
    _extrude(comp, sk, _profile(sk), PLATE_T, NEW)
    _one_body(comp, 'Base plate')


def _build_shaft(occ, k, g):
    """The shaft in its own frame (its axis is the component's z axis, at world (0, r)): 8 mm from the plate's top
    face to SHAFT_ABOVE over the pinion, with a collar from z = 0 to the pinion's underside at z = 1."""
    comp = occ.component
    sk = _sketch(comp, 0.0, 'Shaft')
    _circle(sk, 0.0, 0.0, 0.0, SHAFT_D / 2.0)
    _extrude(comp, sk, _profile(sk), Z0 + k['b'] + SHAFT_ABOVE, NEW)
    body = _one_body(comp, 'Shaft')
    sk2 = _sketch(comp, 0.0, 'Collar')
    _circle(sk2, 0.0, 0.0, 0.0, g['collar_r'])
    _extrude(comp, sk2, _profile(sk2), Z0, JOIN, body)
    _one_body(comp, 'Shaft')


def _build_pinion(occ, k, g):
    """The pinion in its own frame (centre at the origin = world (0, r)): the root disc with its bore, one tooth
    drawn as a closed polyline and joined to it, then a circular pattern of z teeth. Tooth 0 points along -y."""
    comp = occ.component
    sk = _sketch(comp, Z0, 'Root disc')
    root_circle = _circle(sk, 0.0, 0.0, Z0, k['rf'])
    _circle(sk, 0.0, 0.0, Z0, g['bore_r'])
    _extrude(comp, sk, _profile(sk, loops=2), k['b'], NEW)
    body = _one_body(comp, 'Pinion')
    sk2 = _sketch(comp, Z0, 'Tooth 0')
    _polyline(sk2, pinion_tooth(k, -math.pi / 2.0), Z0)
    tooth = _extrude(comp, sk2, _profile(sk2), k['b'], JOIN, body)
    feats = adsk.core.ObjectCollection.create()
    feats.add(tooth)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(k['z'])
    pi.totalAngle = VS('360 deg')
    pats.add(pi)
    _one_body(comp, 'Pinion')
    return root_circle.centerSketchPoint


def _build_rack(occ, k, g):
    comp = occ.component
    pts, _ = rack_outline(k)
    sk = _sketch(comp, Z0, 'Rack outline')
    _polyline(sk, pts, Z0)
    _extrude(comp, sk, _profile(sk), k['b'], NEW)
    _one_body(comp, 'Rack')
    sk2 = _sketch(comp, Z0, 'Rack slide point')
    return sk2.sketchPoints.add(_pt(sk2, 0.0, 0.0, Z0))


# ---------- joints ----------
def _direction(geo, want, default):
    """The JointDirections value whose axis of the joint geometry lies along the world axis `want`."""
    D = adsk.fusion.JointDirections
    try:
        for d, v in ((D.ZAxisJointDirection, geo.primaryAxisVector), (D.XAxisJointDirection, geo.secondaryAxisVector),
                     (D.YAxisJointDirection, geo.thirdAxisVector)):
            if v is not None and abs(abs(v.x * want[0] + v.y * want[1] + v.z * want[2]) - 1.0) < 1e-6:
                return d
    except Exception:
        pass
    return default


def _settle():
    adsk.doEvents()


def _turn_z_deg(occ):
    t = occ.transform2
    return math.degrees(math.atan2(t.getCell(1, 0), t.getCell(0, 0)))


def _drive_rot(j, deg):
    j.jointMotion.rotationValue = 0.0
    _settle()
    j.jointMotion.rotationValue = math.radians(deg)
    _settle()


def _drive_slide(j, mm):
    j.jointMotion.slideValue = 0.0
    _settle()
    j.jointMotion.slideValue = mm * CM
    _settle()


def _revolute(root, one, two, point, pinion):
    geo = adsk.fusion.JointGeometry.createByPoint(point.createForAssemblyContext(pinion))
    ji = root.asBuiltJoints.createInput(one, two, geo)
    ji.setAsRevoluteJointMotion(_direction(geo, (0, 0, 1), adsk.fusion.JointDirections.ZAxisJointDirection))
    j = root.asBuiltJoints.add(ji)
    j.name = 'Pinion turns'
    return j


def _slider(root, one, two, point, rack):
    geo = adsk.fusion.JointGeometry.createByPoint(point.createForAssemblyContext(rack))
    ji = root.asBuiltJoints.createInput(one, two, geo)
    ji.setAsSliderJointMotion(_direction(geo, (1, 0, 0), adsk.fusion.JointDirections.XAxisJointDirection))
    j = root.asBuiltJoints.add(ji)
    j.name = 'Rack slides'
    return j


def _joints(root, plate, shaft, pinion, rack, pin_pt, rack_pt, k):
    """Rigid, revolute and slider as-built joints and the Motion Link. Each moving joint is tried once: a positive
    value must turn the pinion anticlockwise about +z and slide the rack towards +x; if it goes the other way the
    joint is made again with its two occurrences swapped."""
    notes = []
    ji = root.asBuiltJoints.createInput(plate, shaft, None)
    ji.setAsRigidJointMotion()
    root.asBuiltJoints.add(ji).name = 'Shaft fixed'

    jp = _revolute(root, pinion, shaft, pin_pt, pinion)
    _drive_rot(jp, 90.0)
    turned = _turn_z_deg(pinion)
    _drive_rot(jp, 0.0)
    if turned < -45.0:
        jp.deleteMe()
        jp = _revolute(root, shaft, pinion, pin_pt, pinion)
        _drive_rot(jp, 90.0)
        turned = _turn_z_deg(pinion)
        _drive_rot(jp, 0.0)
        notes.append('revolute made shaft-first')
    if turned < 45.0:
        notes.append('pinion turn at +90 deg read as %.1f deg' % turned)

    jr = _slider(root, rack, plate, rack_pt, rack)
    _drive_slide(jr, 10.0)
    moved = rack.transform2.translation.x / CM
    _drive_slide(jr, 0.0)
    if moved < -5.0:
        jr.deleteMe()
        jr = _slider(root, plate, rack, rack_pt, rack)
        _drive_slide(jr, 10.0)
        moved = rack.transform2.translation.x / CM
        _drive_slide(jr, 0.0)
        notes.append('slider made plate-first')
    if moved < 5.0:
        notes.append('rack at +10 mm read as %.3f mm' % moved)

    lim = jr.jointMotion.slideLimits                 # each limit switched on before its value is set
    lim.isMinimumValueEnabled = True
    lim.minimumValue = -k['limit'] * CM
    lim.isMaximumValueEnabled = True
    lim.maximumValue = k['limit'] * CM

    mli = root.motionLinks.createInput(jp, jr)
    mli.valueOne = VS('360 deg')
    mli.valueTwo = VI(k['per_turn'] * CM)
    mli.isReversed = False
    ml = root.motionLinks.add(mli)
    ml.name = 'Pinion drives rack'
    _drive_rot(jp, 90.0)
    if jr.jointMotion.slideValue < 0:
        ml.isReversed = not ml.isReversed
        _drive_rot(jp, 90.0)
        notes.append('Motion Link reversed')
    s90 = jr.jointMotion.slideValue / CM
    x90 = rack.transform2.translation.x / CM
    a90 = _turn_z_deg(pinion)
    _drive_rot(jp, 0.0)
    return s90, x90, a90, notes


# ---------- the build ----------
def build(m=2.0, z=20, n_r=16, alpha=20.0, b=10.0):
    """Builds the rack and pinion in a new design and returns the message. Refuses (ValueError) before it opens a
    document if an input breaks a hard constraint."""
    check_inputs(m, z, n_r, alpha, b)
    z, n_r = int(z), int(n_r)
    k = rack_pinion(m, z, n_r, alpha)
    k['b'] = b
    g = check_geometry(k)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    root = des.rootComponent

    plate = _new_occ(root, 'Base plate', 0.0, 0.0, 0.0)
    shaft = _new_occ(root, 'Shaft', 0.0, k['r'], 0.0)
    pinion = _new_occ(root, 'Pinion', 0.0, k['r'], 0.0)
    rack = _new_occ(root, 'Rack', 0.0, 0.0, 0.0)
    _build_plate(plate, k, g)
    plate.isGrounded = True
    _build_shaft(shaft, k, g)
    pin_pt = _build_pinion(pinion, k, g)
    rack_pt = _build_rack(rack, k, g)

    s90, x90, a90, notes = _joints(root, plate, shaft, pinion, rack, pin_pt, rack_pt, k)
    try:
        app.activeViewport.fit()
    except Exception:
        pass

    lines = [
        'Rack and pinion built. %s' % VERSION,
        'module %g mm, pinion z = %d, rack N_r = %d, pressure angle %g deg, face width %g mm' % (m, z, n_r, alpha, b),
        'pinion: pitch radius r = %.3f mm, base %.3f, tip %.3f, root %.3f' % (k['r'], k['rb'], k['ra'], k['rf']),
        'rack: pitch p = %.3f mm, tooth %.3f mm thick on the pitch line (backlash %.1f mm)'
        % (k['p'], k['s'], BACKLASH),
        'travel per pinion turn = %.3f mm (%.4f mm per degree)' % (k['per_turn'], k['per_deg']),
        'contact ratio = %.3f' % k['eps'],
        'travel limits = -%.3f to +%.3f mm' % (k['limit'], k['limit']),
        'pinion driven 90 deg (turned %.1f deg): rack joint %+.3f mm, rack body %+.3f mm (want %+.3f)'
        % (a90, s90, x90, 90.0 * k['per_deg']),
    ]
    if notes:
        lines.append('notes: ' + '; '.join(notes))
    return '\n'.join(lines)


# ---------- the dialog ----------
_handlers = []
_state = {'auto': False}


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = dict(m=round(ins.itemById('m').value * 10.0, 9),
                        z=int(ins.itemById('z').value),
                        n_r=int(ins.itemById('n_r').value),
                        alpha=round(math.degrees(ins.itemById('alpha').value), 9),
                        b=round(ins.itemById('b').value * 10.0, 9))
            ui.messageBox(build(**vals), TITLE)
        except ValueError as e:
            ui.messageBox('Refused: %s\n%s' % (e, VERSION), TITLE)
        except Exception:
            ui.messageBox('Failed (%s):\n%s' % (VERSION, traceback.format_exc()), TITLE)


class _Destroy(adsk.core.CommandEventHandler):
    def notify(self, args):
        if _state['auto']:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('m', 'Module m', 'mm', VS('2 mm'))
        ins.addIntegerSpinnerCommandInput('z', 'Pinion teeth z', 1, 1000, 1, 20)
        ins.addIntegerSpinnerCommandInput('n_r', 'Rack teeth N_r', 1, 1000, 1, 16)
        ins.addValueInput('alpha', 'Pressure angle alpha', 'deg', VS('20 deg'))
        ins.addValueInput('b', 'Face width b', 'mm', VS('10 mm'))
        on_exec = _Execute()
        cmd.execute.add(on_exec)
        _handlers.append(on_exec)
        on_destroy = _Destroy()
        cmd.destroy.add(on_destroy)
        _handlers.append(on_destroy)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(CMD_ID, TITLE, 'A rack and pinion on a base plate')
        on_created = _Created()
        cdef.commandCreated.add(on_created)
        _handlers.append(on_created)
        cdef.execute()
        if context is not None:              # run by Fusion: stay loaded until the dialog closes
            adsk.autoTerminate(False)
            _state['auto'] = True
    except Exception:
        if ui:
            ui.messageBox('Failed (%s):\n%s' % (VERSION, traceback.format_exc()), TITLE)


def stop(context):
    try:
        old = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
    except Exception:
        pass
