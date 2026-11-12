# =============================================================================
# Geneva.py  -  an external Geneva drive: a driver with a pin and a locking disc, and a slotted wheel that the pin
# indexes by 2 pi / n each turn and the locking disc then holds still. A dialog takes the number of slots n, the
# pin circle radius a and the pin diameter; everything else is worked out from them. A message gives the centre
# distance, the wheel radius, the indexing and dwell angles of the driver and the wheel's peak speed ratio.
#
# Written from projects/geneva/Geneva/PROMPT.txt alone (prompt test).
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'Geneva prompt-test 1.0 (2026-11-11)'
CMD_ID = 'mdfc_prompt_test_geneva'
CM = 0.1                       # the API works in cm: mm * CM = cm
GAP = 0.2                      # running gap everywhere (mm)

_handlers = []                 # Fusion drops event handlers that nothing holds
_keep_alive = [False]          # True when Fusion ran the script and it must stay alive until the dialog closes
_last = {}                     # the design numbers of the last drive built, for set_state()


# ---------- the design calculations (mm and radians) ----------
def geneva(n=4, a=30.0, dp=8.0):
    """Every derived size of the drive, or ValueError when the inputs are refused."""
    if n != int(n) or not 3 <= int(n) <= 12:
        raise ValueError('number of slots n = %s: n must be a whole number from 3 to 12' % n)
    n = int(n)
    if a <= 0:
        raise ValueError('pin circle radius a = %g mm: it must be greater than 0' % a)
    if dp <= 0:
        raise ValueError('pin diameter %g mm: it must be greater than 0' % dp)
    if dp > a / 3.0 + 1e-9:
        raise ValueError('pin diameter %g mm is too large for a = %g mm: at most a / 3 = %.3f mm'
                         % (dp, a, a / 3.0))
    C = a / math.sin(math.pi / n)              # the pin enters and leaves each slot along it
    b = math.sqrt(C * C - a * a)               # wheel radius
    alpha = math.pi / 2 - math.pi / n          # half the driver's turn while the pin is in a slot
    return dict(n=n, a=a, dp=dp, C=C, b=b, alpha=alpha,
                index=2 * alpha,                       # driver turn while indexing
                dwell=2 * math.pi - 2 * alpha,         # driver turn while the wheel stands still
                step=2 * math.pi / n,                  # wheel turn per index (the other way)
                peak=a / (C - a),                      # wheel speed over driver speed at mid-index
                slot_w=dp + 2 * GAP,                   # slots 0.4 mm wider than the pin
                r_bottom=C - a - dp / 2.0 - 1.0,       # slot bottom, 1 mm inside the pin's closest approach
                r_lock=a - dp,                         # locking disc radius
                r_arc=a - dp + GAP,                    # the wheel's locking arcs, round the driver's centre
                r_relief=b + GAP)                      # the locking disc's relief, round the point C towards the pin


def _gamma(g, theta):
    """Direction from the wheel's centre to the pin, the driver at theta from the line of centres."""
    a, C = g['a'], g['C']
    return math.atan2(a * math.sin(theta), a * math.cos(theta) - C)


def _wrap(x):
    """An angle wrapped into -pi to pi."""
    return (x + math.pi) % (2 * math.pi) - math.pi


def wheel_turn(g, phi):
    """The wheel's turn (rad) for a driver turn phi (rad) from the built position (the pin at theta = pi)."""
    whole = math.floor(phi / (2 * math.pi))
    rem = phi - whole * 2 * math.pi
    theta = math.pi + rem
    al = g['alpha']
    if theta < 2 * math.pi - al:
        part = 0.0                                             # dwell before the index
    elif theta > 2 * math.pi + al:
        part = -g['step']                                      # dwell after it
    else:
        part = _wrap(_gamma(g, theta - 2 * math.pi) - _gamma(g, -al))
    return whole * -g['step'] + part


# ---------- temporary bodies, world millimetres ----------
def _tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def _p(x, y, z):
    return adsk.core.Point3D.create(x * CM, y * CM, z * CM)


def _cyl(x, y, z0, z1, r):
    """A cylinder about the vertical through (x, y), from z0 to z1, radius r."""
    return _tbm().createCylinderOrCone(_p(x, y, z0), r * CM, _p(x, y, z1), r * CM)


def _bar(cx, cy, ang, length, width, z0, z1):
    """A box centred on (cx, cy), its length along the angle ang in the xy plane."""
    obb = adsk.core.OrientedBoundingBox3D.create(
        _p(cx, cy, (z0 + z1) / 2.0),
        adsk.core.Vector3D.create(math.cos(ang), math.sin(ang), 0.0),
        adsk.core.Vector3D.create(-math.sin(ang), math.cos(ang), 0.0),
        length * CM, width * CM, (z1 - z0) * CM)
    return _tbm().createBox(obb)


def _boolean(target, tool, kind):
    if not _tbm().booleanOperation(target, tool, kind):
        raise RuntimeError('a boolean of temporary bodies failed')
    return target


def _union(a, b):
    return _boolean(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)


def _cut(a, b):
    return _boolean(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)


def _plate(g):
    """Ground: a plate from z = -6 to 0 under the driver and the wheel, 10 mm beyond both."""
    x0 = -(g['a'] + 6.0) - 10.0
    x1 = g['C'] + g['b'] + 10.0
    hy = max(g['a'] + 6.0, g['b']) + 10.0
    return _bar((x0 + x1) / 2.0, 0.0, 0.0, x1 - x0, 2 * hy, -6.0, 0.0)


def _driver_body(g):
    """Crank disc (z 0 to 3.5), pin (z 0 to 10) and locking disc (z 3.5 to 10), built with the pin at theta = pi."""
    a, C = g['a'], g['C']
    crank = _cyl(0.0, 0.0, 0.0, 3.5, a + 6.0)
    lock = _cyl(0.0, 0.0, 3.0, 10.0, g['r_lock'])        # its part below z = 3.5 lies inside the crank disc
    _cut(lock, _cyl(-C, 0.0, 2.0, 11.0, g['r_relief']))  # the relief, round the point C from the centre towards the pin
    pin = _cyl(-a, 0.0, 0.0, 10.0, g['dp'] / 2.0)
    _union(crank, lock)
    _union(crank, pin)
    return crank


def _wheel_body(g):
    """The wheel, z 4 to 10, centred at (C, 0), at dwell: a slot pointing at pi + pi / n, n slots with round
    bottoms and n locking arcs round the driver's centre."""
    n, C, b, w = g['n'], g['C'], g['b'], g['slot_w']
    wheel = _cyl(C, 0.0, 4.0, 10.0, b)
    rc = g['r_bottom'] + w / 2.0                         # centre of a slot's round bottom, from the wheel's centre
    r_out = b + 2.0                                      # the slot runs out past the rim
    for k in range(n):
        ph = math.pi + math.pi / n + 2 * math.pi * k / n
        mid = (rc + r_out) / 2.0
        _cut(wheel, _bar(C + mid * math.cos(ph), mid * math.sin(ph), ph, r_out - rc, w, 3.0, 11.0))
        _cut(wheel, _cyl(C + rc * math.cos(ph), rc * math.sin(ph), 3.0, 11.0, w / 2.0))
        q = math.pi + 2 * math.pi * k / n                # between two slots; k = 0 faces the driver
        _cut(wheel, _cyl(C + C * math.cos(q), C * math.sin(q), 3.0, 11.0, g['r_arc']))
    return wheel


# ---------- components, bodies and joints ----------
def _component(root, name):
    """A new component at the origin."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def _add_body(occ, tb, name):
    """A temporary body (world coordinates) into the component; it sits at the origin, so nothing moves."""
    comp = occ.component
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def _lumps(tb):
    try:
        return tb.lumps.count
    except Exception:
        return 1


def _sense(joint, occ):
    """sin of the turn the occurrence makes for +0.1 rad on the joint: > 0 when it turns counter-clockwise
    about +z. The joint goes back to 0 (the built position) afterwards."""
    jm = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    try:
        jm.rotationValue = 0.1
        return occ.transform2.getCell(1, 0)
    except Exception:
        return 0.0
    finally:
        try:
            jm.rotationValue = 0.0
        except Exception:
            pass


def _revolute(root, moving, ground, point, name):
    """An As-built revolute joint about z through a sketch point of Ground; a positive value turns the moving part
    counter-clockwise about +z (if it came out the other way, the joint is made again with its parts swapped)."""
    pt = point.createForAssemblyContext(ground)
    for one, two in ((moving, ground), (ground, moving)):
        ji = root.asBuiltJoints.createInput(one, two, adsk.fusion.JointGeometry.createByPoint(pt))
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        if _sense(j, moving) >= 0.0:
            j.name = name
            return j
        j.deleteMe()
    raise RuntimeError('the joint %s turns clockwise whichever way round it is made' % name)


def _report(g, lumps):
    lines = ['External Geneva drive built (%s)' % VERSION,
             'n = %d slots, pin circle radius a = %.3f mm, pin diameter %.3f mm' % (g['n'], g['a'], g['dp']),
             'centre distance C = %.3f mm, wheel radius b = %.3f mm' % (g['C'], g['b']),
             'indexing: %.1f deg of driver turn; dwell: %.1f deg' % (math.degrees(g['index']),
                                                                    math.degrees(g['dwell'])),
             'each index turns the wheel %.1f deg (the other way)' % -math.degrees(g['step']),
             'peak speed ratio (wheel over driver, mid-index) = %.3f' % g['peak'],
             'No Motion Link: turn "Driver turns", then set "Wheel turns" by the Geneva law (set_state does both).']
    if lumps > 1:
        lines.append('Warning: the slot bottoms are %.2f mm from the wheel\'s centre, so the slots meet there and '
                     'the wheel is in %d pieces.' % (g['r_bottom'], lumps))
    return '\n'.join(lines)


def build(n=4, a=30.0, dp=8.0):
    """Build the drive in a new design (sizes in mm); returns the report, or raises ValueError when refused."""
    g = geneva(n, a, dp)                                 # refuse before any document is made
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent

    ground = _component(root, 'Ground')
    driver = _component(root, 'Driver')
    wheel = _component(root, 'Wheel')
    ground.isGrounded = True

    _add_body(ground, _plate(g), 'Plate')
    _add_body(driver, _driver_body(g), 'Driver')
    wb = _wheel_body(g)
    lumps = _lumps(wb)
    _add_body(wheel, wb, 'Wheel')

    gc = ground.component
    sk = gc.sketches.add(gc.xYConstructionPlane)
    sk.name = 'Joint centres'
    p_driver = sk.sketchPoints.add(sk.modelToSketchSpace(_p(0.0, 0.0, 0.0)))
    p_wheel = sk.sketchPoints.add(sk.modelToSketchSpace(_p(g['C'], 0.0, 0.0)))
    _revolute(root, driver, ground, p_driver, 'Driver turns')
    _revolute(root, wheel, ground, p_wheel, 'Wheel turns')

    _last.clear()
    _last.update(g)
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return _report(g, lumps)


def set_state(phi_deg):
    """Turn the driver phi_deg from the built position and set the wheel by the Geneva law (last drive built)."""
    if not _last:
        raise RuntimeError('build a drive first')
    root = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
    js = {j.name: j for j in root.asBuiltJoints}
    phi = math.radians(phi_deg)
    js['Driver turns'].jointMotion.rotationValue = phi
    js['Wheel turns'].jointMotion.rotationValue = wheel_turn(_last, phi)


# ---------- the dialog ----------
def _read(ins):
    """n, a (mm) and the pin diameter (mm) from the dialog; value inputs hold cm."""
    return (ins.itemById('n').value, ins.itemById('a').value * 10.0, ins.itemById('dp').value * 10.0)


def _derived_text(ins):
    try:
        g = geneva(*_read(ins))
        return ('C = %.3f mm, b = %.3f mm<br>indexing %.1f deg, dwell %.1f deg<br>peak speed ratio %.3f'
                % (g['C'], g['b'], math.degrees(g['index']), math.degrees(g['dwell']), g['peak']))
    except ValueError as e:
        return 'Refused: %s' % e
    except Exception:
        return ''


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            ins.addIntegerSpinnerCommandInput('n', 'Number of slots n', 1, 100, 1, 4)
            ins.addValueInput('a', 'Pin circle radius a', 'mm', adsk.core.ValueInput.createByString('30 mm'))
            ins.addValueInput('dp', 'Pin diameter', 'mm', adsk.core.ValueInput.createByString('8 mm'))
            ins.addTextBoxCommandInput('derived', 'Derived', _derived_text(ins), 3, True)
            for event, handler in ((cmd.execute, _Execute()), (cmd.inputChanged, _Changed()),
                                   (cmd.destroy, _Destroy())):
                event.add(handler)
                _handlers.append(handler)
        except Exception:
            adsk.core.Application.get().userInterface.messageBox('Failed:\n' + traceback.format_exc())


class _Changed(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.inputs
            box = ins.itemById('derived')
            if box is not None and args.input.id != 'derived':
                box.formattedText = _derived_text(ins)
        except Exception:
            pass


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            n, a, dp = _read(args.command.commandInputs)
            msg = build(n=n, a=a, dp=dp)
            ui.messageBox(msg, 'Geneva drive')
        except ValueError as e:
            ui.messageBox('Refused: %s' % e, 'Geneva drive')
        except Exception:
            ui.messageBox('Failed:\n' + traceback.format_exc(), 'Geneva drive')


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _keep_alive[0]:
            _keep_alive[0] = False
            adsk.terminate()


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        del _handlers[:]
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Geneva drive',
                                                       'Builds an external Geneva drive from n, a and the pin diameter')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if context is not None:                          # run by Fusion: stay alive until the dialog closes
            _keep_alive[0] = True
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('Failed:\n' + traceback.format_exc())


def stop(context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
