# -*- coding: utf-8 -*-
"""KeyedShaft.py: a shaft turning in a bearing block, with a hub fixed to it by a DIN 6885 parallel key.

Run it from Scripts and Add-Ins. A dialog asks for the shaft, the hub, the key and the duty; OK checks the key
against the DIN 6885 table and the two allowable stresses, then builds the assembly in a new design document:

    Bearing block   grounded, square round the shaft, z = 0 to 20 mm, bore D + 0.2
    Shaft           along z from z = 0 to L_shaft, a keyseat b wide and t1 deep on its +y side
    Key             DIN 6885 form A (round ends), b x h x L_key, t1 of its height in the shaft
    Hub             bore D + 0.1, outside diameter D + 2 t_wall, its top face 5 mm below the shaft's end

As-built Joints: "Shaft turns" (revolute about z, shaft on the bearing block), "Key on shaft" and "Hub on shaft"
(rigid). Drive Joints on "Shaft turns" turns shaft, key and hub together.

Every size below is in millimetres; the Fusion API works in centimetres, so every length is multiplied by CM
on its way into the API.
"""
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

VERSION = 'KeyedShaft v1.0 (2026-11-10)'
CMD_ID = 'mdfcKeyedShaftCmd'
CM = 0.1                                   # mm -> cm


# ---------- the companion: walk up to the folder that holds common/companion_paths.py ----------
def _companion_common():
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up


_COMMON = _companion_common()
if _COMMON and _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
try:
    import companion_paths as CP           # puts every lib/ folder on sys.path; this script needs none of them
except ImportError:
    CP = None

app = adsk.core.Application.get()
ui = app.userInterface
_handlers = []
_own_script = False                        # True when Fusion itself ran this script (it then keeps it alive)


# ---------- the inputs of the dialog: id, label, unit ('mm', or '' for a plain number), default ----------
INPUTS = (
    ('D', 'Shaft diameter D', 'mm', 20.0),
    ('L_shaft', 'Shaft length L_shaft', 'mm', 80.0),
    ('L_hub', 'Hub length L_hub', 'mm', 25.0),
    ('t_wall', 'Hub wall t_wall', 'mm', 8.0),
    ('L_key', 'Key length L_key', 'mm', 18.0),
    ('T', 'Torque to transmit T (N.m)', '', 30.0),
    ('tau_allow', 'Allowable shear stress in the key tau_allow (MPa)', '', 60.0),
    ('sigma_allow', "Allowable crushing stress on the key's sides sigma_allow (MPa)", '', 100.0),
)

# DIN 6885 parallel keys: D above lo up to hi -> b, h, t1 (shaft), t2 (hub), all mm
DIN6885 = (
    (10.0, 12.0, 4.0, 4.0, 2.5, 1.8),
    (12.0, 17.0, 5.0, 5.0, 3.0, 2.3),
    (17.0, 22.0, 6.0, 6.0, 3.5, 2.8),
    (22.0, 30.0, 8.0, 7.0, 4.0, 3.3),
    (30.0, 38.0, 10.0, 8.0, 5.0, 3.3),
    (38.0, 44.0, 12.0, 8.0, 5.0, 3.3),
    (44.0, 50.0, 14.0, 9.0, 5.5, 3.8),
    (50.0, 58.0, 16.0, 10.0, 6.0, 4.3),
)

BLOCK_H = 20.0              # the bearing block runs from z = 0 to 20 mm
HUB_BELOW_END = 5.0         # the hub's top face is 5 mm below the shaft's end
BORE_GAP_HUB = 0.1          # hub bore D + 0.1
BORE_GAP_BLOCK = 0.2        # bearing block bore D + 0.2
SLOT_EXTRA = 1.0            # the shaft's slot is L_key + 1 long


# ---------- the design calculation ----------
def key_size(D):
    """b, h, t1, t2 of the DIN 6885 key for a shaft of diameter D (mm)."""
    for lo, hi, b, h, t1, t2 in DIN6885:
        if lo < D <= hi:
            return b, h, t1, t2
    raise ValueError('D = %g mm is outside the DIN 6885 table (above 10 mm up to 58 mm)' % D)


def design_values(D, L_shaft, L_hub, t_wall, L_key, T, tau_allow, sigma_allow):
    """Everything derived from the inputs, or ValueError saying why the inputs are refused."""
    for name, val in (('L_shaft', L_shaft), ('L_hub', L_hub), ('t_wall', t_wall), ('L_key', L_key), ('T', T),
                      ('tau_allow', tau_allow), ('sigma_allow', sigma_allow)):
        if not val > 0:
            raise ValueError('%s must be more than 0 (it is %g)' % (name, val))
    b, h, t1, t2 = key_size(D)
    if L_key > L_hub:
        raise ValueError('the key (L_key = %g mm) is longer than the hub (L_hub = %g mm): it would stick out of '
                         'the hub' % (L_key, L_hub))

    T_Nmm = 1000.0 * T                               # N.m -> N.mm
    F = 2.0 * T_Nmm / D                              # N, at the shaft's surface
    tau = F / (b * L_key)                            # MPa, shear across the key
    sigma = F / ((h - t1) * L_key)                   # MPa, crushing on the hub side
    tau_shaft = 16.0 * T_Nmm / (math.pi * D ** 3)    # MPa, torsion in the shaft
    over = []
    if tau > tau_allow:
        over.append('the shear stress %.1f MPa in the key is above tau_allow = %g MPa by %.1f MPa'
                    % (tau, tau_allow, tau - tau_allow))
    if sigma > sigma_allow:
        over.append("the crushing stress %.1f MPa on the key's sides is above sigma_allow = %g MPa by %.1f MPa"
                    % (sigma, sigma_allow, sigma - sigma_allow))
    if over:
        raise ValueError('the key is overloaded at T = %g N.m (F = %.0f N): %s' % (T, F, '; '.join(over)))

    # the layout along z
    z_hub1 = L_shaft - HUB_BELOW_END
    z_hub0 = z_hub1 - L_hub
    z_mid = (z_hub0 + z_hub1) / 2.0
    hub_od = D + 2.0 * t_wall
    if L_key <= b:
        raise ValueError('a form A key needs L_key longer than its width b = %g mm (L_key = %g mm)' % (b, L_key))
    if z_hub0 < BLOCK_H:
        raise ValueError('the hub (z = %g to %g mm) would reach into the bearing block (z = 0 to %g mm): make '
                         'L_shaft longer or L_hub shorter' % (z_hub0, z_hub1, BLOCK_H))
    corner = math.hypot(b / 2.0, D / 2.0 + t2)       # the far corner of the hub's slot
    if corner >= hub_od / 2.0:
        raise ValueError('the hub wall t_wall = %g mm is too thin: the key slot (to radius %.2f mm) would break '
                         'through the hub (radius %g mm)' % (t_wall, corner, hub_od / 2.0))
    return dict(D=D, L_shaft=L_shaft, L_hub=L_hub, t_wall=t_wall, L_key=L_key, T=T, tau_allow=tau_allow,
                sigma_allow=sigma_allow, b=b, h=h, t1=t1, t2=t2, F=F, tau=tau, sigma=sigma, tau_shaft=tau_shaft,
                hub_od=hub_od, hub_bore=D + BORE_GAP_HUB, block_a=max(40.0, D + 20.0),
                block_bore=D + BORE_GAP_BLOCK, z_hub0=z_hub0, z_hub1=z_hub1,
                z_key0=z_mid - L_key / 2.0, z_key1=z_mid + L_key / 2.0,
                z_slot0=z_mid - (L_key + SLOT_EXTRA) / 2.0, z_slot1=z_mid + (L_key + SLOT_EXTRA) / 2.0)


def message(v):
    return '\n'.join((
        'Keyed shaft built, %s' % VERSION,
        'D = %g mm, shaft %g mm long, hub %g mm long with a %g mm wall, key %g mm long, T = %g N.m'
        % (v['D'], v['L_shaft'], v['L_hub'], v['t_wall'], v['L_key'], v['T']),
        'DIN 6885 key b x h = %g x %g mm, t1 = %g mm (shaft), t2 = %g mm (hub); hub bore %g mm, hub outside '
        'diameter %g mm' % (v['b'], v['h'], v['t1'], v['t2'], v['hub_bore'], v['hub_od']),
        'F = 2T/D = %.0f N' % v['F'],
        'key shear stress tau = F/(b L_key) = %.2f MPa (tau_allow %g MPa)' % (v['tau'], v['tau_allow']),
        'crushing stress on the hub side sigma = F/((h - t1) L_key) = %.2f MPa (sigma_allow %g MPa)'
        % (v['sigma'], v['sigma_allow']),
        'shaft torsion stress tau_shaft = 16T/(pi D^3) = %.2f MPa' % v['tau_shaft'],
    ))


# ---------- temporary bodies, given in world millimetres ----------
def _tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def _p(x, y, z):
    return adsk.core.Point3D.create(x * CM, y * CM, z * CM)


def _cyl(p0, p1, r):
    return _tbm().createCylinderOrCone(_p(*p0), r * CM, _p(*p1), r * CM)


def _box(x0, x1, y0, y1, z0, z1):
    c = _p((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)
    obb = adsk.core.OrientedBoundingBox3D.create(c, adsk.core.Vector3D.create(1, 0, 0),
                                                 adsk.core.Vector3D.create(0, 1, 0),
                                                 (x1 - x0) * CM, (y1 - y0) * CM, (z1 - z0) * CM)
    return _tbm().createBox(obb)


def _bool(a, b, kind):
    if not _tbm().booleanOperation(a, b, kind):
        raise RuntimeError('a boolean of temporary bodies failed')
    return a


def _union(a, b):
    return _bool(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)


def _cut(a, b):
    return _bool(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)


def _form_a(b, y0, y1, z0, z1):
    """A form A shape (a key, or the cutter of its keyseat): b wide in x, from y0 to y1, from z0 to z1 along z,
    round ends of radius b/2."""
    r = b / 2.0
    s = _box(-r, r, y0, y1, z0 + r, z1 - r)
    _union(s, _cyl((0.0, y0, z0 + r), (0.0, y1, z0 + r), r))
    _union(s, _cyl((0.0, y0, z1 - r), (0.0, y1, z1 - r), r))
    return s


def block_body(v):
    a = v['block_a'] / 2.0
    blk = _box(-a, a, -a, a, 0.0, BLOCK_H)
    return _cut(blk, _cyl((0.0, 0.0, -1.0), (0.0, 0.0, BLOCK_H + 1.0), v['block_bore'] / 2.0))


def shaft_body(v):
    R = v['D'] / 2.0
    s = _cyl((0.0, 0.0, 0.0), (0.0, 0.0, v['L_shaft']), R)
    slot = _form_a(v['b'], R - v['t1'], R + 1.0, v['z_slot0'], v['z_slot1'])     # open above the shaft's surface
    return _cut(s, slot)


def key_body(v):
    y0 = v['D'] / 2.0 - v['t1']                     # t1 of its height in the shaft's slot, the rest in the hub's
    return _form_a(v['b'], y0, y0 + v['h'], v['z_key0'], v['z_key1'])


def hub_body(v):
    z0, z1 = v['z_hub0'], v['z_hub1']
    hub = _cyl((0.0, 0.0, z0), (0.0, 0.0, z1), v['hub_od'] / 2.0)
    _cut(hub, _cyl((0.0, 0.0, z0 - 1.0), (0.0, 0.0, z1 + 1.0), v['hub_bore'] / 2.0))
    r = v['b'] / 2.0
    _cut(hub, _box(-r, r, 0.0, v['D'] / 2.0 + v['t2'], z0 - 1.0, z1 + 1.0))       # t2 above the shaft's surface
    return hub


# ---------- components, bodies and joints ----------
def _new_occ(root, name):
    """A new component at the world origin: its body is made where it belongs, so it is never moved."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def _add_body(occ, tb, name):
    m = occ.transform2.copy()                       # the identity here; kept so the body lands right regardless
    m.invert()
    _tbm().transform(tb, m)
    comp = occ.component
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    try:
        comp.bRepBodies.add(tb, bf)
    finally:
        bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def _axis_geometry(root, occ_shaft, D):
    """Joint geometry on the shaft's axis: the centre of the circular edge at its end z = 0 (z along the axis)."""
    body = occ_shaft.component.bRepBodies.item(0)
    R = D / 2.0 * CM
    for e in body.edges:
        c = adsk.core.Circle3D.cast(e.geometry) or adsk.core.Arc3D.cast(e.geometry)
        if c is None:
            continue
        p = c.center
        if abs(p.x) < 1e-6 and abs(p.y) < 1e-6 and abs(p.z) < 1e-6 and abs(c.radius - R) < 1e-6:
            return adsk.fusion.JointGeometry.createByCurve(e.createForAssemblyContext(occ_shaft),
                                                           adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    return adsk.fusion.JointGeometry.createByPoint(root.originConstructionPoint)


def _as_built(root, one, two, geo, name, revolute):
    abj = root.asBuiltJoints
    inp = abj.createInput(one, two, geo)
    if revolute:
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = abj.add(inp)
    j.name = name
    return j


def _new_design():
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    if des is None:
        des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:                                            # Fusion versions with design intent: allow components
        des.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
    except Exception:
        pass
    return des


def build(D=20.0, L_shaft=80.0, L_hub=25.0, t_wall=8.0, L_key=18.0, T=30.0, tau_allow=60.0, sigma_allow=100.0):
    """Check the inputs (ValueError if refused), build the assembly in a new design and return the message."""
    v = design_values(float(D), float(L_shaft), float(L_hub), float(t_wall), float(L_key), float(T),
                      float(tau_allow), float(sigma_allow))
    des = _new_design()
    root = des.rootComponent

    block = _new_occ(root, 'Bearing block')
    _add_body(block, block_body(v), 'Bearing block')
    shaft = _new_occ(root, 'Shaft')
    _add_body(shaft, shaft_body(v), 'Shaft')
    key = _new_occ(root, 'Key')
    _add_body(key, key_body(v), 'Key')
    hub = _new_occ(root, 'Hub')
    _add_body(hub, hub_body(v), 'Hub')
    block.isGrounded = True

    _as_built(root, shaft, block, _axis_geometry(root, shaft, v['D']), 'Shaft turns', True)
    _as_built(root, key, shaft, _axis_geometry(root, shaft, v['D']), 'Key on shaft', False)
    _as_built(root, hub, shaft, _axis_geometry(root, shaft, v['D']), 'Hub on shaft', False)
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return message(v)


# ---------- the dialog ----------
class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.command.commandInputs
            vals = {}
            for key, _label, unit, _default in INPUTS:
                ci = ins.itemById(key)
                vals[key] = ci.value * 10.0 if unit == 'mm' else ci.value       # a length input holds cm
            ui.messageBox(build(**vals), 'Keyed shaft')
        except ValueError as e:
            ui.messageBox('Refused (%s):\n%s' % (VERSION, e), 'Keyed shaft')
        except Exception:
            ui.messageBox('KeyedShaft failed (%s):\n%s' % (VERSION, traceback.format_exc()), 'Keyed shaft')


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _own_script:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            for key, label, unit, default in INPUTS:
                if unit == 'mm':
                    ins.addValueInput(key, label, 'mm', adsk.core.ValueInput.createByString('%g mm' % default))
                else:
                    ins.addValueInput(key, label, '', adsk.core.ValueInput.createByReal(default))
            ins.addTextBoxCommandInput('derived', '', 'b, h, t1, t2, the hub and the three stresses are worked '
                                       'out from these inputs (DIN 6885).', 2, True)
            on_execute = _Execute()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui.messageBox('KeyedShaft dialog failed:\n%s' % traceback.format_exc())


def run(context):
    global _own_script
    try:
        _own_script = context is not None            # Fusion passes a context; another script calling run() may not
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Keyed shaft',
                                                       'A shaft in a bearing block with a hub on a DIN 6885 key')
        on_created = _Created()                      # looked up when run() runs, so a subclass put here is used
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        if _own_script:
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('KeyedShaft failed:\n%s' % traceback.format_exc())


def stop(context):
    try:
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
