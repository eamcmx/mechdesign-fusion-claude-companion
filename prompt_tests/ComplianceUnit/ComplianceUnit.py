# -*- coding: utf-8 -*-
"""ComplianceUnit.py: the palletizer's compliance unit (Volume 2, Chapter 48), written from its prompt alone.

A vertical float between the tool changer's ISO face and the vacuum or bag gripper. The housing (top plate, spigot,
two cheeks and the tube's boss, one block of 2024-T3) carries an HGR25 rail on each cheek; the carriage, whose
underside repeats the tool plate's, slides up on two HGH25CA blocks per rail when the load touches the stack. Four
springs press it down, by two lugs on each wing, onto four stop blocks; an M12 proximity sensor sees the +x wing rise.
The gripper's air crosses the float through a steel tube that slides in a sealed receiver on the carriage.

Every design value comes from projects/palletizer/lib/compliance_unit.py (LAY, GUIDE, AIR, ISO_B, PIN_B, design()).
World millimetres, z = 0 the unit's top face on the tool plate, z up; the API's cm through mdfc_parts. Top-level
groups Body (grounded) and Carriage; set_state raises the carriage by its transform. No joints.
"""
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_common():
    """Walk up from this script's folder to the first one holding common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('common/companion_paths.py not found above %s' % __file__)
        d = up


_COMMON = _companion_common()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP   # noqa: E402,F401  every lib/ folder is now on sys.path
import mdfc_parts as MP        # noqa: E402
import compliance_unit as CU   # noqa: E402

VERSION = 'ComplianceUnit PT-1 (prompt test, 11 Nov 2026)'
CMD_ID = 'mdfcComplianceUnitPromptTest'
CM = MP.CM

# ---------------------------------------------------------------------------------------------------------------
# the layout, as compliance_unit.py derives it
# ---------------------------------------------------------------------------------------------------------------
L = CU.LAY
G_ = CU.GUIDE
AIR = CU.AIR
HZ = CU.height()
H = HZ['H']                                   # 270: the unit's top to the gripper's face
RAIL = HZ['rail']                             # 218
T_TOP = L['top_t']                            # 20
Z_RAIL = (-T_TOP, -T_TOP - RAIL)              # the rails' top and foot (-20, -238)
Z_CAR = (-T_TOP - RAIL, -H)                   # the carriage plate's top and the gripper's face (-238, -270)
Z_WING = -T_TOP - L['stroke']                 # the wings' tops at rest, the stroke under the top plate (-50)
ISO = [float(a) for a in CU.ISO_B]            # the six M10 in the body's angles: 0, 60, ... 300
PIN = float(CU.PIN_B)                         # the pin at 90
PASSAGES = [float(a) for a in AIR['angles']]  # 30, 150, 210, 330
R_ISO = AIR['r']                              # 80: the 160 circle
Y_GAL = 40.0                                  # the galleries along x at y = +-40
Z_GAL_H = AIR['z_gal']                        # the housing's galleries 10 under the face
Z_GAL_C = Z_CAR[1] + 21.0                     # the carriage's 21 over its face
X_RUN = L['cheek_x'][0] - G_['HR']            # the rails' running faces at x = +-90

# details the prompt leaves open (chosen here, listed in the report)
SPIGOT_H = 8.0           # the 100 g6 spigot's height over the face, under the tool plate's 10 mm recess
PIN_IN, PIN_OUT = 12.0, 8.0
M10_CB = (8.75, 10.0)    # the M10 x 25 heads' counterbores from below: radius, depth (head flush, 15 mm into the tool plate)
PIN_HOLE_C = 12.0        # the carriage's pin hole depth
STUD = (13.0, 15.0)      # the seats' M10 studs: length, the hole's depth
H1 = 5.5                 # the block's underside over the rail's foot (HIWIN's H1 for size 25)
RAIL_HOLE = (3.5, 5.5, 9.0)   # the rail's through hole and counterbore radii, counterbore depth
RAIL_PITCH = 60.0
TAP_CLEAR = 0.02         # tapped holes drawn this much over the screw's diameter, so they never clash
PLUG = 8.0               # the plugs' length in the galleries' ends
GROOVE = (12.6, 2.7)     # the O-ring groove's radius and width, its top 7 under the receiver's top
ORING = (10.01, 12.5, 2.5)   # the O-ring's inner and outer radius and its section
SENSOR = dict(d=12.0, L=40.0, gap=2.0)
BRACKET = (25.0, 22.0, 14.0)  # the sensor bracket: width along x, depth along y, how far it reaches under the sensor

_handlers = []
_last = {}
_AUTO = {'on': False}
_DESIGN = {}


# ---------------------------------------------------------------------------------------------------------------
# the design numbers
# ---------------------------------------------------------------------------------------------------------------
def _design():
    if 'd' not in _DESIGN:
        _DESIGN['d'] = CU.design()
    return _DESIGN['d']


def check_inputs(gripper, stroke, F_pre):
    """Refuse what the unit cannot take; return what is printed about the inputs."""
    d = _design()
    if gripper not in ('bag', 'vacuum', 'clamp'):
        raise ValueError('unknown gripper %r: bag, vacuum or clamp' % gripper)
    if gripper == 'clamp':
        n = d['ring']['clamp'][H]
        raise ValueError('the clamp gripper is refused: with the unit %.0f mm tall in the stack the tool changer\'s '
                         'locking ring would open in the worst stop (ring factor %.2f, under 1)' % (H, n))
    if not stroke > 0.0:
        raise ValueError('a stroke of %.1f mm is refused: it must be over 0' % stroke)
    if stroke > L['stroke'] + 1e-9:
        raise ValueError('a stroke of %.1f mm is refused: the rails give %.0f mm' % (stroke, L['stroke']))
    need = d['grippers'][gripper]['springs']['F_pre']
    if F_pre < need - 1e-6:
        raise ValueError('a preload of %.1f N is refused: the springs\' preload must be at least %.1f N to keep the '
                         'carriage on its stops in the stops that throw the %s gripper\'s load up' % (F_pre, need, gripper))
    return need


# ---------------------------------------------------------------------------------------------------------------
# temporary bodies (world mm)
# ---------------------------------------------------------------------------------------------------------------
def _box(x0, x1, y0, y1, z0, z1):
    xs, ys, zs = sorted((x0, x1)), sorted((y0, y1)), sorted((z0, z1))
    return MP.box(xs[0], xs[1], ys[0], ys[1], zs[0], zs[1])


def _xbox(s, a0, a1, y0, y1, z0, z1):
    """A box between s*a0 and s*a1 along x (s = +1 or -1, the side)."""
    return _box(s * a0, s * a1, y0, y1, z0, z1)


def _polar(a, r=R_ISO):
    return r * math.cos(math.radians(a)), r * math.sin(math.radians(a))


def _rail_screw_z():
    n = 4
    end = (RAIL - (n - 1) * RAIL_PITCH) / 2.0
    return [Z_RAIL[0] - end - RAIL_PITCH * k for k in range(n)]


def housing_body():
    """The top plate 274 x 200 x 20 with its spigot, the two cheeks and the tube's boss, then every hole."""
    px, py = L['plate'][0] / 2.0, L['plate'][1] / 2.0
    c0, c1 = L['cheek_x']
    cy = L['cheek_y']
    b = _box(-px, px, -py, py, -T_TOP, 0.0)
    MP.union(b, MP.cyl((0, 0, -1.0), (0, 0, SPIGOT_H), 50.0))
    for s in (1, -1):
        MP.union(b, _xbox(s, c0, c1, -cy, cy, Z_RAIL[1], -T_TOP + 1.0))
    rb, db = AIR['boss']
    MP.union(b, MP.cyl((0, 0, -T_TOP - db), (0, 0, -T_TOP + 1.0), rb))
    cuts = []
    for a in ISO:                                      # six M10 x 25 from below, heads in counterbores
        x, y = _polar(a)
        cuts.append(MP.cyl((x, y, -T_TOP - 1.0), (x, y, 1.0), 5.5))
        cuts.append(MP.cyl((x, y, -T_TOP - 1.0), (x, y, -T_TOP + M10_CB[1]), M10_CB[0]))
    x, y = _polar(PIN)                                 # the 10 pin's hole
    cuts.append(MP.cyl((x, y, -PIN_IN), (x, y, 1.0), 5.0))
    z_pass = Z_GAL_H - AIR['gallery'] / 2.0
    for a in PASSAGES:                                 # the four passages from the face down to the galleries
        x, y = _polar(a)
        cuts.append(MP.cyl((x, y, z_pass), (x, y, 1.0), AIR['passage'] / 2.0))
    rg = AIR['gallery'] / 2.0
    for sy in (1, -1):                                 # the galleries along x, drilled through, plugged at both edges
        cuts.append(MP.cyl((-px - 1.0, sy * Y_GAL, Z_GAL_H), (px + 1.0, sy * Y_GAL, Z_GAL_H), rg))
    cuts.append(MP.cyl((0, -py - 1.0, Z_GAL_H), (0, Y_GAL + rg, Z_GAL_H), rg))   # along y, from the -y edge
    ca = CU.air()
    cuts.append(MP.cyl((0, 0, -T_TOP - db - 1.0), (0, 0, ca['tube_top']), AIR['tube'][0] / 2.0 + 0.01))  # the tube's seat
    cuts.append(MP.cyl((0, 0, ca['tube_top'] - 1.0), (0, 0, Z_GAL_H), 6.0))     # into the cross gallery
    for x, y in L['spring_xy']:                         # the seats' M10 studs
        cuts.append(MP.cyl((x, y, -T_TOP - 1.0), (x, y, -T_TOP + STUD[1]), 5.0 + TAP_CLEAR))
    for s in (1, -1):
        for z in _rail_screw_z():                       # the rails' M6, tapped in the cheeks
            cuts.append(_xcyl(s, c0 - 1.0, c0 + 10.0, 0.0, z, 3.0 + TAP_CLEAR))
        for sy in (1, -1):                              # the stop blocks' M8, clear through the cheeks
            ym = sy * (L['lug_y'][0] + L['lug_y'][1]) / 2.0
            for z in L['stop_screws']:
                cuts.append(_xcyl(s, c0 - 1.0, c1 + 1.0, ym, z, 4.5))
    MP.cut(b, *cuts)
    return b


def _xcyl(s, a0, a1, y, z, r):
    return MP.cyl((s * a0, y, z), (s * a1, y, z), r)


def housing_plugs():
    px, py = L['plate'][0] / 2.0, L['plate'][1] / 2.0
    return _plugs(px, py, Z_GAL_H)


def _plugs(px, py, zg):
    r = AIR['gallery'] / 2.0 - 0.01
    out = []
    for s in (1, -1):
        for sy in (1, -1):
            out.append(MP.cyl((s * (px - PLUG), sy * Y_GAL, zg), (s * px, sy * Y_GAL, zg), r))
    out.append(MP.cyl((0, -py, zg), (0, -py + PLUG, zg), r))
    return out


def carriage_body():
    """The plate 176 x 200 x 32 with the two wings, the four lugs, the receiver and the springs' spigots; the tool
    plate's underside (recess, six M10 tapped, the pin hole), the passages, galleries and the receiver's bore."""
    cx, cy = L['carriage_xy']
    zt, zf = Z_CAR
    w0, w1 = L['wing_x']
    wy = L['wing_y']
    b = _box(-cx, cx, -cy, cy, zf, zt)
    for s in (1, -1):
        MP.union(b, _xbox(s, w0, w1, -wy, wy, zt - 1.0, Z_WING))
        for sy in (1, -1):
            MP.union(b, _xbox(s, w1 - 1.0, L['lug_x'][1], sy * L['lug_y'][0], sy * L['lug_y'][1],
                              Z_WING - L['lug_t'], Z_WING))
    rr, rh = AIR['receiver']
    z_rec = zt + rh                                   # the receiver's top
    MP.union(b, MP.cyl((0, 0, zt - 1.0), (0, 0, z_rec), rr))
    rs, ls = L['seat'][1], L['seat'][2]
    for x, y in L['spring_xy']:
        MP.union(b, MP.cyl((x, y, zt - 1.0), (x, y, zt + ls), rs))
    cuts = [MP.cyl((0, 0, zf - 1.0), (0, 0, zf + 10.0), 50.0)]          # the 100 H7 recess, 10 deep
    for a in ISO:                                                        # six M10 tapped 15 deep
        x, y = _polar(a)
        cuts.append(MP.cyl((x, y, zf - 1.0), (x, y, zf + 15.0), 4.25))
    x, y = _polar(PIN)
    cuts.append(MP.cyl((x, y, zf - 1.0), (x, y, zf + PIN_HOLE_C), 5.0))   # the 10 pin hole
    rg = AIR['gallery'] / 2.0
    for a in PASSAGES:
        x, y = _polar(a)
        cuts.append(MP.cyl((x, y, zf - 1.0), (x, y, Z_GAL_C + rg), AIR['passage'] / 2.0))
    for sy in (1, -1):
        cuts.append(MP.cyl((-cx - 1.0, sy * Y_GAL, Z_GAL_C), (cx + 1.0, sy * Y_GAL, Z_GAL_C), rg))
    cuts.append(MP.cyl((0, -cy - 1.0, Z_GAL_C), (0, Y_GAL + rg, Z_GAL_C), rg))
    cuts.append(MP.cyl((0, 0, zt), (0, 0, z_rec + 1.0), AIR['tube'][0] / 2.0 + 0.1))   # the receiver's bore, floor at zt
    cuts.append(MP.cyl((0, 0, Z_GAL_C), (0, 0, zt + 1.0), 6.0))                     # the floor into the cross gallery
    cuts.append(MP.cyl((0, 0, z_rec - 7.0 - GROOVE[1]), (0, 0, z_rec - 7.0), GROOVE[0]))   # the O-ring's groove
    MP.cut(b, *cuts)
    return b


def carriage_plugs():
    return _plugs(L['carriage_xy'][0], L['carriage_xy'][1], Z_GAL_C)


def oring_body():
    z_rec = Z_CAR[0] + AIR['receiver'][1]
    top = z_rec - 7.0 - 0.1
    return MP.tube((0, 0, top - ORING[2]), (0, 0, top), ORING[1], ORING[0])


def tube_body():
    ca = CU.air()
    return MP.tube((0, 0, ca['tube_bot']), (0, 0, ca['tube_top']), AIR['tube'][0] / 2.0, AIR['tube'][1] / 2.0)


def block_centres():
    up = Z_WING - L['gap'] - G_['L'] / 2.0
    return [up, up - L['d']]


def linear_block(s, zc):
    """An HGH25CA: H 40 over the rail's foot (its top on the wing's outer face), 48 wide, 84 long, round the rail."""
    c0 = L['cheek_x'][0]
    b = _xbox(s, c0 - G_['H'], c0 - H1, -G_['W'] / 2.0, G_['W'] / 2.0, zc - G_['L'] / 2.0, zc + G_['L'] / 2.0)
    MP.cut(b, _xbox(s, X_RUN, c0 + 1.0, -G_['WR'] / 2.0, G_['WR'] / 2.0, zc - G_['L'] / 2.0 - 1.0, zc + G_['L'] / 2.0 + 1.0))
    return b


def rail_body(s):
    c0 = L['cheek_x'][0]
    b = _xbox(s, X_RUN, c0, -G_['WR'] / 2.0, G_['WR'] / 2.0, Z_RAIL[1], Z_RAIL[0])
    cuts = []
    for z in _rail_screw_z():
        cuts.append(_xcyl(s, X_RUN - 1.0, c0 + 1.0, 0.0, z, RAIL_HOLE[0]))
        cuts.append(_xcyl(s, X_RUN - 1.0, X_RUN + RAIL_HOLE[2], 0.0, z, RAIL_HOLE[1]))
    MP.cut(b, *cuts)
    return b


def stop_block(s, sy):
    z_top = Z_WING - L['lug_t']
    x0, x1 = L['stop_x']
    b = _xbox(s, x0, x1, sy * L['lug_y'][0], sy * L['lug_y'][1], z_top - L['stop_h'], z_top)
    ym = sy * (L['lug_y'][0] + L['lug_y'][1]) / 2.0
    MP.cut(b, *[_xcyl(s, x1 + 1.0, x1 - 14.0, ym, z, 4.0 + TAP_CLEAR) for z in L['stop_screws']])
    return b


def seat_body(x, y):
    r, rs, ls = L['seat']
    zb = L['seat_bottom']
    b = MP.cyl((x, y, zb), (x, y, -T_TOP), r)
    MP.union(b, MP.cyl((x, y, zb - ls), (x, y, zb + 1.0), rs))
    MP.union(b, MP.cyl((x, y, -T_TOP - 1.0), (x, y, -T_TOP + STUD[0]), 5.0))
    return b


def sensor_geometry():
    xs = (L['wing_x'][0] + L['wing_x'][1]) / 2.0
    y0 = L['wing_y'] + SENSOR['gap']
    zs = Z_WING + L['sense'] + SENSOR['d'] / 2.0
    return xs, y0, zs


def sensor_body():
    xs, y0, zs = sensor_geometry()
    return MP.cyl((xs, y0, zs), (xs, y0 + SENSOR['L'], zs), SENSOR['d'] / 2.0)


def bracket_body():
    xs, y0, zs = sensor_geometry()
    w, dy, under = BRACKET
    ya = y0 + 8.0
    b = _box(xs - w / 2.0, xs + w / 2.0, ya, ya + dy, zs - under, -T_TOP)
    MP.cut(b, MP.cyl((xs, ya - 1.0, zs), (xs, ya + dy + 1.0, zs), SENSOR['d'] / 2.0 + 0.1))
    return b


def spring_template(root, sp):
    """One spring of the design: a circle swept once along a helix (a fitted spline through its points) in a scratch
    component, copied as a temporary body and ground flat between the seat (-138) and the carriage (-238)."""
    occ = MP.new_occ(root, 'Spring sweep')
    comp = occ.component
    R, d = sp['D'] / 2.0, sp['d']
    z0, z1 = Z_CAR[0], L['seat_bottom']
    pitch = (sp['L1'] - 2.0 * d) / sp['Na']            # closed and ground ends: L = p Na + 2 d
    za, zb = z0 - d, z1 + d
    turns = (zb - za) / pitch
    n = int(math.ceil(turns * 24))
    pts = adsk.core.ObjectCollection.create()
    for i in range(n + 1):
        a = 2.0 * math.pi * turns * i / n
        pts.add(MP.P3(R * math.cos(a) * CM, R * math.sin(a) * CM, (za + (zb - za) * i / n) * CM))
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.isComputeDeferred = True
    spline = sk.sketchCurves.sketchFittedSplines.add(pts)
    sk.isComputeDeferred = False
    pl_in = comp.constructionPlanes.createInput()
    pl_in.setByDistanceOnPath(spline, adsk.core.ValueInput.createByReal(0.0))
    plane = comp.constructionPlanes.add(pl_in)
    sk2 = comp.sketches.add(plane)
    sk2.sketchCurves.sketchCircles.addByCenterRadius(sk2.modelToSketchSpace(MP.P3(R * CM, 0.0, za * CM)), d / 2.0 * CM)
    path = comp.features.createPath(spline, False)
    si = comp.features.sweepFeatures.createInput(sk2.profiles.item(0), path,
                                                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    si.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    sw = comp.features.sweepFeatures.add(si)
    tb = MP.tbm().copy(sw.bodies.item(0))
    MP.intersect(tb, _box(-R - d, R + d, -R - d, R + d, z0, z1))
    occ.deleteMe()
    return tb


def _moved(tb, x, y, z=0.0):
    c = MP.tbm().copy(tb)
    m = adsk.core.Matrix3D.create()
    m.translation = MP.V3(x * CM, y * CM, z * CM)
    MP.tbm().transform(c, m)
    return c


# ---------------------------------------------------------------------------------------------------------------
# components
# ---------------------------------------------------------------------------------------------------------------
AL = ('Aluminum 2024-T3', 'Aluminum 2024-T4', 'Aluminum 2024', 'Aluminum')
STEEL = ('Steel',)
RUBBER = ('Rubber, Nitrile', 'Rubber', 'Rubber, Silicone')
BRASS = ('Brass', 'Steel')


def _material(body, names):
    try:
        lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    except Exception:
        lib = None
    if lib is None:
        return None
    for nm in names:
        try:
            m = lib.materials.itemByName(nm)
        except Exception:
            m = None
        if m:
            body.material = m
            return nm
    return None


def _occ(parent_comp, name):
    """A component inside a group, at the group's origin, grounded to it. A name already used (the Carriage part in
    the Carriage group) takes the ' (1)' Fusion gives a repeated name."""
    occ = parent_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    try:
        occ.component.name = name
    except Exception:
        occ.component.name = name + ' (1)'
    occ.isGroundToParent = True
    return occ


def _part(group, name, kind, tb, mats):
    occ = _occ(group.component, name)
    occ.component.attributes.add('mdfc', 'kind', kind)
    body = MP.add_body(occ, tb, name.split(' (')[0])
    _material(body, mats)
    return occ


def parts_list(root):
    """{kind: count} of every part (an occurrence with a body) in the design."""
    out = {}
    for o in root.allOccurrences:
        if not o.component.bRepBodies.count:
            continue
        a = o.component.attributes.itemByName('mdfc', 'kind')
        k = a.value if a else o.component.name
        out[k] = out.get(k, 0) + 1
    return out


def set_state(*args):
    """set_state(lift) or set_state(groups, lift): the carriage raised by `lift` mm, by its transform."""
    if len(args) == 1:
        groups, lift = _last['groups'], args[0]
    else:
        groups, lift = args[0], args[1]
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(0.0, 0.0, float(lift) * CM)
    groups['Carriage'].transform2 = m
    return lift


# ---------------------------------------------------------------------------------------------------------------
# the build
# ---------------------------------------------------------------------------------------------------------------
def build(gripper='bag', stroke=30.0, F_pre=853.3):
    global _last
    need = check_inputs(gripper, stroke, F_pre)
    d = _design()
    sp = d['coil']
    gd = d['grippers'][gripper]

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    root = des.rootComponent

    body_g = MP.new_occ(root, 'Body')
    car_g = MP.new_occ(root, 'Carriage')
    body_g.isGrounded = True

    # the body: the housing and everything fixed to it
    _part(body_g, 'Housing', 'Housing, 2024-T3', housing_body(), AL)
    x, y = _polar(PIN)
    _part(body_g, 'Pin 10', 'Pin 10 x 20', MP.cyl((x, y, -PIN_IN), (x, y, PIN_OUT), 4.99), STEEL)
    for i, a in enumerate(ISO):
        x, y = _polar(a)
        _part(body_g, 'Screw M10x25 %d' % (i + 1), 'Screw ISO 4762 M10 x 25',
              MP.cap_screw(10.0, 25.0, (x, y, -T_TOP + M10_CB[1]), (0.0, 0.0, 1.0)), STEEL)
    for i, tb in enumerate(housing_plugs()):
        _part(body_g, 'Plug housing %d' % (i + 1), 'Plug 8 x 8', tb, STEEL)
    k = 0
    for s, tag in ((1, '+x'), (-1, '-x')):
        _part(body_g, 'Rail HGR25 %s' % tag, 'Rail HGR25R x %.0f' % RAIL, rail_body(s), STEEL)
        for z in _rail_screw_z():
            k += 1
            _part(body_g, 'Screw M6x20 %d' % k, 'Screw ISO 4762 M6 x 20',
                  MP.cap_screw(6.0, 20.0, (s * (X_RUN + RAIL_HOLE[2]), 0.0, z), (s, 0.0, 0.0)), STEEL)
    k = 0
    for s, tx in ((1, '+x'), (-1, '-x')):
        for sy, ty in ((1, '+y'), (-1, '-y')):
            _part(body_g, 'Stop block %s%s' % (tx, ty), 'Stop block', stop_block(s, sy), STEEL)
            ym = sy * (L['lug_y'][0] + L['lug_y'][1]) / 2.0
            for z in L['stop_screws']:
                k += 1
                _part(body_g, 'Screw M8x30 %d' % k, 'Screw ISO 4762 M8 x 30',
                      MP.cap_screw(8.0, 30.0, (s * L['cheek_x'][1], ym, z), (-s, 0.0, 0.0)), STEEL)
    tags = ('+x+y', '-x+y', '-x-y', '+x-y')
    for (x, y), t in zip(L['spring_xy'], tags):
        _part(body_g, 'Spring seat %s' % t, 'Spring seat 46 with M10 stud', seat_body(x, y), STEEL)
    coil = spring_template(root, sp)
    for (x, y), t in zip(L['spring_xy'], tags):
        _part(body_g, 'Spring %s' % t, 'Spring, music wire %g x %g, %g installed' % (sp['d'], sp['D'], sp['L1']),
              _moved(coil, x, y), STEEL)
    _part(body_g, 'Tube 20x16', 'Tube 20 x 16, steel', tube_body(), STEEL)
    _part(body_g, 'Sensor M12', 'Proximity sensor M12 x 40', sensor_body(), BRASS)
    _part(body_g, 'Sensor bracket', 'Sensor bracket', bracket_body(), AL)

    # the carriage and what it carries
    _part(car_g, 'Carriage', 'Carriage, 2024-T3', carriage_body(), AL)
    k = 0
    for s, tx in ((1, '+x'), (-1, '-x')):
        for zc, tz in zip(block_centres(), ('upper', 'lower')):
            _part(car_g, 'Block HGH25CA %s %s' % (tx, tz), 'Block HGH25CA', linear_block(s, zc), STEEL)
    _part(car_g, 'O-ring 20x2.5', 'O-ring 20 x 2.5, NBR', oring_body(), RUBBER)
    for i, tb in enumerate(carriage_plugs()):
        _part(car_g, 'Plug carriage %d' % (i + 1), 'Plug 8 x 8', tb, STEEL)

    groups = {'Body': body_g, 'Carriage': car_g}
    _last = dict(groups=groups, coil=sp, gripper=gripper, design=d)
    set_state(groups, 0.0)

    pl = parts_list(root)
    try:
        m_body = body_g.physicalProperties.mass
        m_car = car_g.physicalProperties.mass
        mass = 'Weighed: the body %.2f kg, the carriage %.2f kg.' % (m_body, m_car)
    except Exception:
        mass = ''
    ring = d['ring']
    gs = gd['guides']
    lines = [
        'Compliance unit for the %s gripper.' % gripper,
        'Height %.0f mm from the tool plate to the gripper\'s face (top plate %.0f, rails %.0f, carriage %.0f); '
        'stroke asked %.1f mm, built %.0f mm (the rails give %.0f, the sensor sees %.0f).'
        % (H, T_TOP, RAIL, L['carriage_t'], stroke, L['stroke'], L['stroke'], L['sense']),
        'Guides: 2 x %s rails %.0f long, 4 x %s blocks (C0 %.0f N); the worst block carries %.0f N in the worst stop, '
        'static factor %.2f (at least %.0f).' % (G_['rail'], RAIL, G_['name'], G_['C0'], gs['P'], gs['fSL'], G_['fSL_min']),
        'Springs: %d x music wire %g x %g mean, %.1f active turns, %.0f installed and %.0f at the top of the stroke, '
        'rate %.2f N/mm, %.1f N each installed (%.1f N in all), %.1f N at the top, static factor %.2f. Preload asked '
        '%.1f N, needed %.1f N.' % (CU.SPRING['n'], sp['d'], sp['D'], sp['Na'], sp['L1'], sp['L2'], sp['k'], sp['F1'],
                                    4 * sp['F1'], sp['F2'], sp['n'], F_pre, need),
        'Ring factor with the unit in the stack: %.2f (%s), against %.2f without it; vacuum %.2f, bag %.2f, clamp %.2f.'
        % (ring[gripper][H], gripper, ring[gripper][0.0], ring['vacuum'][H], ring['bag'][H], ring['clamp'][H]),
        'Parts (%d): %s.' % (sum(pl.values()), ', '.join('%s %d' % (k_, v) for k_, v in sorted(pl.items()))),
    ]
    if mass:
        lines.append(mass)
    lines.append(VERSION)
    return '\n'.join(lines)


# ---------------------------------------------------------------------------------------------------------------
# the dialog
# ---------------------------------------------------------------------------------------------------------------
class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('gripper', 'Gripper', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in ('bag', 'vacuum', 'clamp'):
            dd.listItems.add(k, k == 'bag', '')
        ins.addValueInput('stroke', 'Stroke', 'mm', adsk.core.ValueInput.createByString('30 mm'))
        ins.addValueInput('F_pre', 'Springs\' preload (N)', '', adsk.core.ValueInput.createByReal(853.3))
        ex = _Execute()
        cmd.execute.add(ex)
        _handlers.append(ex)
        de = _Destroy()
        cmd.destroy.add(de)
        _handlers.append(de)


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            gripper = ins.itemById('gripper').selectedItem.name
            stroke = ins.itemById('stroke').value / CM
            F_pre = ins.itemById('F_pre').value
            msg = build(gripper=gripper, stroke=stroke, F_pre=F_pre)
            ui.messageBox(msg)
        except ValueError as e:
            ui.messageBox('Refused: %s\n%s' % (e, VERSION))
        except Exception:
            ui.messageBox('Failed:\n%s\n%s' % (traceback.format_exc(), VERSION))


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _AUTO['on']:
            adsk.terminate()


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Compliance unit', 'Builds the palletizer\'s compliance unit')
        on = _Created()
        cd.commandCreated.add(on)
        _handlers.append(on)
        if context is not None:
            _AUTO['on'] = True
            adsk.autoTerminate(False)
        cd.execute()
    except Exception:
        ui.messageBox('Failed:\n%s' % traceback.format_exc())


def stop(context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
