# -*- coding: utf-8 -*-
"""StudyParts: the six parts Fusion's static and modal Simulation studies are run on (Chapters 58, 59 and 62), each
built in a new design of its own as one body with its material, with the faces each study picks found on the body.

    cantilever      200 x 20 x 10 along x from x = 0, the 10 mm depth along z
    keyseat         the 30 mm stub, 150 long along x, a DIN 6885 form A keyseat 8 x 4 x 25 at its middle, the
                    floor's four edges rounded r = 0.6 (r/d = 0.02)
    tine            the leg 250 long (25 along y, 10 along x), its top at z = -230; the foot 255 along y from the
                    leg's outer face inward, 25 deep, under the leg's lower end
    bracket_sharp   the foot 12 thick and 70 square, the arm 24 square and 48 long, the root sharp
    bracket_r25     the same with a 2.5 mm fillet all round the arm's root
    lug             12 thick, 55 wide, the eye's end round with radius 27.5 on the hole's centre, the hole 35 across,
                    the shank 100 long to the spar (Aluminum 7075)

The sizes are typed here as the prompt gives them; design_office/lib/fea_studies.py gives the hand calculations the
studies are set against and the hand volume each part is held to. Every body is made from temporary bodies in a base
feature, the fillets are fillet features on its edges. Lengths in mm, the API's cm inside. Nothing is saved.
"""
import importlib
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('No common/companion_paths.py in any folder above %s' % __file__)
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP            # noqa: E402  every lib/ folder is now on sys.path
import fea_studies as FS                # noqa: E402  design_office/lib: the hand calculations and hand volumes
importlib.reload(FS)

VERSION = 'StudyParts v1.0'
PARTS = ('cantilever', 'keyseat', 'tine', 'bracket_sharp', 'bracket_r25', 'lug')
DEFAULT = 'cantilever'
MATERIALS = dict(cantilever='Steel', keyseat='Steel', tine='Steel', bracket_sharp='Steel', bracket_r25='Steel',
                 lug='Aluminum 7075')
CMD_ID = 'mdfcStudyPartsPromptTest'
CM = 0.1                                # cm per mm: the API works in cm

# ---- the sizes, mm, as the hard constraints give them ---------------------------------------------------------------
CANT = dict(L=200.0, b=20.0, h=10.0)                     # along x, along y, the depth along z
KEY = dict(d=30.0, L=150.0, b=8.0, t1=4.0, l=25.0, r=0.6)  # stub; DIN 6885 form A keyseat width, depth, length; fillet
TINE = dict(leg=250.0, a=25.0, t=10.0, top=-230.0, foot=255.0, deep=25.0)   # leg length, along y, along x; foot
BRK = dict(t=12.0, side=70.0, a=24.0, h=48.0, r=2.5)     # foot thickness and side; arm side and length; root fillet
LUG = dict(t=12.0, w=55.0, R=27.5, d=35.0, L=100.0)      # thickness, width, eye radius, hole, shank to the spar

app = None
ui = None
_handlers = []
_standalone = False
_last = {}

P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create


# ---- temporary bodies, world millimetres ------------------------------------------------------------------------------
def _tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def _box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2 * CM, (y0 + y1) / 2 * CM, (z0 + z1) / 2 * CM)
    obb = adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0), (x1 - x0) * CM, (y1 - y0) * CM,
                                                 (z1 - z0) * CM)
    return _tbm().createBox(obb)


def _cyl(p0, p1, r):
    return _tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def _bool(a, kind, *bs):
    for b in bs:
        if not _tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def _cut(a, *bs):
    return _bool(a, adsk.fusion.BooleanTypes.DifferenceBooleanType, *bs)


def _union(a, *bs):
    return _bool(a, adsk.fusion.BooleanTypes.UnionBooleanType, *bs)


# Every flat face of a part is kept whole: a part is one block cut by tools, never two blocks joined on a shared plane,
# so that no face is left split where two bodies met.
def _tb_cantilever():
    c = CANT
    return _box(0.0, c['L'], -c['b'] / 2, c['b'] / 2, -c['h'] / 2, c['h'] / 2)


def _tb_keyseat():
    k = KEY
    R, rr = k['d'] / 2, k['b'] / 2
    zf = R - k['t1']                                   # the floor, 11 above the axis
    xm = k['L'] / 2                                    # the keyseat at the stub's middle
    s = (k['l'] - k['b']) / 2                          # the round ends' centres from the middle
    stub = _cyl((0.0, 0.0, 0.0), (k['L'], 0.0, 0.0), R)
    ztop = R + 2.0
    slot = _box(xm - k['l'] / 2 - 0.5, xm + k['l'] / 2 + 0.5, -rr, rr, zf, ztop)
    for xc, sign in ((xm - s, -1), (xm + s, 1)):       # each round end: the end mill's circle, radius b/2
        x_far = xc + sign * (rr + 1.0)
        corner = _box(min(xc, x_far), max(xc, x_far), -rr - 1.0, rr + 1.0, zf - 1.0, ztop + 1.0)
        _cut(corner, _cyl((xc, 0.0, zf - 2.0), (xc, 0.0, ztop + 2.0), rr))
        _cut(slot, corner)
    return _cut(stub, slot)


def _tb_tine():
    t = TINE
    y_out = t['a'] / 2                                 # the leg's outer face; the foot runs from it inward (-y)
    y_in = -t['a'] / 2
    z_low = t['top'] - t['leg']                        # the leg's lower end, -480
    y_end = y_out - t['foot']                          # the foot's inner end, -242.5
    blk = _box(-t['t'] / 2, t['t'] / 2, y_end, y_out, z_low - t['deep'], t['top'])
    return _cut(blk, _box(-t['t'] / 2 - 1.0, t['t'] / 2 + 1.0, y_end - 1.0, y_in, z_low, t['top'] + 1.0))


def _tb_bracket():
    b = BRK
    foot = _box(-b['side'] / 2, b['side'] / 2, -b['side'] / 2, b['side'] / 2, 0.0, b['t'])
    arm = _box(-b['a'] / 2, b['a'] / 2, -b['a'] / 2, b['a'] / 2, b['t'] / 2, b['t'] + b['h'])   # into the foot
    return _union(foot, arm)


def _tb_lug():
    g = LUG
    R, w = g['R'], g['w']
    plate = _box(-R - 0.5, g['L'], -w / 2, w / 2, 0.0, g['t'])     # hole's centre at the origin, the spar at x = 100
    corner = _box(-R - 1.0, 0.0, -w / 2 - 1.0, w / 2 + 1.0, -1.0, g['t'] + 1.0)
    _cut(corner, _cyl((0.0, 0.0, -2.0), (0.0, 0.0, g['t'] + 2.0), R))
    _cut(plate, corner)                                # the eye's end round on the hole's centre
    return _cut(plate, _cyl((0.0, 0.0, -1.0), (0.0, 0.0, g['t'] + 1.0), g['d'] / 2))


BUILDERS = dict(cantilever=_tb_cantilever, keyseat=_tb_keyseat, tine=_tb_tine, bracket_sharp=_tb_bracket,
                bracket_r25=_tb_bracket, lug=_tb_lug)

# The faces each study picks: (key, what it is, how it is found). ('plane', outward normal, its coordinate in mm along
# that axis) or ('bore', radius in mm).
_Z_LOW = TINE['top'] - TINE['leg']
_BRACKET_PICKS = (('fixed', "the foot's back face at z = 0, bolted down", ('plane', (0, 0, -1), 0.0)),
                  ('load', "the arm's end at z = %g, %g from the foot's face, where the push goes on"
                   % (BRK['t'] + BRK['h'], BRK['h']), ('plane', (0, 0, 1), BRK['t'] + BRK['h'])))
PICKS = dict(
    cantilever=(('fixed', 'the root end at x = 0', ('plane', (-1, 0, 0), 0.0)),
                ('load', 'the free end at x = %g, the tip load along -z' % CANT['L'], ('plane', (1, 0, 0), CANT['L']))),
    keyseat=(('fixed', 'the end at x = 0', ('plane', (-1, 0, 0), 0.0)),
             ('load', 'the end at x = %g, the torque about x' % KEY['L'], ('plane', (1, 0, 0), KEY['L']))),
    tine=(('fixed', "the leg's top at z = %g, where it hangs from the jaw" % TINE['top'],
           ('plane', (0, 0, 1), TINE['top'])),
          ('load', "the foot's upper face at z = %g, where the bag lies" % _Z_LOW, ('plane', (0, 0, 1), _Z_LOW))),
    bracket_sharp=_BRACKET_PICKS,
    bracket_r25=_BRACKET_PICKS,
    lug=(('fixed', 'the shank end at the spar, x = %g' % LUG['L'], ('plane', (1, 0, 0), LUG['L'])),
         ('load', "the hole's bore, %g across, where the GE 20 ES's outer ring bears" % LUG['d'],
          ('bore', LUG['d'] / 2))),
)


# ---- faces on the body ---------------------------------------------------------------------------------------------
def _outward(body, face):
    """A point on the face (cm) and the face's outward normal there, checked against the body: a step along the
    normal must leave the solid."""
    p = face.pointOnFace
    good, n = face.evaluator.getNormalAtPoint(p)
    if not good:
        return p, None
    n.normalize()
    e = 0.005                                          # 0.05 mm
    q = P3(p.x + n.x * e, p.y + n.y * e, p.z + n.z * e)
    if body.pointContainment(q) == adsk.fusion.PointContainment.PointInsidePointContainment:
        n.scaleBy(-1.0)
    return p, n


def _planes(body, normal, coord_mm):
    """The flat faces whose outward normal is `normal` (an axis) and which lie at coord_mm along that axis."""
    ax = [i for i in range(3) if normal[i]][0]
    out = []
    for f in body.faces:
        if f.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
            continue
        p, n = _outward(body, f)
        if n is None or n.x * normal[0] + n.y * normal[1] + n.z * normal[2] < 0.9999:
            continue
        if abs((p.x, p.y, p.z)[ax] / CM - coord_mm) < 1e-3:
            out.append((f, p, n))
    return out


def _bores(body, r_mm):
    """The cylindrical faces of radius r_mm whose outward normal points at their axis (a hole, not a boss)."""
    out = []
    for f in body.faces:
        if f.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
            continue
        cyl = adsk.core.Cylinder.cast(f.geometry)
        if abs(cyl.radius / CM - r_mm) > 1e-3:
            continue
        p, n = _outward(body, f)
        if n is None:
            continue
        o, a = cyl.origin, cyl.axis
        a.normalize()
        w = V3(p.x - o.x, p.y - o.y, p.z - o.z)
        along = w.dotProduct(a)
        radial = V3(w.x - a.x * along, w.y - a.y * along, w.z - a.z * along)
        if radial.dotProduct(n) < 0:
            out.append((f, p, n))
    return out


def _find(body, how):
    hits = _planes(body, how[1], how[2]) if how[0] == 'plane' else _bores(body, how[1])
    return hits[0] if len(hits) == 1 else None


def _vec(n):
    return '(%s)' % ', '.join('%+.3g' % (0.0 if abs(v) < 5e-7 else v) for v in (n.x, n.y, n.z))


# ---- the model -------------------------------------------------------------------------------------------------------
def _material(name):
    libs = adsk.core.Application.get().materialLibraries
    for ln in ('Fusion Material Library', 'Fusion 360 Material Library'):
        lib = libs.itemByName(ln)
        if lib:
            m = lib.materials.itemByName(name)
            if m:
                return m
    for i in range(libs.count):
        m = libs.item(i).materials.itemByName(name)
        if m:
            return m
    raise RuntimeError("No material called %s in Fusion's material library" % name)


def _fillet(comp, edges, r_mm):
    """A fillet feature of radius r_mm on the given edges."""
    col = adsk.core.ObjectCollection.create()
    for e in edges:
        col.add(e)
    fl = comp.features.filletFeatures
    fi = fl.createInput()
    r = adsk.core.ValueInput.createByString('%g mm' % r_mm)
    try:
        fi.edgeSetInputs.addConstantRadiusEdgeSet(col, r, False)
    except AttributeError:
        fi.addConstantRadiusEdgeSet(col, r, False)
    feat = fl.add(fi)
    if feat is None or feat.healthState == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState:
        raise RuntimeError('the %g mm fillet failed' % r_mm)
    return feat


def _hand(part):
    """The hand calculation the part's study is set against, from fea_studies."""
    try:
        if part == 'cantilever':
            c, m = FS.cantilever(), FS.cantilever_modes()
            return ('%g N at the tip: %.1f MPa at the root, the tip %.3f mm down; first bending modes %.1f Hz (thin), '
                    '%.1f Hz (stiff)' % (FS.CANT['P'], c['sigma'][0.0], c['delta'], m['thin'][0], m['stiff'][0]))
        if part == 'keyseat':
            k = FS.keyseat()
            return ('T = %.1f N m: tau %.2f MPa nominal, %.2f MPa at the keyseat (Kts %g), r/d %.2f'
                    % (FS.KEYSEAT['T'], k['tau'], k['tau_peak'], FS.KEYSEAT['Kts'], k['r_over_d']))
        if part == 'tine':
            t = FS.tine_modes()
            return ('first modes %.1f Hz out of the leg\'s plane, %.1f Hz in it'
                    % (t['outplane'][0], t['inplane'][0]))
        if part.startswith('bracket'):
            b = FS.bracket()
            peak = b['sharp'] if part == 'bracket_sharp' else b['drawn']
            return ('push %.0f N at %g mm: %.1f MPa nominal at the root, %.1f MPa peak (Kt %s)'
                    % (FS.BRACKET['F'], FS.BRACKET['h'], b['sigma'], peak, '2.7' if part == 'bracket_sharp' else '1.7'))
        if part == 'lug':
            g = FS.lug()
            return ('P = %.0f N: %.1f MPa on the net section, %.1f MPa bearing, %.1f MPa peak (Kt %g)'
                    % (FS.LUG['P'], g['sigma0'], g['bearing'], g['peak'], FS.LUG['Kt']))
    except Exception as e:                              # e.g. no numpy for the tine's modes
        return 'not worked out here (%s)' % e
    return ''


def build(part=DEFAULT):
    """Build one study part in a new design; returns the message. Refuses a part that is not one of the six."""
    global _last
    if part not in PARTS:
        raise ValueError('No part called %r: the parts are %s' % (part, ', '.join(PARTS)))
    a = adsk.core.Application.get()
    doc = a.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    comp = des.rootComponent

    tb = BUILDERS[part]()
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = comp.bRepBodies.item(0)

    if part == 'keyseat':                               # the floor's four edges: two straight, two round ends
        floor = _find(body, ('plane', (0, 0, 1), KEY['d'] / 2 - KEY['t1']))
        if floor is None or floor[0].edges.count != 4:
            raise RuntimeError("the keyseat's floor was not found with its four edges")
        _fillet(comp, list(floor[0].edges), KEY['r'])
    elif part == 'bracket_r25':                         # all round the arm's root: the inner loop of the foot's face
        top = _find(body, ('plane', (0, 0, 1), BRK['t']))
        inner = [lp for lp in top[0].loops if not lp.isOuter] if top else []
        if len(inner) != 1 or inner[0].edges.count != 4:
            raise RuntimeError("the arm's root was not found on the foot's face")
        _fillet(comp, list(inner[0].edges), BRK['r'])

    if comp.bRepBodies.count != 1 or comp.occurrences.count != 0:
        raise RuntimeError('the part must be one body in the root component')
    body = comp.bRepBodies.item(0)
    body.name = part
    body.material = _material(MATERIALS[part])

    try:
        pp = body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
    except Exception:
        pp = body.physicalProperties
    volume, mass = pp.volume * 1000.0, pp.mass          # mm^3, kg
    hand = FS.VOLUMES[part]

    found, picks, lines = {}, {}, []
    for key, what, how in PICKS[part]:
        hit = _find(body, how)
        found[key] = hit is not None
        if hit:
            f, p, n = hit
            picks[key] = dict(what=what, point=(p.x / CM, p.y / CM, p.z / CM), normal=(n.x, n.y, n.z),
                              area=f.area / CM ** 2)
            lines.append('  %s: %s; outward normal %s, %.1f mm2, found' % (key, what, _vec(n), f.area / CM ** 2))
        else:
            lines.append('  %s: %s; NOT FOUND on the body' % (key, what))

    _last = dict(part=part, volume=volume, hand_volume=hand, material=body.material.name, faces=body.faces.count,
                 mass=mass, found=found, picks=picks, version=VERSION)
    msg = '\n'.join([
        '%s: %s' % (VERSION, part),
        'One body in the root component, %s.' % body.material.name,
        'Volume %.2f mm3 (hand volume in fea_studies %.2f mm3, %+.4f %%).' % (volume, hand, 100 * (volume - hand) / hand),
        'Mass %.4f kg. Faces: %d.' % (mass, body.faces.count),
        'Faces to pick in the Simulation window: %s.' % ', '.join(
            '%s %s' % (k, 'found' if v else 'MISSING') for k, v in found.items())] + lines + [
        'Hand calculation: %s' % _hand(part)])
    print(msg)
    try:
        a.activeViewport.fit()
    except Exception:
        pass
    return msg


# ---- the command dialog ----------------------------------------------------------------------------------------------
class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            part = args.command.commandInputs.itemById('part').selectedItem.name
            msg = build(part)
            ui.messageBox(msg, 'StudyParts')
        except Exception:
            ui.messageBox('StudyParts failed:\n%s' % traceback.format_exc(), 'StudyParts')


class _Destroy(adsk.core.CommandEventHandler):
    def notify(self, args):
        if _standalone:                                 # a script started by Fusion ends with its dialog
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        cmd.isExecutedWhenPreEmpted = False
        dd = cmd.commandInputs.addDropDownCommandInput('part', 'Part', adsk.core.DropDownStyles.TextListDropDownStyle)
        for p in PARTS:
            dd.listItems.add(p, p == DEFAULT)
        on_exec, on_destroy = _Execute(), _Destroy()
        cmd.execute.add(on_exec)
        cmd.destroy.add(on_destroy)
        _handlers.extend([on_exec, on_destroy])


def run(context):
    global app, ui, _standalone
    app = adsk.core.Application.get()
    ui = app.userInterface
    _standalone = context is not None                   # Fusion passes a context; another script calling run() does not
    try:
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Study parts',
                                                       'Builds one of the six parts the Simulation studies are run on')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        if _standalone:
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StudyParts failed:\n%s' % traceback.format_exc(), 'StudyParts')
