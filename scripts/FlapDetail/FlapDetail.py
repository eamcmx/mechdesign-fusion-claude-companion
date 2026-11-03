# =============================================================================
# FlapDetail.py  -  Chapter 53 of "Mechanical Design in Autodesk Fusion with Claude": the flap actuation system of
# Chapter 34 in detail. It builds Chapter 34's system with FlapSystem.py, loaded, not copied, with the motor unit drawn
# to maxon's figures (lib/flap_detail.py: the EC-i 52 and the GP 52 C at 26:1 behind the unit's flange, the brake in
# the length the specification leaves), the pinion on the gearhead's 12 mm shaft by maxon's 4 x 4 x 20 key with an
# end screw and washer, the hinge pins' and the horn pin's bearing seats split out as the zones that are hardened, and
# then gives every made part its material and every fitting face its class and roughness as Chapter 51's
# MaterialsFits.py does, the zones and the pinion's keyed bore by this chapter's own rules.
# Written from PROMPT.txt beside it; checked by checks/flap_detail_check.py.
#
#   INPUTS        the dialog: the gearhead's ratio, one of the GP 52 C's two-stage ratios (26)
#   DERIVED       flap_detail: numbers(), pinion_force(), pinion_rim(), pinion_key(), hinge_zones()
#   HARD RULES    build() refuses a ratio the motor cannot drive (over its continuous torque or its speed at 28 V)
#                 and a ratio maxon does not make; nothing of FlapSystem's, FlapActuator's or MaterialsFits' code copied
#   CONVENTIONS   as FlapSystem.py; the changed parts keep their names, the new ones named for what they are
#   ACCEPTANCE    the message; the check: the unit's lengths, the key and the pinion's hub, the screw, the zones and
#                 the faces' classes, no interference at 0 to 40 deg, the refusal
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
LIB = os.path.join(os.path.dirname(SCRIPTS), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_fd53_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


FS = _load('FlapSystem')                # loads FlapActuator and reloads flap_design
MF = _load('MaterialsFits')
import flap_design as fd                # noqa: E402  (the module FS and its actuator use, after their reloads)
import mdfc_parts as mp                 # noqa: E402
import flap_detail as FDET              # noqa: E402
importlib.reload(FDET)
ACT = FS.ACT

VERSION = 'FlapDetail 1.0 (2026-11-03)'
CM = 0.1
VI = adsk.core.ValueInput.createByReal
_last = {}
PIN = FDET.PIN
HARD = 'induction hardened 50 HRC min., case 0.8 to 1.2 mm, ground'
SEAT = 'h6 Rz 10.0'                      # SKF's shaft seat for a clearance or transition fit, hardened (Chapter 51)
BESIDE = 'h6 Ra 0.8'                     # the pin beside the ring, in the cheeks' and spacers' bores (an IT6 pin's finish)
KEYED = 'K7 Ra 1.6'                      # the pinion's bore on maxon's shaft (lib/flap_detail.pinion_key)
KEYWAY = 'JS9 Ra 3.2'                    # the hub's keyway, the loose fit (Chapter 51)


def design(i_gh=26.0):
    return FDET.numbers(i=i_gh)


def refuse(i_gh, n):
    short = []
    if i_gh not in FDET.RATIOS_2STAGE:
        short.append('maxon makes the two-stage GP 52 C at %s:1 only' % ', '.join('%g' % r for r in FDET.RATIOS_2STAGE))
    else:
        if n['T_m'] > FDET.MOTOR['T_n']:
            short.append('at %g:1 the motor would give %.1f mN m, over its %.0f continuous' % (i_gh, n['T_m'], FDET.MOTOR['T_n']))
        if n['n_m'] > n['n_28']:
            short.append('at %g:1 the motor would turn at %.0f rev/min, over the %.0f it reaches at 28 V'
                         % (i_gh, n['n_m'], n['n_28']))
    if short:
        raise ValueError('The motor unit falls short: %s.' % '; '.join(short))


# ---------- the replacements, each recognising only the calls it means ----------
def along_y(x, z, y0, y1, r, r_in=0.0):
    return (_orig.get('along_y') or ACT.along_y)(x, z, y0, y1, r, r_in)


def in_frame(occ, tb):
    """A temporary body given in the parent's frame moved into the occurrence's own, for FlapSystem's combine."""
    m = occ.transform2.copy()
    m.invert()
    mp.tbm().transform(tb, m)
    return tb


def add_body_detail(occ, tb, name, material=None):
    """The motor unit's gearhead, motor and brake at maxon's lengths behind the unit's flange (6 thick from s = 40):
    the GP 52 C 65 long, the EC-i 52 to the catalogue's overall 145.1 with it, the brake in the 176 the specification
    gives the unit."""
    if occ.component.name == 'Motor unit' and name in ('Gearhead', 'Motor', 'Brake'):
        s0 = PIN['flange'][1]
        s1 = s0 + FDET.GEARHEAD['L1']
        s2 = s0 + FDET.OVERALL
        s3 = PIN['flange'][0] + FDET.SPEC['L']
        span = {'Gearhead': (s0, s1), 'Motor': (s1, s2), 'Brake': (s2, s3)}[name]
        label = {'Gearhead': 'Gearhead GP 52 C 26-1', 'Motor': 'Motor EC-i 52 180 W', 'Brake': 'Brake to specification'}[name]
        tb = along_y(0, fd.LAY['z_P'], span[0], span[1], FDET.GEARHEAD['D'] / 2)
        return _orig['add_body'](occ, tb, label, material)
    return _orig['add_body'](occ, tb, name, material)


def gear_detail(parent, name, *a, **kw):
    """The pinion with a hub forward to the shaft's end, 18 across, and the keyway for maxon's 4 x 4 key cut through
    its bore, DIN 6885-1's hub depth t2 = 1.8 above it."""
    if name != 'Pinion z18':
        return _orig['gear'](parent, name, *a, **kw)
    kw['hub'] = (PIN['hub'][0], PIN['hub'][1], PIN['r_hub'])
    o = _orig['gear'](parent, name, *a, **kw)
    zP = fd.LAY['z_P']
    w = FDET.GEARHEAD['key'][0] / 2
    tool = mp.box(-w, w, PIN['hub'][0] - 0.5, fd.LAY['gear_face'][1] + 0.5, zP, zP + 6.0 + PIN['t2'])
    FS.cut(o, in_frame(o, tool))
    _made['pinion'] = o
    return o


def part_detail(parent, name, tb, material='Steel', t=(0.0, 0.0, 0.0)):
    """The gearhead's output shaft at maxon's 29.5 from the flange's face, its keyseat (t1 = 2.5) with the key in it
    and its centre hole M4 (drawn at the thread's size, as the book's tapped holes are); the end washer, 18 across
    so that it bears on the pinion's hub, and the M4 x 10 through it."""
    if name != 'Gearhead shaft':
        return _orig['part'](parent, name, tb, material, t)
    zP = fd.LAY['z_P']
    s0, s1 = PIN['shaft']
    w, h, kl = FDET.GEARHEAD['key']
    sh = along_y(0, zP, s0, s1, FDET.GEARHEAD['shaft'][0] / 2)
    mp.cut(sh, mp.box(-w / 2, w / 2, PIN['key'][0], PIN['key'][1], zP + 6.0 - PIN['t1'], zP + 6.1),
           along_y(0, zP, s0 - 0.01, s0 + PIN['centre_hole'], PIN['screw'][0] / 2 + 0.05))
    o = _orig['part'](parent, name, sh, material, t)
    key = mp.box(-w / 2, w / 2, PIN['key'][0], PIN['key'][1], zP + 6.0 - PIN['t1'], zP + 6.0 - PIN['t1'] + h)
    mp.add_body(o, key, 'Key DIN 6885-A 4x4x20', 'Steel')
    W = FDET.END_WASHER
    sw, sl = PIN['washer'][0], PIN['screw'][1]
    _made['washer'] = _orig['part'](parent, 'Pinion end washer', along_y(0, zP, PIN['washer'][0], PIN['washer'][1],
                                                                     W['d2'] / 2, W['d1'] / 2))
    _made['screw'] = _orig['part'](parent, 'Cap screw ISO 4762 M4x10', mp.cap_screw(PIN['screw'][0], sl, (0.0, PIN['washer'][0], zP),
                                                                          (0, 1, 0)))
    _made['shaft'] = o
    return o


def along_y_detail(x, z, y0, y1, r, r_in=0.0):
    """The hole through the housing's back wall for the gearhead's shaft, 14 across in Chapter 33's housing, opened to
    17 so that maxon's key, whose corner stands 7.76 from the axis, passes it."""
    if (x, z, round(y0, 2), round(y1, 2), r, r_in) == (0, fd.LAY['z_P'], round(fd.LAY['shoulder_ring'][0] - 0.01, 2),
                                                       round(fd.LAY['lobe_s'] + 0.01, 2), 7.0, 0.0):
        r = HOLE_R
    return _orig['along_y'](x, z, y0, y1, r, r_in)


HOLE_R = 8.5
_orig, _made = {}, {}


def _patch():
    _orig.update(add_body=mp.add_body, gear=ACT.gear, part=ACT.part, along_y=ACT.along_y)
    mp.add_body = add_body_detail
    ACT.along_y = along_y_detail
    ACT.gear = gear_detail
    ACT.part = part_detail


def _unpatch():
    mp.add_body, ACT.gear, ACT.part, ACT.along_y = _orig['add_body'], _orig['gear'], _orig['part'], _orig['along_y']


# ---------- the zones hardened ----------
def split_zone(occ, x0, x1, radius):
    """The pin's cylinder split at x0 and x1 (its component's frame is the flap's) by two planes across it."""
    comp = occ.component
    for x in (x0, x1):
        cyl = [f for b in comp.bRepBodies for f in b.faces
               if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - radius) < 1e-3
               and f.boundingBox.minPoint.x * 10 < x - 0.01 and f.boundingBox.maxPoint.x * 10 > x + 0.01]
        pi_ = comp.constructionPlanes.createInput()
        pi_.setByOffset(comp.yZConstructionPlane, VI(x * CM))
        pl = comp.constructionPlanes.add(pi_)
        pl.isLightBulbOn = False
        faces = adsk.core.ObjectCollection.create()
        for f in cyl:
            faces.add(f)
        comp.features.splitFaceFeatures.add(comp.features.splitFaceFeatures.createInput(faces, pl, True))


def pin_faces(occ, radius=9.99):
    """The pin's cylinders of that radius as the assembly holds them (the faces MaterialsFits marks)."""
    return [f for b in occ.bRepBodies for f in b.faces
            if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - radius) < 1e-3]


def zones(root):
    hz = FDET.hinge_zones()
    out = {}
    for o in root.allOccurrences:
        nm = o.component.name
        if nm in ('Hinge pin L', 'Hinge pin R', 'Horn pin'):
            x0, x1 = hz['horn'] if nm == 'Horn pin' else hz['hinges'][0 if nm.endswith('L') else 1]
            split_zone(o, x0, x1, 9.99)
            out[nm] = (o, x0, x1)
    return out


def mark(f, value, kind, des, new):
    if not f.attributes.itemByName('mdfc', 'fit'):
        new.append(f)
    f.attributes.add('mdfc', 'fit', value)
    f.appearance = MF.appearance(des, MF.COLOURS[kind])


def by_hand(des, zs):
    """What Chapter 51's census cannot settle. On each pin the seat's class and finish under the ring only, the face
    marked as hardened; the faces beside it, in the cheeks or the spacers, h6 with the turned finish of an IT6 pin
    (the census gave the seat's class to all three, for they touch the ring's edge at the same 0.01 mm), and the horn
    pin's seat in the rod end's ball (the census paired the rod end's bore with the rod's thread). The pinion's bore
    and keyway on maxon's shaft, one subassembly turning together. Returns ({pin: zone faces}, faces changed, faces
    that had no class before)."""
    changed, new, zone = [], [], {}
    for nm, (o, x0, x1) in zs.items():
        zone[nm] = []
        for f in pin_faces(o):
            bb = f.boundingBox
            inside = bb.minPoint.x * 10 > x0 - 0.01 and bb.maxPoint.x * 10 < x1 + 0.01
            if inside:
                mark(f, SEAT, 'plain bearing seat', des, new)
                f.attributes.add('mdfc', 'treat', HARD)
                zone[nm].append(f)
            else:
                mark(f, BESIDE, 'located', des, new)
            changed.append(f)
    o = next((x for x in des.rootComponent.allOccurrences if x.component.name == 'Pinion z18'), None)
    for b in o.bRepBodies if o else []:
        for f in b.faces:
            g = f.geometry
            bb = f.boundingBox
            if isinstance(g, adsk.core.Cylinder) and abs(g.radius * 10 - 6.0) < 1e-3:
                mark(f, KEYED, 'keyed', des, new)
                changed.append(f)
            elif isinstance(g, adsk.core.Plane) and abs(abs(g.normal.x) - 1.0) < 1e-6 \
                    and abs(abs(bb.minPoint.x * 10) - FDET.GEARHEAD['key'][0] / 2) < 1e-3:
                mark(f, KEYWAY, 'key seat', des, new)
                changed.append(f)
    return zone, changed, new


def build(i_gh=26.0):
    n = design(i_gh)
    refuse(i_gh, n)
    app = adsk.core.Application.get()
    _made.clear()
    _patch()
    try:
        msg34 = FS.build()
    finally:
        _unpatch()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    zs = zones(root)
    msg51 = MF.apply('flap')
    zone, hand, new = by_hand(des, zs)
    pf, rim, key = FDET.pinion_force(), FDET.pinion_rim(), FDET.pinion_key()
    _last.update(numbers=n, zones=zone, hand=hand, new=new, mf=dict(MF._last), made=dict(_made))
    return ('Flap system in detail: the motor unit at maxon\'s lengths, the GP 52 C at %g:1 and the EC-i 52, %.1f mm with '
            'the flange, the brake in the %.1f mm left; the pinion keyed on the 12 mm shaft, %.1f mm of key in its hub, '
            'held by an M4 x 10 and an end washer.\nMotor %.1f mN m of %.0f continuous at %.0f rev/min (%.0f at 28 V), %.2f A; '
            'brake %.3f N m specified (needs %.3f); pinion\'s force %.0f N at the limit, %.0f at the ultimate, against '
            'the gearhead\'s %.0f N at %.0f mm; the keyed rim K_B %.3f, fatigue factor %.2f.\nZones hardened: %d faces; '
            'by hand: %d faces.\n%s\n\n[%s]'
            % (i_gh, FDET.FLANGE_T + FDET.OVERALL, n['brake_len'], key['L_in'], n['T_m'], FDET.MOTOR['T_n'], n['n_m'],
               n['n_28'], n['I_m'], n['brake'], n['brake_need'], pf['W'], pf['W_u'], pf['rating'], pf['at'], rim['KB'],
               rim['nf'], sum(len(v) for v in zone.values()), len(hand), msg51.split('\n')[0], VERSION))


def pose(delta):
    FS.pose(delta, FS._last_joints)


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('i_gh', 'Gearhead ratio, GP 52 C two-stage', 26.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            vals = {k: float(inputs.itemById(k).value) for k, _, _ in FIELDS}
            ui.messageBox(build(**vals))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('FlapDetail failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        for k, label, v in FIELDS:
            cmd.commandInputs.addValueInput(k, label, '', adsk.core.ValueInput.createByReal(v))


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcFlapDetail'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Flap actuator in detail', 'Chapter 53: the flap actuator in detail')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('FlapDetail failed:\n%s' % traceback.format_exc())
