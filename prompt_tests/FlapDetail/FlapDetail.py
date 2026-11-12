# =============================================================================
# FlapDetail.py  -  Chapter 53 of "Mechanical Design in Autodesk Fusion with Claude": the flap actuation system in
# detail. FlapSystem.py builds the system with its own build(); before it runs, four of the functions that draw the
# parts are replaced, each acting only on the call it means: the body adder (the motor unit's gearhead, motor and
# brake drawn to maxon's EC-i 52 and GP 52 C), the gear (the pinion with a hub and a keyway), the part (the gearhead's
# shaft as maxon makes it, its key, the end washer and the M4 screw) and the cylinder that cuts the shaft's hole
# through the housing's back wall (opened from 14 to 17 across). After the build the hinge pins' and the horn pin's
# seats are split out and marked hardened, and MaterialsFits.apply() gives the materials and the fits; the horn
# pin's seat, the pins' faces beside the seats, the pinion's bore and its keyway are given their classes by hand.
#
#   INPUTS        the dialog: the gearhead's ratio, one of the two-stage GP 52 C's (26)
#   DERIVED       flap_detail: numbers(i), MOTOR, GEARHEAD, OVERALL, SPEC, PIN, pinion_force(), pinion_rim(),
#                 pinion_key(), hinge_zones(); flap_design.LAY; MaterialsFits.apply(); materials_fits' rules
#   HARD RULES    a ratio maxon does not make refused; one at which the motor would exceed its continuous torque or
#                 the speed it reaches at 28 V refused
#   CONVENTIONS   FlapSystem's own: a new design, the flap's As-built revolute joint, the actuator's joints and
#                 Motion Links; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the motor unit's bodies; the shaft, key, hub, washer and screw; three zones; the
#                 faces' classes; no interference at 0 to 40 deg; the 21:1 gearhead refused
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import flap_design as fd        # noqa: E402
import flap_detail as FDET      # noqa: E402
import materials_fits as M      # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(fd)
importlib.reload(FDET)
importlib.reload(M)
importlib.reload(mp)


def _load(name, script):
    spec = importlib.util.spec_from_file_location(name, CP.script(script))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


FS = _load('mdfc_flapsystem', 'FlapSystem')             # its ACT is the FlapActuator module it builds with
MF = _load('mdfc_materialsfits', 'MaterialsFits')

VERSION = 'FlapDetail 1.0 (2026-11-11)'
CM = 0.1
VI = adsk.core.ValueInput.createByReal
L = fd.LAY
ZP = L['z_P']                                           # the motor unit's axis in the actuator's frame (x = 0)
GH, MO, PIN, SPEC = FDET.GEARHEAD, FDET.MOTOR, FDET.PIN, FDET.SPEC
OLD_HOLE, HOLE = 14.0, 17.0                             # the back wall's hole for the gearhead's shaft, across
M4_HOLE = 4.1                                           # the shaft's centre hole, drawn clear of the M4 (thread not drawn)
SHANK = 9.99                                            # the pins' shank radius (FlapSystem.pin: 19.98 across)

# the classes given by hand, written as MaterialsFits writes them
SEAT = MF.finish(M.PLAIN_SHAFT['clearance'], dict(rz=M.PLAIN_RZ[0]))      # a plain bearing's seat on a pin: h6 Rz 10
BESIDE = MF.finish('h6', dict(ra=M._seat_ra(6, 20.0)))                    # H7/h6 located, its own grade's Ra
KEYED = MF.finish('K7', dict(ra=M._seat_ra(7, GH['shaft'][0])))           # the pinion's bore on maxon's shaft
KEYWAY = MF.finish(M.KEYS_HUB, dict(ra=M.OTHER_RA))                       # DIN 6885-1, the hub's side
HARD = 'hardened to HRC %.0f-%.0f, case %.1f-%.1f mm' % (FDET.HARD['HRC_range'] + FDET.HARD['case'])


def refuse(i):
    """The motor unit at ratio i, or a ValueError: a ratio maxon does not make in the two-stage GP 52 C, or one at
    which the motor would exceed its continuous torque or the speed it reaches at 28 V with this load."""
    if not any(abs(i - r) < 1e-6 for r in FDET.RATIOS_2STAGE):
        raise ValueError('maxon makes no two-stage GP 52 C of %g:1; the ratios are %s.'
                         % (i, ', '.join('%g:1' % r for r in FDET.RATIOS_2STAGE)))
    n = FDET.numbers(i=i)
    short = []
    if n['T_m'] > MO['T_n']:
        short.append('at %g:1 the motor would give %.0f mN m, over its %.0f continuous' % (i, n['T_m'], MO['T_n']))
    if n['n_m'] > n['n_28']:
        short.append('at %g:1 the motor would turn at %.0f rev/min, over the %.0f it reaches at 28 V'
                     % (i, n['n_m'], n['n_28']))
    if short:
        raise ValueError('The motor unit falls short: %s.' % '; '.join(short))
    return n


def pose(delta):
    """The flap down by delta (deg), the actuator swung and its rod out, through FlapSystem's own pose()."""
    FS.pose(delta, FS._last_joints)


def _assembly_key():
    """The key the design office's inventory gives the flap system (the assembly FlapSystem.py builds)."""
    D = MF.D
    for row in getattr(D, 'MACHINES', []):
        if any('FlapSystem' in str(v) for v in row[1:]):
            return row[0]
    scripts = D.SCRIPTS
    for k in scripts:
        v = scripts[k] if isinstance(scripts, dict) else k
        if 'FlapSystem' in str(v):
            return k
    for k in scripts:
        if 'flap' in str(k).lower():
            return k
    raise RuntimeError('the design office lists no assembly built by FlapSystem.py')


def _radius(f):
    g = f.geometry
    return g.radius * 10.0 if isinstance(g, adsk.core.Cylinder) else None


# ---------- the replacements, each acting on the one call it means ----------
def _replacements(i, orig, hits):
    ACT = FS.ACT
    s_face, s_back = PIN['flange']                       # the flange's face on the housing's back face, its back
    unit = {'Gearhead': (s_back, s_back + GH['L1'], GH['D'], 'Gearhead GP 52 C %g-1' % i),
            'Motor': (s_back + GH['L1'], s_back + FDET.OVERALL, MO['D'], 'Motor EC-i 52 180 W'),
            'Brake': (s_back + FDET.OVERALL, s_face + SPEC['L'], SPEC['D'], 'Brake to specification')}

    def add_body(occ, tb, name, material=None):
        if name in unit and occ.component.name == 'Motor unit':
            a, b, d, new = unit[name]
            hits[name] += 1
            return orig['add_body'](occ, mp.cyl((0.0, a, ZP), (0.0, b, ZP), d / 2.0), new, material)
        return orig['add_body'](occ, tb, name, material)

    def gear(parent, name, z, x, zc, y0, y1, face, bore, phase_deg, hub=None, web=None):
        if name != 'Pinion z18' or abs(zc - ZP) > 1e-9:
            return orig['gear'](parent, name, z, x, zc, y0, y1, face, bore, phase_deg, hub=hub, web=web)
        hits['gear'] += 1
        h0, h1 = PIN['hub']
        # the blank and a hub forward to the shaft's end, drawn solid (the hub run 1 mm into the blank, under its
        # root); the bore and the keyway cut together after the teeth, through hub and teeth
        o = orig['gear'](parent, name, z, x, zc, y0, y1, face, 0.0, phase_deg, hub=(h0, h1 + 1.0, PIN['r_hub']), web=web)
        rb = GH['shaft'][0] / 2.0
        kb = GH['key'][0]
        tool = mp.cyl((x, h0 - 0.01, zc), (x, y1 + 0.01, zc), rb)
        mp.union(tool, mp.box(x - kb / 2.0, x + kb / 2.0, h0 - 0.01, y1 + 0.01, zc, zc + rb + PIN['t2']))
        ACT._cut_body(o, tool)
        return o

    def part(parent, name, tb, material='Steel', t=(0.0, 0.0, 0.0)):
        if name != 'Gearhead shaft':
            return orig['part'](parent, name, tb, material, t)
        hits['part'] += 1
        d = GH['shaft'][0]
        s0, s1 = PIN['shaft']
        k0, k1 = PIN['key']
        kb, kh, _ = GH['key']
        sh = mp.cyl((0.0, s0, ZP), (0.0, s1, ZP), d / 2.0)
        mp.keyseat(sh, d, k0, k1, kb, PIN['t1'], 0.0, ZP)
        mp.cut(sh, mp.cyl((0.0, s0 - 0.01, ZP), (0.0, s0 + PIN['centre_hole'], ZP), M4_HOLE / 2.0))
        o = orig['part'](parent, name, sh, material, t)
        orig['add_body'](o, mp.parallel_key(d, k0, k1, kb, kh, PIN['t1'], 0.0, ZP), 'Key DIN 6885-A 4x4x20', 'Steel')
        # the end washer on the hub and the M4 x 10 through it into the shaft, in the pinion's subassembly so that
        # they turn with it
        w = FDET.END_WASHER
        w0, w1 = PIN['washer']
        orig['part'](parent, 'Pinion end washer', mp.tube((0.0, w0, ZP), (0.0, w1, ZP), w['d2'] / 2.0, w['d1'] / 2.0),
                     'Steel')
        orig['part'](parent, 'Cap screw ISO 4762 M4x10',
                     mp.cap_screw(PIN['screw'][0], PIN['screw'][1], (0.0, w0, ZP), (0, 1, 0)), 'Steel')
        return o

    def along_y(x, z, y0, y1, r, r_in=0.0):
        if (not r_in and abs(r - OLD_HOLE / 2.0) < 1e-9 and abs(x) < 1e-9 and abs(z - ZP) < 1e-9
                and abs(y0 - (L['shoulder_ring'][0] - 0.01)) < 1e-6 and abs(y1 - (L['lobe_s'] + 0.01)) < 1e-6):
            hits['hole'] += 1
            r = HOLE / 2.0
        return orig['along_y'](x, z, y0, y1, r, r_in)

    return dict(add_body=add_body, gear=gear, part=part, along_y=along_y)


# ---------- the seats on the pins ----------
def split_seat(leaf, a, b):
    """The pin's shank split across at x = a and x = b (its component's frame is the world's); returns the face
    between, and the faces beside it, through the occurrence."""
    comp = leaf.component
    body = comp.bRepBodies.item(0)
    faces = adsk.core.ObjectCollection.create()
    for f in body.faces:
        r = _radius(f)
        if r is not None and abs(r - SHANK) < 1e-3:
            faces.add(f)
    tools = adsk.core.ObjectCollection.create()
    for x in (a, b):
        ci = comp.constructionPlanes.createInput()
        ci.setByOffset(comp.yZConstructionPlane, VI(x * CM))
        pl = comp.constructionPlanes.add(ci)
        pl.isLightBulbOn = False
        tools.add(pl)
    sf = comp.features.splitFaceFeatures
    sf.add(sf.createInput(faces, tools, True))
    zone, beside = [], []
    for f in leaf.bRepBodies.item(0).faces:
        r = _radius(f)
        if r is None or abs(r - SHANK) > 1e-3:
            continue
        bb = f.boundingBox                                       # the flap turns about x: x stays the world's
        x0, x1 = bb.minPoint.x * 10.0, bb.maxPoint.x * 10.0
        (zone if (x0 > a - 0.01 and x1 < b + 0.01) else beside).append(f)
    return zone, beside


def pinion_faces(leaf):
    """The pinion's bore and its keyway's two sides, through the occurrence (found in its component's frame, whose
    origin is on the pinion's axis)."""
    rb = GH['shaft'][0] / 2.0
    kb = GH['key'][0]
    bore, sides = [], []
    for f in leaf.bRepBodies.item(0).faces:
        nf = f.nativeObject or f
        g = nf.geometry
        if isinstance(g, adsk.core.Cylinder) and abs(g.radius * 10.0 - rb) < 1e-3:
            bore.append(f)
        elif isinstance(g, adsk.core.Plane) and abs(abs(g.normal.x) - 1.0) < 1e-6:
            bb = nf.boundingBox
            if (abs(abs(bb.minPoint.x * 10.0) - kb / 2.0) < 1e-3 and abs(bb.maxPoint.x - bb.minPoint.x) < 1e-4
                    and bb.minPoint.z * 10.0 > 0.0 and bb.maxPoint.z * 10.0 < rb + PIN['t2'] + 1e-3):
                sides.append(f)
    return bore, sides


_last = {}


def build(i_gh=26.0):
    i = float(i_gh)
    n = refuse(i)
    kb, kh, _ = GH['key']
    corner = math.hypot(GH['shaft'][0] / 2.0 - PIN['t1'] + kh, kb / 2.0)
    if corner >= HOLE / 2.0:
        raise RuntimeError('the key\'s corner, %.2f mm from the axis, does not pass the %.0f mm hole' % (corner, HOLE))
    ACT = FS.ACT
    orig = dict(add_body=mp.add_body, gear=ACT.gear, part=ACT.part, along_y=ACT.along_y)
    hits = dict(Gearhead=0, Motor=0, Brake=0, gear=0, part=0, hole=0)
    new_fn = _replacements(i, orig, hits)
    mp.add_body = new_fn['add_body']
    ACT.gear, ACT.part, ACT.along_y = new_fn['gear'], new_fn['part'], new_fn['along_y']
    try:
        FS.build()
    finally:
        mp.add_body = orig['add_body']
        ACT.gear, ACT.part, ACT.along_y = orig['gear'], orig['part'], orig['along_y']
    missed = [k for k, v in hits.items() if v != 1]
    if missed:
        raise RuntimeError('the replaced functions did not each meet their one call: %s' % hits)
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    leaves = {o.component.name: o for o in root.allOccurrences if o.component.bRepBodies.count}
    # the seats on the hinge pins and the horn pin, split out and marked hardened
    hz = FDET.hinge_zones()
    where = {'Hinge pin L': hz['hinges'][0], 'Hinge pin R': hz['hinges'][1], 'Horn pin': hz['horn']}
    zones, beside = {}, {}
    for name, (a, b) in where.items():
        zones[name], beside[name] = split_seat(leaves[name], a, b)
        for f in zones[name]:
            f.attributes.add('mdfc', 'treat', HARD)
    # the materials and the fits, as Chapter 51 decides them on the open design
    key = _assembly_key()
    mf_msg = MF.apply(key)
    # by hand: the seats and the faces beside them, the pinion's bore and its keyway
    bore, sides = pinion_faces(leaves['Pinion z18'])
    hand, new = [], []

    def give(f, val, joint):
        if f.attributes.itemByName('mdfc', 'fit') is None:
            new.append(f)
        f.attributes.add('mdfc', 'fit', val)
        f.appearance = MF.appearance(des, MF.COLOURS[joint])
        hand.append(f)
    for name in where:
        for f in zones[name]:
            give(f, SEAT, 'plain bearing seat')
        for f in beside[name]:
            give(f, BESIDE, 'located')
    for f in bore:
        give(f, KEYED, 'keyed')
    for f in sides:
        give(f, KEYWAY, 'key seat')
    _last.clear()
    _last.update(i=i, numbers=n, zones=zones, beside=beside, bore=bore, sides=sides, hand=hand, new=new, mf=MF._last,
                 key=key)
    pf, rim, pk = FDET.pinion_force(), FDET.pinion_rim(), FDET.pinion_key()
    lens = sorted({round(b - a, 3) for a, b in where.values()})
    return ('Flap system in detail: the motor unit to maxon\'s EC-i 52 180 W and GP 52 C %g:1, the pinion keyed on '
            'the gearhead\'s shaft, the pins\' seats hardened, the materials and the fits given.\n\n'
            'Motor unit: the flange %.0f, the gearhead %.0f, the motor %.1f (maxon\'s overall %.1f), the brake %.1f long, '
            'all %.0f across but the flange, %.0f mm from the flange\'s face as the specification.\n'
            'Motor: %.0f mN m at %.0f rev/min (%.0f %% of its %.0f continuous; it reaches %.0f rev/min at 28 V with this '
            'load), %.2f A (%.0f %% of %.2f); %.2f A from the bus.\n'
            'Brake: %.3f N m to hold the ultimate, %.3f N m with the factor of %.0f.\n'
            'Pinion: tooth force %.0f N at the limit and %.0f N at the ultimate, %.0f mm from the flange, against the '
            'gearhead\'s %.0f N at %.0f mm (%.2f and %.2f of it; %.1f N m against %.2f N m about the flange).\n'
            'Rim over the keyway: m_B %.2f, K_B %.3f; the pinion\'s fatigue factor %.2f (%.2f without the keyway).\n'
            'Key %gx%gx%g in the hub %.1f mm: %.1f MPa on its side at the limit, %.1f at the ultimate; the K7 bore on '
            'maxon\'s shaft %+.3f to %+.3f mm; the M4 x %.0f through the %.0f mm washer, %.0f mm in the shaft (%.0f with '
            'the shaft at its shortest). The key\'s corner %.2f mm from the axis, the hole %.0f across.\n'
            'Zones %s: the hinge pins %.0f to %.0f and %.0f to %.0f, the horn pin %.0f to %.0f mm; %s mm long.\n'
            'Faces: %s\nBy hand %d faces (%d carried no class before): the seats %s, beside them %s, the pinion\'s bore '
            '%s, its keyway %s.\n\n[%s]'
            % (i, PIN['flange'][1] - PIN['flange'][0], GH['L1'], FDET.OVERALL - GH['L1'], FDET.OVERALL, n['brake_len'],
               GH['D'], SPEC['L'],
               n['T_m'], n['n_m'], 100 * n['use_T'], MO['T_n'], n['n_28'], n['I_m'], 100 * n['use_I'], MO['I_n'],
               n['I_bus'],
               n['brake_need'], n['brake'], FDET.BRAKE_FACTOR,
               pf['W'], pf['W_u'], pf['arm'], pf['rating'], pf['at'], pf['over'], pf['over_u'], pf['M'], pf['M_rating'],
               rim['mB'], rim['KB'], rim['nf'], rim['nf_v2'],
               GH['key'][0], GH['key'][1], GH['key'][2], pk['L_in'], pk['sig'], pk['sig_u'], pk['fit'][0], pk['fit'][1],
               PIN['screw'][1], FDET.END_WASHER['h'], pk['engage'], pk['engage_short'], corner, HOLE,
               HARD, hz['hinges'][0][0], hz['hinges'][0][1], hz['hinges'][1][0], hz['hinges'][1][1], hz['horn'][0],
               hz['horn'][1], ', '.join('%g' % v for v in lens),
               mf_msg.split('\n\n[')[0], len(hand), len(new), SEAT, BESIDE, KEYED, KEYWAY, VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('i_gh', 'Gearhead ratio (GP 52 C, two stages)', 26.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            ui.messageBox(build(**{k: ins.itemById(k).value for k, _, _ in FIELDS}), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for key, label, dflt in FIELDS:
            ins.addValueInput(key, label, '', VI(dflt))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'FlapDetailCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Flap system in detail', 'The flap actuation system of Chapter 53')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('FlapDetail could not start:\n' + traceback.format_exc())
