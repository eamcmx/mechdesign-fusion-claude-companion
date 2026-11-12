# =============================================================================
# VacuumGripper.py  -  the palletizer's vacuum gripper (Chapter 46 of "Mechanical Design in Autodesk Fusion with
# Claude"), written from projects/palletizer/VacuumGripper/PROMPT.txt alone (a prompt test). The adapter to
# ISO 9409-1-160-6-M10 is the parallel-jaw gripper's, built by ClampGripper.py's adapter(); under it two aluminium
# beams and a plate; over each of the six cartons of a layer a zone of six Schmalz SPB1 80 ED-65 bellows cups on
# spring level compensators, a manifold and two SCPSi 2-14 ejectors; and the layer of cartons the gripper holds.
#
#   INPUTS        the dialog: the cartons under the gripper, a carton's mass (kg), the vacuum the system holds (bar)
#   DERIVED       lib/vacuum_gripper.py: CUP, EJECTOR, LAY, SPRING, spring(), accelerations(), holding()
#   HARD RULES    build() refuses more than six cartons, a carton the cups do not hold (use over 1 at S = 2.0 in
#                 the cycle and the worst brake stop for its payload) and a vacuum under 0.6 bar
#   CONVENTIONS   z = 0 the gripper's mounting face, z up, x along the pallet's 1200 mm; groups Adapter and Frame
#                 (grounded), Zone 1 to 6 (plungers and cups), Pack; set_state() lifts the zones and the pack
#   ACCEPTANCE    the message (cartons, the cups' use in the worst stop, the parts list, the version tag)
# =============================================================================
import importlib
import importlib.util
import json
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
import mdfc_parts as mp       # noqa: E402
import vacuum_gripper as vg   # noqa: E402
importlib.reload(mp)
importlib.reload(vg)


def _load(name, script):
    """Another script of the companion, loaded as a module (not copied): its functions are called as they are."""
    spec = importlib.util.spec_from_file_location(name, CP.script(script))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


CGS = _load('mdfc_clampgripper', 'ClampGripper')        # projects/palletizer/ClampGripper/ClampGripper.py

VERSION = 'VacuumGripper 1.0 (prompt test, 2026-11-11)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

L = vg.LAY
CUP = vg.CUP
EJ = vg.EJECTOR
SP = vg.SPRING
PS = L['plate_screw']
N_ZONES = vg.LAYER['nx'] * vg.LAYER['ny']               # six zones, one over each carton of a 3 x 2 layer
M8 = CGS.M8                                              # the adapter's four M8, where the clamp gripper has them
M8_L = 16.0          # the M8 from under the beam's 5 mm top wall: 11 mm into the adapter's 15 mm tapped holes
M8_CLEAR = 4.5       # the M8's clearance hole in the top wall, as the clamp gripper's beam
M8_KEY_BEAM = 9.5    # the key's way in through the beam's bottom wall (19 mm: clear of the side wall's inner face)
M8_KEY_PLATE = 10.0  # and through the plate (vacuum_gripper.frame() weighs the plate with four holes of 20 mm)
CUP_WALL = 2.0       # the bellows' rubber wall
EJ_GAP = 1.0         # each ejector this far from the manifold's middle, the two side by side along x
SPRING_GAP = 0.005   # the wire's ends this far off the plate and the seat, so that the spring only touches nothing


def zone_centres():
    """The six zones' centres (mm), Zone 1 to 6: along x, and at each x the -y zone first."""
    return [(zx, zy) for zx in L['zone_x'] for zy in L['zone_y']]


def zone_cups(zx, zy):
    return [(zx + dx, zy + dy) for dx in L['cup_dx'] for dy in L['cup_dy']]


def all_cups():
    return [c for zx, zy in zone_centres() for c in zone_cups(zx, zy)]


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def cone(x, y, z0, r0, z1, r1):
    return mp.tbm().createCylinderOrCone(P3(x * CM, y * CM, z0 * CM), r0 * CM, P3(x * CM, y * CM, z1 * CM), r1 * CM)


def moved(tb, dx=0.0, dy=0.0, dz=0.0):
    m = adsk.core.Matrix3D.create()
    m.translation = V3(dx * CM, dy * CM, dz * CM)
    mp.tbm().transform(tb, m)
    return tb


def group(root, name, grounded=False):
    o = mp.new_occ(root, name)
    o.isGroundToParent = False
    if grounded:
        o.isGrounded = True
    return o


def part(g, name, tb, material='Steel'):
    o = mp.new_occ(g.component, name)
    mp.add_body(o, tb, name, material)
    return o


# ---------- the spring, swept once along a helix ----------
def spring_body(root):
    """The level compensator's spring as a temporary body, its axis on z at x = y = 0: music wire d swept along a
    helix of mean diameter D, its total coils Nt (spring()) spread evenly over the installed length L1 between the
    plate's underside and the seat. The sweep is made once in a scratch component, copied as a temporary body and
    the scratch component deleted."""
    s = vg.spring()
    r, R = SP['d'] / 2.0, SP['D'] / 2.0
    z_seat, z_plate = L['seat'] + L['seat_t'], L['plate'][1]          # the seat's face, the plate's underside
    turns = s['Nt']
    # the wire's end sections are square to the helix, tilted by the lead angle: their lowest point is r cos(lead)
    # below the wire's centre, so the centre line runs from z_seat + e to z_plate - e
    p = (z_plate - z_seat - 2 * r) / turns
    e = r * math.cos(math.atan(p / (2 * math.pi * R))) + SPRING_GAP
    z0, z1 = z_seat + e, z_plate - e
    p = (z1 - z0) / turns
    occ = mp.new_occ(root, 'Helix sweep (temporary)')
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.isComputeDeferred = True
    pts = adsk.core.ObjectCollection.create()
    n = int(math.ceil(turns * 16))
    for i in range(n + 1):
        t = turns * i / n
        a = 2 * math.pi * t
        pts.add(sk.modelToSketchSpace(P3(R * math.cos(a) * CM, R * math.sin(a) * CM, (z0 + p * t) * CM)))
    helix = sk.sketchCurves.sketchFittedSplines.add(pts)
    sk.isComputeDeferred = False
    cpi = comp.constructionPlanes.createInput()
    cpi.setByDistanceOnPath(helix, VI(0.0))
    plane = comp.constructionPlanes.add(cpi)
    ps = comp.sketches.add(plane)
    ps.sketchCurves.sketchCircles.addByCenterRadius(ps.modelToSketchSpace(P3(R * CM, 0.0, z0 * CM)), r * CM)
    sw = comp.features.sweepFeatures
    inp = sw.createInput(ps.profiles.item(0), comp.features.createPath(helix, False),
                         adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    feat = sw.add(inp)
    tb = mp.tbm().copy(feat.bodies.item(0))
    occ.deleteMe()
    return tb


# ---------- the parts ----------
def adapter(root):
    """The parallel-jaw gripper's adapter to ISO 9409-1-160-6-M10, built by ClampGripper.py itself."""
    return CGS.adapter(root)


def frame(root, spring_tb):
    g = group(root, 'Frame', True)
    zt, zb = L['beam']                                           # -16, -116
    t = L['beam_t']
    hw = L['beam_w'] / 2
    hl = L['beam_len'] / 2
    for k, s in enumerate((1, -1)):
        yc = s * L['beam_y']
        b = mp.box(-hl, hl, yc - hw, yc + hw, zb, zt)
        mp.cut(b, mp.box(-hl - 1, hl + 1, yc - hw + t, yc + hw - t, zb + t, zt - t))      # 60 x 100, walls 5
        for x, y in M8:
            if y * s > 0:
                mp.cut(b, zcyl(zt - t - 1, zt + 1, M8_CLEAR, x, y))                       # the adapter's M8
                mp.cut(b, zcyl(zb - 1, zb + t + 1, M8_KEY_BEAM, x, y))                    # the key's way in
        for x in PS['x']:
            mp.cut(b, zcyl(zt - t - 1, zt + 1, PS['access_r'], x, yc))                    # the M6 put in from above
            mp.cut(b, zcyl(zb - 1, zb + t + 1, PS['clear_r'], x, yc))                     # their clearance holes
        part(g, 'Beam %d' % (k + 1), b, 'Aluminum')
    p_top, p_bot = L['plate']                                    # -116, -126
    pl = mp.box(-L['plate_l'] / 2, L['plate_l'] / 2, -L['plate_w'] / 2, L['plate_w'] / 2, p_bot, p_top)
    for x, y in all_cups():
        mp.cut(pl, zcyl(p_bot - 1, p_top + 1, L['sleeve_r'][1], x, y))                   # the plungers' bores
    for x, y in M8:
        mp.cut(pl, zcyl(p_bot - 1, p_top + 1, M8_KEY_PLATE, x, y))
    for s in (1, -1):
        for x in PS['x']:
            mp.cut(pl, zcyl(p_top - 9.0, p_top + 1, PS['d'] / 2, x, s * L['beam_y']))   # M6 tapped 9 deep (drawn at 6)
    part(g, 'Plate', pl, 'Aluminum')
    for k, (x, y) in enumerate(M8):
        part(g, 'Cap screw M8x%d %d' % (M8_L, k + 1), mp.cap_screw(8.0, M8_L, (x, y, zt - t), (0, 0, 1)))
    n = 0
    for s in (1, -1):
        for x in PS['x']:
            n += 1
            part(g, 'Cap screw M6x14 %d' % n,
                 mp.cap_screw(PS['d'], PS['L'], (x, s * L['beam_y'], zb + t), (0, 0, -1)))   # head on the bottom wall
    for k, (x, y) in enumerate(all_cups()):
        part(g, 'Sleeve %d' % (k + 1), mp.tube((x, y, L['sleeve'][0]), (x, y, L['sleeve'][1]), *L['sleeve_r']),
             'Aluminum')
        part(g, 'Spring %.1fx%.0f %d' % (SP['d'], SP['D'], k + 1), moved(mp.tbm().copy(spring_tb), x, y), 'Steel')
    m0, m1 = L['manifold']                                       # -116, -76
    mx, my = L['manifold_xy']
    e = 0
    for k, (zx, zy) in enumerate(zone_centres()):
        part(g, 'Manifold %d' % (k + 1), mp.box(zx - mx / 2, zx + mx / 2, zy - my / 2, zy + my / 2, m0, m1), 'Aluminum')
        for s in (-1, 1):
            e += 1
            xa, xb = sorted((zx + s * EJ_GAP, zx + s * (EJ_GAP + EJ['L'])))
            part(g, 'Ejector %s %d' % (EJ['name'], e),
                 mp.box(xa, xb, zy - EJ['H'] / 2, zy + EJ['H'] / 2, m1, m1 + EJ['B']), 'ABS Plastic')
    return g


def plunger_body(x, y):
    """The plunger 16 x 2 from its foot to the collar, the collar 24 at its top and the spring's seat 24 x 4."""
    ro, ri = L['plunger_r']
    b = zcyl(L['plunger_low'], L['collar'][0], ro, x, y)
    mp.union(b, zcyl(L['collar'][0], L['collar'][1], L['collar_r'], x, y),
             zcyl(L['seat'], L['seat'] + L['seat_t'], L['seat_r'], x, y))
    return mp.cut(b, zcyl(L['plunger_low'] - 1, L['collar'][1] + 1, ri, x, y))


def cup_body(x, y):
    """The SPB1 80 ED-65 bellows cup on the plunger's foot: its connection d1 x H2, one fold out to d2 and in again,
    the lip's flare to D1 on the lip's plane; a rubber wall, open at the lip, the plunger's bore through its top."""
    zt, zl = L['cup_top'], L['cup_lip']
    zc = zt - CUP['H2']
    zf = zc - 0.2 * (zc - zl)                    # the fold
    zw = zc - 0.4 * (zc - zl)                    # the waist
    r1, rf, rl = CUP['d1'] / 2, CUP['d2'] / 2, CUP['D1'] / 2
    rw = rf - 6.0
    w = CUP_WALL
    b = zcyl(zt, zc, r1, x, y)
    mp.union(b, cone(x, y, zc, r1, zf, rf), cone(x, y, zf, rf, zw, rw), cone(x, y, zw, rw, zl, rl))
    slope = (rl - rw) / (zw - zl)
    inner = zcyl(zt + 1, zc - 0.5, L['plunger_r'][1], x, y)
    mp.union(inner, cone(x, y, zc, r1 - w, zf, rf - w), cone(x, y, zf, rf - w, zw, rw - w),
             cone(x, y, zw, rw - w, zl - 1.0, rl - w + slope))
    return mp.cut(b, inner)


def zone(root, k, zx, zy):
    g = group(root, 'Zone %d' % k)
    for j, (x, y) in enumerate(zone_cups(zx, zy)):
        i = (k - 1) * 6 + j + 1
        part(g, 'Plunger %d' % i, plunger_body(x, y), 'Steel')
        part(g, 'Cup %s %d' % (CUP['name'], i), cup_body(x, y), 'Rubber')
    return g


def pack(root, n):
    """n cartons, one under each of zones 1 to n, their tops on the cups' lips."""
    g = group(root, 'Pack')
    c = vg.CARTON
    lx, ly, h = c['l'] * 1000, c['w'] * 1000, c['h'] * 1000
    top = L['cup_lip']
    for k, (zx, zy) in enumerate(zone_centres()[:n]):
        part(g, 'Carton %d' % (k + 1), mp.box(zx - lx / 2, zx + lx / 2, zy - ly / 2, zy + ly / 2, top - h, top), 'Paper')
    return g


# ---------- the states ----------
def place(occ, dx=0.0, dy=0.0, dz=0.0):
    m = adsk.core.Matrix3D.create()
    m.translation = V3(dx * CM, dy * CM, dz * CM)
    occ.transform2 = m


def set_state(G_, lift=None):
    """Every zone (its plungers and cups) and the pack raised by `lift` mm, as cartons push the cups up the
    compensators' stroke; set_state(lift) alone works on the last build's groups."""
    if lift is None:
        G_, lift = _last['groups'], G_
    for k in range(1, N_ZONES + 1):
        place(G_['Zone %d' % k], dz=lift)
    place(G_['Pack'], dz=lift)
    adsk.doEvents()


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0 or o.fullPathName.startswith('Pack'):
            continue
        nm = o.component.name
        if nm.endswith(')') and ' (' in nm:
            nm = nm[:nm.rindex(' (')]
        wds = nm.split()
        if wds and wds[-1].isdigit():
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


# ---------- the rules ----------
def gripper_mass():
    """The gripper's mass for the payload, as vacuum_gripper.design() takes it: Fusion's weighing if written, else
    70 kg."""
    try:
        gw = json.load(open(os.path.join(os.path.dirname(os.path.abspath(vg.__file__)), 'vacuum_gripper_weighed.json')))
        return gw['total']['m'], 'as weighed'
    except (OSError, ValueError, KeyError, TypeError):
        return 70.0, 'estimated'


_ACC = {}


def accelerations(payload):
    k = round(payload, 6)
    if k not in _ACC:
        _ACC[k] = vg.accelerations(payload)
    return _ACC[k]


def refuse(n, m, p_bar):
    if n > N_ZONES:
        raise ValueError('%d cartons: the gripper has six zones, one over each carton of a layer of 3 x 2, so it takes '
                         'six cartons at most.' % n)
    if n < 1:
        raise ValueError('%d cartons: put at least one carton under one of the six zones.' % n)
    if p_bar < vg.P_U - 1e-9:
        raise ValueError('A vacuum of %.2f bar: the cups\' forces (%.0f N pulling, %.0f N sideways) are worked at '
                         '%.1f bar, so the system must hold %.1f bar or more.'
                         % (p_bar, CUP['F'], CUP['F_lat'], vg.P_U, vg.P_U))
    if m <= 0:
        raise ValueError('A carton of %.2f kg: give its mass.' % m)
    mg, how = gripper_mass()
    payload = vg.cg.M_CHANGER + mg + n * m
    acc = accelerations(payload)
    h = vg.holding(vg.N_CUP, m, acc)
    sc = acc['stop_case']
    out = dict(mg=mg, how=how, payload=payload, acc=acc, hold=h, stop='J%s stopped at the %s' % (sc['joint'], sc['pose']))
    if h['u'] > 1.0:
        raise ValueError('A carton of %.2f kg, %d under the gripper (payload %.1f kg): its zone\'s six cups would be '
                         'used to %.3f of what they hold (S = %.1f) in the worst stop (%s) and %.3f on the cycle, over '
                         '1, so the cups do not hold it.'
                         % (m, n, payload, h['u'], vg.S, out['stop'], h['u_cycle']))
    return out


_last = {}


def build(n=6, m=6.5, p_bar=0.6):
    n = int(round(n))
    r = refuse(n, m, p_bar)
    if abs(CGS.L['adapter'][1] - L['adapter'][1]) > 1e-6:
        raise RuntimeError('The clamp gripper\'s adapter ends at z %.1f, the vacuum gripper\'s beams start at z %.1f.'
                           % (CGS.L['adapter'][1], L['adapter'][1]))
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    G_ = {'Adapter': adapter(root)}
    tb = spring_body(root)
    G_['Frame'] = frame(root, tb)
    for k, (zx, zy) in enumerate(zone_centres()):
        G_['Zone %d' % (k + 1)] = zone(root, k + 1, zx, zy)
    G_['Pack'] = pack(root, n)
    mp.colour(des, G_['Adapter'], 'Paint - Enamel Glossy (Blue)')
    mp.colour(des, G_['Frame'], 'Paint - Enamel Glossy (Grey)')
    for k in range(1, N_ZONES + 1):
        mp.colour(des, G_['Zone %d' % k], 'Paint - Enamel Glossy (Yellow)')
    set_state(G_, 0.0)
    _last.update(groups=G_, n=n, m=m, p_bar=p_bar, rules=r)
    app.activeViewport.fit()
    pl = parts_list(root)
    h, acc, s = r['hold'], r['acc'], vg.spring()
    return ('Vacuum gripper built on %d carton%s of %.1f kg (zones 1 to %d), the vacuum held at %.2f bar; the cups\' '
            'lips at z %.1f mm on the cartons\' tops.\n'
            'The cups\' use in the worst stop: %.3f (S = %.1f; %s, %.2f m/s2 up with g, %.2f m/s2 sideways); on the '
            'cycle %.3f. Payload %.1f kg (the changer %.1f, the gripper %.1f %s, the cartons %.1f).\n'
            'Each zone: six %s (%.0f N at -%.1f bar, %.0f N lateral) on spring compensators (music wire %.1f x %.0f, '
            '%.0f N hanging, %.0f N at the end of the %.0f mm stroke, %.1f mm installed), a manifold %.0f x %.0f x %.0f '
            'and two %s.\n\nParts: %d components; %s.\n\n[%s]'
            % (n, '' if n == 1 else 's', m, n, p_bar, L['cup_lip'], h['u_stop'], vg.S, r['stop'], acc['stop_v'],
               acc['stop_h'], h['u_cycle'], r['payload'], vg.cg.M_CHANGER, r['mg'], r['how'], n * m, CUP['name'],
               CUP['F'], vg.P_U, CUP['F_lat'], SP['d'], SP['D'], SP['F1'], SP['F2'], L['stroke'], s['L1'],
               L['manifold_xy'][0], L['manifold_xy'][1], L['manifold'][1] - L['manifold'][0], EJ['name'],
               sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('n', 'Cartons under the gripper', 6.0), ('m', 'A carton\'s mass (kg)', 6.5),
          ('p_bar', 'The vacuum the system holds (bar, 0.6 or more)', 0.6)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = {k: ins.itemById(k).value for k, _, _ in FIELDS}
            ui.messageBox(build(**vals), VERSION)
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
        cid = 'VacuumGripperPromptTestCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Vacuum gripper',
                                                         'The palletizer\'s vacuum gripper of Chapter 46')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('VacuumGripper could not start:\n' + traceback.format_exc())
