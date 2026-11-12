# =============================================================================
# VacuumGripper.py  -  Chapter 46 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer's vacuum
# gripper for light cartons and slip sheets. The adapter to ISO 9409-1-160-6-M10 of Chapter 45; two aluminium beams
# and a plate 1240 x 840 under them; over each of the six cartons of a layer a zone of six Schmalz SPB1 80 bellows
# cups on spring level compensators, a manifold and two SCPSi 2-14 compact ejectors. The sizes are
# lib/vacuum_gripper.py's. Written from PROMPT.txt beside it; checked by checks/vacuum_gripper_check.py.
#
#   INPUTS        the dialog: the cartons under the gripper (1 to 6), a carton's mass (kg), the vacuum (bar)
#   DERIVED       vacuum_gripper: CUP, EJECTOR, LAY, SPRING, holding() with the stops of each payload
#   HARD RULES    build() refuses more than six cartons, a carton the cups do not hold through the cycle and the
#                 worst stop at S = 2, and a vacuum under the 0.6 bar the holding is worked at
#   CONVENTIONS   z = 0 the gripper's mounting face, z up; x along the pallet's 1200; groups Adapter and Frame
#                 (grounded), Zone 1 to 6 (the plungers and cups), Pack; set_state() lifts each zone's plungers
#   ACCEPTANCE    the message; the check: no interference, every cup's lip on its carton, the plungers' stroke
#                 clear, the springs measured, the ISO pattern measured, the groups weighed
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import json
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                         # the project's folder: what this script writes goes in its results/
ROOT = PROJECT                                          # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import tool_changer           # noqa: E402
import clamp_gripper          # noqa: E402
import vacuum_gripper as vg   # noqa: E402
import mdfc_parts as mp       # noqa: E402
importlib.reload(tool_changer)                  # the modules this one builds on, as Fusion keeps old ones
importlib.reload(clamp_gripper)
importlib.reload(vg)
importlib.reload(mp)

VERSION = 'VacuumGripper 1.1 (2026-11-12)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

L = vg.LAY
CUP = vg.CUP
SP = vg.SPRING
ISO = (30.0, 90.0, 150.0, 210.0, 270.0, 330.0)
M8 = ((55.0, 55.0), (-55.0, 55.0), (-55.0, -55.0), (55.0, -55.0))


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def ztube(z0, z1, ro, ri, x=0.0, y=0.0):
    return mp.tube((x, y, z0), (x, y, z1), ro, ri)


def at(r, deg):
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


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


def cups_xy():
    """The 36 cups' axes, zone by zone: (zone, x, y)."""
    out = []
    k = 0
    for zy in L['zone_y']:
        for zx in L['zone_x']:
            k += 1
            for dy in L['cup_dy']:
                for dx in L['cup_dx']:
                    out.append((k, zx + dx, zy + dy))
    return out


def zones_xy():
    out = []
    for zy in L['zone_y']:
        for zx in L['zone_x']:
            out.append((zx, zy))
    return out


# ---------- the spring: one coil swept along a helix, then copied ----------
def spring_body(root):
    """A coil spring with its axis on z through the origin, from z = 0 down to -L1, mean diameter D, wire d, Na
    active turns between two closed ground ends: swept once in a scratch occurrence and returned as a temporary body."""
    s = vg.spring()
    D, d = SP['D'], SP['d']
    L1 = s['L1']
    tmp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = tmp.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    pts = adsk.core.ObjectCollection.create()
    turns = s['Na'] + 2.0
    n = int(turns * 12)
    z_top, z_bot = -d / 2, -L1 + d / 2
    for i in range(n + 1):
        t = i / n
        th = 2 * math.pi * turns * t
        # closed ends: the first and last turn at the wire's pitch, the active turns spread over the rest
        u = t * turns
        if u <= 1.0:
            z = z_top - d * u
        elif u >= turns - 1.0:
            z = z_bot + d * (turns - u)
        else:
            z = (z_top - d) + ((z_bot + d) - (z_top - d)) * (u - 1.0) / (turns - 2.0)
        pts.add(P3(D / 2 * math.cos(th) * CM, D / 2 * math.sin(th) * CM, z * CM))
    spl = sk.sketchCurves.sketchFittedSplines.add(pts)
    path = comp.features.createPath(spl, False)
    pi_ = comp.constructionPlanes.createInput()
    pi_.setByDistanceOnPath(path, VI(0.0))
    pl = comp.constructionPlanes.add(pi_)
    sk2 = comp.sketches.add(pl)
    c = sk2.modelToSketchSpace(P3(D / 2 * CM, 0, z_top * CM))
    sk2.sketchCurves.sketchCircles.addByCenterRadius(c, d / 2 * CM)
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk2.profiles.item(0), path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    sw.add(inp)
    tb = mp.tbm().copy(comp.bRepBodies.item(0))
    tmp.deleteMe()
    return tb


def moved(tb, dx, dy, dz):
    b = mp.tbm().copy(tb)
    m = adsk.core.Matrix3D.create()
    m.translation = V3(dx * CM, dy * CM, dz * CM)
    mp.tbm().transform(b, m)
    return b


# ---------- the parts ----------
def adapter(root):
    g = group(root, 'Adapter', True)
    b = zcyl(L['adapter'][1], 0.0, 100.0)
    mp.union(b, zcyl(0.0, 8.0, 49.99))
    for a in ISO:
        x, y = at(80.0, a)
        mp.cut(b, zcyl(-17.0, 1.0, 5.5, x, y))
        mp.cut(b, zcyl(-17.0, -5.5, 8.75, x, y))
    mp.cut(b, zcyl(-8.0, 9.0, 5.0, 80.0, 0.0))
    for x, y in M8:
        mp.cut(b, zcyl(-17.0, -1.0, 4.0, x, y))
    part(g, 'Adapter plate', b, 'Aluminum')
    for k, a in enumerate(ISO):
        x, y = at(80.0, a)
        part(g, 'Cap screw M10x25 %d' % (k + 1), mp.cap_screw(10.0, 25.0, (x, y, -5.5), (0, 0, 1)))
    part(g, 'Dowel pin 10x17', zcyl(-8.0, 9.0, 5.0, 80.0, 0.0))
    return g


def frame(root):
    g = group(root, 'Frame', True)
    zt, zb = L['beam']
    t = L['beam_t']
    hl = L['beam_len'] / 2
    ps = L['plate_screw']
    for k, s in enumerate((1, -1)):
        y0, y1 = sorted((s * (L['beam_y'] - L['beam_w'] / 2), s * (L['beam_y'] + L['beam_w'] / 2)))
        b = mp.box(-hl, hl, y0, y1, zb, zt)
        mp.cut(b, mp.box(-hl - 1, hl + 1, y0 + t, y1 - t, zb + t, zt - t))
        for x, y in M8:
            if s * y > 0:
                mp.cut(b, zcyl(zt - t - 1, zt + 1, 4.5, x, y))
                mp.cut(b, zcyl(zb - 1, zb + t + 1, 10.0, x, y))
        yb = s * L['beam_y']
        for x in ps['x']:
            mp.cut(b, zcyl(zt - t - 1, zt + 1, ps['access_r'], x, yb))      # the way in to the plate's M6
            mp.cut(b, zcyl(zb - 1, zb + t + 1, ps['clear_r'], x, yb))
        part(g, 'Beam %d' % (k + 1), b, 'Aluminum')
    for k, (x, y) in enumerate(M8):
        part(g, 'Cap screw M8x20 %d' % (k + 1), mp.cap_screw(8.0, 20.0, (x, y, zt - t), (0, 0, 1)))
    p0, p1 = L['plate']
    pb = mp.box(-L['plate_l'] / 2, L['plate_l'] / 2, -L['plate_w'] / 2, L['plate_w'] / 2, p1, p0)
    for _, x, y in cups_xy():
        mp.cut(pb, zcyl(p1 - 1, p0 + 1, L['sleeve_r'][1], x, y))
    for x, y in M8:
        mp.cut(pb, zcyl(p1 - 1, p0 + 1, 10.0, x, y))                    # the way in to the beams' M8
    k = 0
    for s in (1, -1):
        for x in ps['x']:
            yb = s * L['beam_y']
            mp.cut(pb, zcyl(zb + t - ps['L'] - 0.5, p0 + 1, ps['d'] / 2, x, yb))   # tapped 9 mm, blind
            k += 1
            part(g, 'Cap screw M6x14 %d' % k, mp.cap_screw(ps['d'], ps['L'], (x, yb, zb + t), (0, 0, -1)))
    part(g, 'Plate', pb, 'Aluminum')
    sp = spring_body(root)
    for k, (zn, x, y) in enumerate(cups_xy()):
        part(g, 'Sleeve %d' % (k + 1), ztube(L['sleeve'][0], L['sleeve'][1], L['sleeve_r'][0], L['sleeve_r'][1], x, y),
             'Aluminum')
        part(g, 'Spring %d' % (k + 1), moved(sp, x, y, p1))
    mx, my = L['manifold_xy']
    for k, (zx, zy) in enumerate(zones_xy()):
        m = mp.box(zx - mx / 2, zx + mx / 2, zy - my / 2, zy + my / 2, L['manifold'][0], L['manifold'][1])
        part(g, 'Manifold %d' % (k + 1), m, 'Aluminum')
        ej = vg.EJECTOR
        for e, sx in enumerate((-1, 1)):
            x0 = zx + sx * 2.0
            x1 = zx + sx * (2.0 + ej['L'])
            # lying on its side on the manifold: its length along x, its height along y, its width up
            part(g, 'Ejector SCPSi 2-14 %d' % (2 * k + e + 1),
                 mp.box(min(x0, x1), max(x0, x1), zy - ej['H'] / 2, zy + ej['H'] / 2, L['manifold'][1],
                        L['manifold'][1] + ej['B']), 'ABS Plastic')
    return g


def cup_body(x, y):
    """The SPB1 80 drawn round from its data sheet: the connector's head Dk over H2, the bellows at d2, the lip
    opening to D1 at the bottom; its top at the plunger's end."""
    zt = L['cup_top']
    b = zcyl(zt - CUP['H2'], zt, 27.1 / 2, x, y)
    mp.union(b, zcyl(zt - CUP['H'] + 8.0, zt - CUP['H2'], CUP['d2'] / 2, x, y))
    lip = mp.tbm().createCylinderOrCone(P3(x * CM, y * CM, (zt - CUP['H'] + 8.0) * CM), CUP['d2'] / 2 * CM,
                                        P3(x * CM, y * CM, (zt - CUP['H']) * CM), CUP['D1'] / 2 * CM)
    mp.union(b, lip)
    mp.cut(b, zcyl(zt - CUP['H'] - 1.0, zt - CUP['H2'] - 3.0, CUP['d1'] / 2, x, y))
    return b


def zone(root, k):
    g = group(root, 'Zone %d' % k)
    for i, (zn, x, y) in enumerate([c for c in cups_xy() if c[0] == k]):
        pl = ztube(L['plunger_low'], L['collar'][1], L['plunger_r'][0], L['plunger_r'][1], x, y)
        mp.union(pl, ztube(L['collar'][0], L['collar'][1], L['collar_r'], L['plunger_r'][1], x, y))
        mp.union(pl, ztube(L['seat'], L['seat'] + L['seat_t'], L['seat_r'], L['plunger_r'][1], x, y))
        part(g, 'Plunger %d-%d' % (k, i + 1), pl)
        part(g, 'Cup SPB1 80 %d-%d' % (k, i + 1), cup_body(x, y), 'Rubber')
    return g


def pack(root, n):
    g = group(root, 'Pack')
    c = vg.CARTON
    lx, ly, h = c['l'] * 1000, c['w'] * 1000, c['h'] * 1000
    z1 = L['cup_lip']
    for k, (zx, zy) in enumerate(zones_xy()[:n]):
        part(g, 'Carton %d' % (k + 1), mp.box(zx - lx / 2, zx + lx / 2, zy - ly / 2, zy + ly / 2, z1 - h, z1), 'Paper')
    return g


# ---------- the states ----------
def place(occ, dx=0.0, dy=0.0, dz=0.0):
    m = adsk.core.Matrix3D.create()
    m.translation = V3(dx * CM, dy * CM, dz * CM)
    occ.transform2 = m


def set_state(G_, lift=0.0, zones=None, pack_dz=None):
    """Each zone's plungers pushed up by `lift` mm (as by cartons that much taller), the pack raised with them."""
    for k in range(1, 7):
        place(G_['Zone %d' % k], dz=lift if (zones is None or k in zones) else 0.0)
    place(G_['Pack'], dz=lift if pack_dz is None else pack_dz)
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
        if wds and (wds[-1].isdigit() or (('-' in wds[-1]) and wds[-1].replace('-', '').isdigit())):
            wds = wds[:-1]                                                   # the part's own number
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


def refuse(n, m, p_bar):
    n = int(round(n))
    if n < 1 or n > 6:
        raise ValueError('%d cartons: the gripper has six zones, one carton under each.' % n)
    if p_bar < vg.P_U - 1e-9:
        raise ValueError('A vacuum of %.2f bar: the cups\' holding is worked at %.1f bar; less holds less.'
                         % (p_bar, vg.P_U))
    mg = vg.design()['m_gripper']
    pl = clamp_gripper.M_CHANGER + mg + n * m
    h = vg.holding(vg.N_CUP, m, vg.accelerations(pl))
    if h['u'] > 1.0:
        raise ValueError('A carton of %.1f kg on six cups, %d under the gripper: the cups would be used %.2f times over '
                         'in the worst stop at S = %.1f.' % (m, n, h['u'], vg.S))
    return h


_last = {}


def build(n=6, m=vg.M_CARTON, p_bar=vg.P_U):
    n = int(round(n))
    h = refuse(n, m, p_bar)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    G_ = {'Adapter': adapter(root), 'Frame': frame(root)}
    for k in range(1, 7):
        G_['Zone %d' % k] = zone(root, k)
    G_['Pack'] = pack(root, n)
    for k, col in (('Adapter', 'Paint - Enamel Glossy (Blue)'), ('Frame', 'Paint - Enamel Glossy (Grey)')):
        mp.colour(des, G_[k], col)
    for k in range(1, 7):
        mp.colour(des, G_['Zone %d' % k], 'Paint - Enamel Glossy (Yellow)')
    set_state(G_, 0.0)
    _last.update(groups=G_, n=n, m=m)
    app.activeViewport.fit()
    pl = parts_list(root)
    return ('Vacuum gripper built on %d cartons of %.1f kg.\nSix SPB1 80 cups over each, on spring compensators of %.0f mm; '
            'two SCPSi 2-14 per zone; the cups used %.2f in the worst stop at S = %.1f.\n\nParts: %d components; %s.\n\n[%s]'
            % (n, m, L['stroke'], h['u'], vg.S, sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())),
               VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('n', 'Cartons under the gripper', 6.0), ('m', 'A carton\'s mass (kg)', vg.M_CARTON),
          ('p_bar', 'The vacuum (bar)', vg.P_U)]


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
        cid = 'VacuumGripperCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Vacuum gripper', 'The palletizer\'s vacuum gripper of Chapter 46')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('VacuumGripper could not start:\n' + traceback.format_exc())
