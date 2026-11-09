# =============================================================================
# StudyAssemblies.py  -  Chapters 59 and 62 of "Mechanical Design in Autodesk Fusion with Claude": the real parts the
# Simulation studies of Chapters 59 and 62 are run on, each with the neighbours that carry its load. It builds the machine as
# its detail chapter left it (through RangeSweep.py), takes the part and its neighbours out of it, and puts them in a
# design of their own, one component each, with their materials, where they sit in the machine:
#   stop            the bag gripper's opening stop of jaw A: the bracket P13-018 with Chapter 62's change (the arm
#                   turned, the root rounded 2.5 mm), the DYSC-25 in its M30 thread, its lock nut and the end plate
#   stop_diagonal   the same with the arm as Chapter 56 made it, on its diagonal, and the same 2.5 mm root
#   fitting         the flap actuator's spar fitting F01-022, its two GE 20 ES, the actuator's two trunnions and the
#                   rear spar web for 200 mm either side of the fitting
#   hinge           the flap's right hinge: the hinge bracket, its rib, the GE 20 ES in its eye, the hinge pin and the
#                   flap's two hinge fittings on it
#   arms            Chapter 59: the palletizer's lower arm (without J2's reducer), its upper arm and its wrist down to
#                   the tool flange, at the cycle's high point, for the arms' sideways give at the flange. The robot's
#                   model ends the upper arm's box open beside the front fork's cheeks, which then touch it only along
#                   an edge; for the study only, the box's last 10 mm become an end plate across the box and the
#                   cheeks, the welded closure that joins them
#   arms_closed     the same with the two diaphragms the lower arm's box lacks, one at each end of the box, for the
#                   study only: Chapter 59's second study of the arms
# The bolts are left out: the study puts a bolt connector with the bolt's preload through each pair of holes. The script
# prints each part's volume and material and the points where the study's supports, connectors and loads go.
# Checked by checks/study_assemblies_check.py.
#
#   INPUTS        the dialog: the assembly
#   DERIVED       lugs_brackets (the stop's layout and change), flap_design (the fitting's and hinge's loads),
#                 range_sweep (the opening stop's layout), StopBracket.change (the arm turned)
#   HARD RULES    every body copied from the machine's own model, none rebuilt, but the arms' study-only plates;
#                 nothing saved
#   CONVENTIONS   a new design document per assembly; mm, the API's cm inside
#   ACCEPTANCE    the message: the parts, their volumes and materials, the picks
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
SCRIPTS = os.path.dirname(HERE)
LIB = os.path.join(os.path.dirname(SCRIPTS), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import mdfc_parts as mp          # noqa: E402
import range_sweep as RS         # noqa: E402
import flap_design as FD         # noqa: E402

VERSION = 'StudyAssemblies 1.3 (2026-11-09)'
ASSEMBLIES = ('stop', 'stop_diagonal', 'fitting', 'hinge', 'arms', 'arms_closed')
# the arms' parts by their names in the detailed robot: the lower arm without J2's reducer output, the upper arm without
# the pin D's washer and screw, the wrist from its pin to the tool flange without J4's motor and the levelling link
ARMS = ('Fork plate 1', 'Fork plate 2', 'Hub boss J2 side', 'Hub boss J3 side', 'Balancer cross pin Q', 'Lower arm box',
        'Elbow hub', 'Elbow pin',
        'Upper arm cheek 1', 'Upper arm cheek 2', 'Bearing 32213 elbow 1', 'Bearing 32213 elbow 2', 'Yoke bulkhead',
        'Upper arm box', 'Front fork cheek 1', 'Front fork cheek 2', 'Bearing 32213 wrist 1', 'Bearing 32213 wrist 2',
        'Torque tube', 'Rear lever', 'Pin D',
        'Wrist pin', 'Wrist hub', 'Wrist housing', 'RV-80N case J4', 'RV-80N output J4', 'Tool flange')
LATERAL = 1000.0                 # N across the arms' plane, on the tool flange's outer face
END_PLATE = (10.0, 340.0, 200.0, 120.0)   # mm, arms only: the end plate's thickness, its width across the arms' plane
#                                           (the front fork cheeks' outer faces), its depth (the box's), and how far
#                                           short of the wrist pin the upper arm's box ends
WEB_PATCH = 200.0                # mm either side of the fitting's base plate, along the span
_last = {}


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_sa62_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def spec(which):
    """The machine, the parts to take (name, a box in world mm to cut it to or None) and the picks: name -> (point in
    world mm, what goes there)."""
    if which in ('stop', 'stop_diagonal'):
        o = RS.stop_layout()['open']
        x0, x1 = BG_PLATE_X()
        foot = o['foot']['x']
        picks = dict(load=((o['x'], o['face'][0], o['face'][1]), "the DYSC's front face: the push along its axis"))
        r = 4.5                                  # the M8 clearance holes, 9 across, in the foot and the plate
        for k, (y, z) in enumerate(o['bolts']):
            picks['bolt %d foot' % (k + 1)] = ((0.5 * (foot[0] + foot[1]), y + r, z), "the M8 hole in the foot: a bolt connector")
            picks['bolt %d plate' % (k + 1)] = ((0.5 * (x0 + x1), y + r, z), "the M8 hole in the plate: the same connector")
        for k, (y, z) in enumerate(((50.0, -40.0), (-50.0, -40.0), (50.0, -92.0), (-50.0, -92.0))):
            picks['frame %d' % (k + 1)] = ((0.5 * (x0 + x1), y + r, z), "the plate's M8 hole to the frame: fixed")
        return dict(machine='bag', parts=[('Stop bracket open 1', None), ('Shock absorber DYSC-25-25-Y1F open 1', None),
                                          ('Lock nut DYSC-25-25-Y1F open 1', None), ('End plate 1', None)],
                    picks=picks, turned=(which == 'stop'))
    S = FD.SYS
    if which == 'fitting':
        b0, b1 = S['base']
        sp = S['spar']
        x = S['base_x'] + WEB_PATCH
        picks = {}
        r = 4.05                                 # the M8 holes of the fitting and the web
        for k, (bx, bz) in enumerate(S['bolts']):
            picks['bolt %d plate' % (k + 1)] = ((bx + r, 0.5 * (b0 + b1), bz), "the M8 hole in the base plate: a bolt connector")
            picks['bolt %d web' % (k + 1)] = ((bx + r, 0.5 * (sp[0] + sp[1]), bz), "the M8 hole in the web: the same connector")
        picks['web ends'] = ((x, 0.5 * (sp[0] + sp[1]), 0.0), "the web patch's cut ends, both sides: fixed")
        return dict(machine='flap', parts=[('Spar fitting', None), ('Bearing GE 20 ES fitting R', None),
                                           ('Bearing GE 20 ES fitting L', None), ('Trunnion R', None), ('Trunnion L', None),
                                           ('Rear spar web', ((-x, x), (sp[0] - 1, sp[1] + 1), (-200.0, 300.0)))],
                    picks=picks)
    if which == 'hinge':
        h = S['hinges'][1]
        t = S['bracket_t'] / 2
        picks = {}
        r = 4.05
        xr = h + t + S['rib_t'] / 2
        for k, (by, bz) in enumerate(S['bracket_bolts']):
            picks['bolt %d bracket' % (k + 1)] = ((h, by + r, bz), "the M8 hole in the bracket: a bolt connector")
            picks['bolt %d rib' % (k + 1)] = ((xr, by + r, bz), "the M8 hole in the rib: the same connector")
        ym = 0.5 * (S['rib'][0] + S['rib'][1])
        picks['rib top'] = ((xr, ym, FD.skin_up(ym)), "the rib's edge on the upper skin: fixed")
        picks['rib bottom'] = ((xr, ym, FD.skin_low(ym) + 2.0), "the rib's edge on the lower skin: fixed")
        picks['load'] = ((h + S['cheek'][1], 30.0, 0.0), "the outer hinge fitting's face: the flap's hinge load, with the inner's")
        return dict(machine='flap', parts=[('Hinge bracket R', None), ('Hinge rib R', None), ('Bearing GE 20 ES hinge R', None),
                                           ('Hinge pin R', None), ('Hinge fitting Ro', None), ('Hinge fitting Ri', None)],
                    picks=picks)
    if which in ('arms', 'arms_closed'):
        import palletizer_design as PD
        hi = PD.waypoints(PD.brief())['hi']
        p = PD.pts(hi['q2'], hi['q3'])
        ax, az = p['A']
        fx, fz = p['fl']
        picks = {}
        # the hub's bosses, 280 across, from y 190 to 500 either side of the arms' plane: held, as the hand model holds
        # the hub (its bosses rigid)
        picks['hub J2 side'] = ((ax + 140.0, -320.0, az), "the J2 side boss's outside: fixed")
        picks['hub J3 side'] = ((ax + 140.0, 345.0, az), "the J3 side boss's outside: fixed")
        # the tool flange, 30 thick under J4's output: the sideways force on its outer face
        picks['flange'] = ((fx, 60.0, fz - 30.0), "the tool flange's outer face: the force across the arms' plane")
        return dict(machine='robot', pose=(hi['q2'], hi['q3']), parts=[(n, None) for n in ARMS], picks=picks)
    raise ValueError('No assembly called %s: the assemblies are %s.' % (which, ', '.join(ASSEMBLIES)))


def BG_PLATE_X():
    import bag_gripper as BG
    return BG.LAY['plate_x']


def loads(which):
    """The loads each study takes, N, as world vectors: the stop's push along the absorber's axis at the design's
    push and at the DYSC's rating; the fitting's push from the actuator at ultimate, both ways; the hinge's share of
    the flap's ultimate hinge load."""
    if which in ('stop', 'stop_diagonal'):
        n = RS.stop_layout()['open']['axis']
        F = RS.jaw_stops()['open']['F']
        Fmax = RS.DYSC[25]['F_stop']
        return dict(design=(0.0, F * n[0], F * n[1]), rating=(0.0, Fmax * n[0], Fmax * n[1]), F=F, F_max=Fmax)
    d = FD.system_design()
    if which == 'fitting':
        fy, fz = d['ult']['Fv']
        return dict(flight=(0.0, -fy, -fz), reversed=(0.0, fy, fz), F=d['ult']['F'])
    if which == 'hinge':
        ry, rz = d['ult']['R']
        return dict(ultimate=(0.0, -ry / 2, -rz / 2), F=d['R_hinge'])
    if which in ('arms', 'arms_closed'):
        import palletizer_links as PL
        fx, fz = spec('arms')['picks']['flange'][0][0::2]
        h = PL.lateral(at=(fx / 1000.0, fz / 1000.0))
        return dict(lateral=(0.0, LATERAL, 0.0), F=LATERAL, at=(fx, 0.0, fz), c_hand=h['c_flange'],
                    give_hand=h['c_flange'] * LATERAL * 1000.0, c_flange=PL.lateral()['c_flange'])
    raise ValueError(which)


def _material(name):
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    return lib.materials.itemByName(name) if name else None


def take(which):
    """Build the machine, make the chapter's change, and copy the parts' bodies in world mm: [(name, [bodies],
    material name)], with the machine's own volume of each part."""
    sp = spec(which)
    rsm = _load('RangeSweep')
    msg, _setter = rsm.machine(sp['machine'])
    if sp.get('pose'):
        _setter((None, sp['pose']))                # the robot's joints where the study wants them, q2 and q3 in deg
        adsk.doEvents()
    app = adsk.core.Application.get()
    mdoc = app.activeDocument
    try:
        mdoc.attributes.add('mdfc', 'run', 'StudyAssemblies')
    except Exception:
        pass
    des = adsk.fusion.Design.cast(app.activeProduct)
    leaves = [o for o in des.rootComponent.allOccurrences if o.component.bRepBodies.count]
    by = {o.component.name: o for o in leaves}
    if sp['machine'] == 'bag':
        sb = _load('StopBracket')
        sb.TURNED = sp['turned']
        sb.change(by['Stop bracket open 1'], 1, 2.5)
        leaves = [o for o in des.rootComponent.allOccurrences if o.component.bRepBodies.count]
        by = {o.component.name: o for o in leaves}
    tbm = adsk.fusion.TemporaryBRepManager.get()
    out = []
    for name, box in sp['parts']:
        if name not in by:
            raise RuntimeError('no part called %s in the %s' % (name, sp['machine']))
        occ = by[name]
        bodies, mat, vol = [], None, 0.0
        for b in occ.bRepBodies:
            tb = tbm.copy(b)
            if box is not None:
                mp.intersect(tb, mp.box(box[0][0], box[0][1], box[1][0], box[1][1], box[2][0], box[2][1]))
            bodies.append(tb)
            mat = mat or (b.material.name if b.material else None)
            vol += b.volume * 1000.0
        out.append((name, bodies, mat, vol))
    return out, mdoc


PLATE = 'Box end plate (study only)'
DIAPHRAGMS = ('Lower box diaphragm, fork end (study only)', 'Lower box diaphragm, hub end (study only)')


def end_plate(parts):
    """arms only: the upper arm's box shortened by the plate's thickness where it ends, and the end plate in its place,
    across the box and the front fork's cheeks, of the box's own steel. The parts as take() gives them; the plate's
    machine volume is None, for the robot's model has no plate."""
    import palletizer_design as PD
    t, w, d, short = END_PLATE
    q = spec('arms')['pose'] or (0.0, 0.0)          # where take() put the robot: the study's pose, or as built
    p = PD.pts(*q)
    B, W = p['B'], p['W']                            # the elbow's and the wrist's axes, (x, z) in mm
    L = math.hypot(W[0] - B[0], W[1] - B[1])
    u = ((W[0] - B[0]) / L, 0.0, (W[1] - B[1]) / L)
    s_end = L - short                                # where the box ends, from B along the upper arm

    def slab(s0, s1, half_w, half_d):
        c = (B[0] + 0.5 * (s0 + s1) * u[0], 0.0, B[1] + 0.5 * (s0 + s1) * u[2])
        return mp.tbm().createBox(adsk.core.OrientedBoundingBox3D.create(
            mp.P3(c[0] * mp.CM, c[1] * mp.CM, c[2] * mp.CM), mp.V3(*u), mp.V3(0.0, 1.0, 0.0),
            (s1 - s0) * mp.CM, 2.0 * half_w * mp.CM, 2.0 * half_d * mp.CM))
    out, steel = [], None
    for name, bodies, mat, vol in parts:
        if name == 'Upper arm box':
            for tb in bodies:
                mp.cut(tb, slab(s_end - t, s_end + 1.0, w / 2.0 + 30.0, d / 2.0 + 50.0))
            steel = mat
        out.append((name, bodies, mat, vol))
    out.append((PLATE, [slab(s_end - t, s_end, w / 2.0, d / 2.0)], steel, None))
    return out


def diaphragms(parts):
    """arms_closed only: the two plates the lower arm's box lacks. The robot's box (220 deep, 300 wide, 12 wall) is a
    tube open at both ends, from 650 mm above the shoulder to 80 mm short of the elbow's axis; the fork's plates lap
    its side walls at one end and the elbow's hub sits in a saddle at the other. Each diaphragm fills the box's inside,
    12 thick: one at its fork end, from 650 to 662 mm, the other from 122 to 110 mm short of the elbow's axis, against
    the hub; of the box's own steel."""
    import palletizer_design as PD
    t, d, w, wall = 12.0, 220.0, 300.0, 12.0
    q = spec('arms')['pose'] or (0.0, 0.0)          # where take() put the robot: the study's pose, or as built
    p = PD.pts(*q)
    A, B = p['A'], p['B']
    L = math.hypot(B[0] - A[0], B[1] - A[1])
    u = ((B[0] - A[0]) / L, 0.0, (B[1] - A[1]) / L)

    def slab(s0, s1):
        c = (A[0] + 0.5 * (s0 + s1) * u[0], 0.0, A[1] + 0.5 * (s0 + s1) * u[2])
        return mp.tbm().createBox(adsk.core.OrientedBoundingBox3D.create(
            mp.P3(c[0] * mp.CM, c[1] * mp.CM, c[2] * mp.CM), mp.V3(*u), mp.V3(0.0, 1.0, 0.0),
            (s1 - s0) * mp.CM, (w - 2 * wall) * mp.CM, (d - 2 * wall) * mp.CM))
    steel = [m for n, _b, m, _v in parts if n == 'Lower arm box'][0]
    return parts + [(DIAPHRAGMS[0], [slab(650.0, 650.0 + t)], steel, None),
                    (DIAPHRAGMS[1], [slab(L - 110.0 - t, L - 110.0)], steel, None)]


def build(which='stop'):
    if which not in ASSEMBLIES:
        raise ValueError('No assembly called %s: the assemblies are %s.' % (which, ', '.join(ASSEMBLIES)))
    app = adsk.core.Application.get()
    parts, mdoc = take(which)
    if which in ('arms', 'arms_closed'):
        parts = end_plate(parts)
    if which == 'arms_closed':
        parts = diaphragms(parts)
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    try:
        doc.attributes.add('mdfc', 'run', 'StudyAssemblies')
    except Exception:
        pass
    try:
        mdoc.close(False)
    except Exception:
        pass
    doc.activate()
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    rows = []
    for name, bodies, mat, vol in parts:
        occ = mp.new_occ(root, name)
        v = 0.0
        lo, hi = [1e9] * 3, [-1e9] * 3
        for k, tb in enumerate(bodies):
            b = mp.add_body(occ, tb, name if len(bodies) == 1 else '%s %d' % (name, k + 1))
            m = _material(mat)
            if m is not None:
                b.material = m
            v += b.volume * 1000.0
            bb = b.boundingBox
            lo = [min(lo[i], c * 10.0) for i, c in enumerate((bb.minPoint.x, bb.minPoint.y, bb.minPoint.z))]
            hi = [max(hi[i], c * 10.0) for i, c in enumerate((bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z))]
        rows.append(dict(part=name, bodies=len(bodies), material=mat, volume=v, machine_volume=vol, box=(lo, hi)))
    sp = spec(which)
    found = {}
    for key, (p, _what) in sp['picks'].items():
        pt = adsk.core.Point3D.create(p[0] / 10.0, p[1] / 10.0, p[2] / 10.0)
        hits = root.findBRepUsingPoint(pt, adsk.fusion.BRepEntityTypes.BRepFaceEntityType, 0.05, True)
        found[key] = hits.count > 0
    out = dict(assembly=which, parts=rows, picks={k: v[0] for k, v in sp['picks'].items()},
               what={k: v[1] for k, v in sp['picks'].items()}, found=found, loads=loads(which))
    _last.clear()
    _last.update(out)
    lines = ['%s: %d bodies, %s, %.1f mm3%s' % (r['part'], r['bodies'], r['material'], r['volume'],
                                              '' if r['machine_volume'] is None or abs(r['volume'] - r['machine_volume']) < 1e-3
                                              else ' (cut from %.1f)' % r['machine_volume']) for r in rows]
    return ('Study assembly %s:\n%s\npicks: %s\n\n[%s]' % (which, '\n'.join(lines), ', '.join(
        '%s %s' % (k, 'found' if v else 'MISSING') for k, v in found.items()), VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.command.commandInputs.itemById('assembly').selectedItem.name))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('StudyAssemblies failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('assembly', 'Assembly', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k, p in enumerate(ASSEMBLIES):
            dd.listItems.add(p, k == 0)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcStudyAssemblies'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Study assemblies', 'Chapter 62: the real parts studied, with their neighbours')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StudyAssemblies failed:\n%s' % traceback.format_exc())
