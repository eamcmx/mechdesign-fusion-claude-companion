# =============================================================================
# StudyAssemblies.py  -  Chapter 62 of "Mechanical Design in Autodesk Fusion with Claude": the real parts the
# Simulation studies of Chapter 62 are run on, each with the neighbours that carry its load. It builds the machine as
# its detail chapter left it (through RangeSweep.py), takes the part and its neighbours out of it, and puts them in a
# design of their own, one component each, with their materials, where they sit in the machine:
#   stop            the bag gripper's opening stop of jaw A: the bracket P13-018 with Chapter 62's change (the arm
#                   turned, the root rounded 2.5 mm), the DYSC-25 in its M30 thread, its lock nut and the end plate
#   stop_diagonal   the same with the arm as Chapter 56 made it, on its diagonal, and the same 2.5 mm root
#   fitting         the flap actuator's spar fitting F01-022, its two GE 20 ES, the actuator's two trunnions and the
#                   rear spar web for 200 mm either side of the fitting
#   hinge           the flap's right hinge: the hinge bracket, its rib, the GE 20 ES in its eye, the hinge pin and the
#                   flap's two hinge fittings on it
# The bolts are left out: the study puts a bolt connector with the bolt's preload through each pair of holes. The script
# prints each part's volume and material and the points where the study's supports, connectors and loads go.
# Checked by checks/study_assemblies_check.py.
#
#   INPUTS        the dialog: the assembly
#   DERIVED       lugs_brackets (the stop's layout and change), flap_design (the fitting's and hinge's loads),
#                 range_sweep (the opening stop's layout), StopBracket.change (the arm turned)
#   HARD RULES    every body copied from the machine's own model, none rebuilt; nothing saved
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

VERSION = 'StudyAssemblies 1.0 (2026-11-07)'
ASSEMBLIES = ('stop', 'stop_diagonal', 'fitting', 'hinge')
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


def build(which='stop'):
    if which not in ASSEMBLIES:
        raise ValueError('No assembly called %s: the assemblies are %s.' % (which, ', '.join(ASSEMBLIES)))
    app = adsk.core.Application.get()
    parts, mdoc = take(which)
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
        for k, tb in enumerate(bodies):
            b = mp.add_body(occ, tb, name if len(bodies) == 1 else '%s %d' % (name, k + 1))
            m = _material(mat)
            if m is not None:
                b.material = m
            v += b.volume * 1000.0
        rows.append(dict(part=name, bodies=len(bodies), material=mat, volume=v, machine_volume=vol))
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
                                              '' if abs(r['volume'] - r['machine_volume']) < 1e-3 else
                                              ' (cut from %.1f)' % r['machine_volume']) for r in rows]
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
