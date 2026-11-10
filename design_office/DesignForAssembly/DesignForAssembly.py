# =============================================================================
# DesignForAssembly.py  -  Chapter 65 of "Mechanical Design in Autodesk Fusion with Claude": the bag gripper through
# Boothroyd and Dewhurst's test for unnecessary parts, in the model. It builds the gripper through RangeSweep.py as
# Parts 9 and 10 left it, takes Fusion's census, gives every piece the test's answer from lib/dfma.py as an attribute
# (the question that keeps it, or "candidate"), colours the pieces by the answer, and reads the answers back to count
# the pieces, the theoretical minimum and the assembly time. Then it makes the three fixings the test found missing:
# the tines joined to their jaw's tube (the jaw weldment), the end blocks joined to the end plates with the eight
# M8 x 30 between them gone and their holes filled, two M8 x 20 through the beam's side walls into each spigot, and two
# M8 x 30 from inside each end plate into its cylinder bracket, every hole cut; checks the new screws for interference,
# counts again and writes results/dfa.json. The design stays open.
# Written from PROMPT.txt beside it; checked by checks/design_for_assembly_check.py.
#
#   INPUTS        the dialog: the machine (bag, the only one the test is answered for)
#   DERIVED       lib/dfma.py (the answers, the times, the fixings), lib/bom_detail.py (the lines)
#   HARD RULES    nothing of the machines' scripts copied; built through RangeSweep.machine(); nothing saved
#   CONVENTIONS   a new design document; mm and N, the API's cm inside
#   ACCEPTANCE    the message; the check: the answers read back against the lib, the fixings in the model, no
#                 interference of the new screws, the counts after the fixings
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import io
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

import bom_detail as BD          # noqa: E402
import dfma as DF                # noqa: E402
import mdfc_parts as mp          # noqa: E402
import bag_gripper as BG         # noqa: E402
for _m in (BD, DF):
    importlib.reload(_m)

VERSION = 'DesignForAssembly 1.0 (2026-11-04)'
CM = 0.1
GROUP = 'mdfc_dfa'
COLOURS = dict(essential='Paint - Enamel Glossy (Blue)', candidate='Paint - Enamel Glossy (Red)',
               unfixed='Paint - Enamel Glossy (Yellow)', new='Paint - Enamel Glossy (Yellow)')
_last = {}


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_dfa65_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def census(des):
    rows, occ = [], {}
    for o in des.rootComponent.allOccurrences:
        if not o.bRepBodies.count:
            continue
        mats = ', '.join(sorted({b.material.name for b in o.bRepBodies if b.material}))
        rows.append((o.component.name, sum(b.physicalProperties.mass for b in o.bRepBodies), mats,
                     sum(b.volume for b in o.bRepBodies)))
        occ.setdefault(o.component.name, []).append(o)
    return rows, occ


def lines_of(rows):
    """The first-level lines of the bill (Chapter 64), each with its pieces' component names."""
    b = BD.finish(BD.bom('bag', rows))
    out = {}
    for g in b['lines']:
        if not g['number'] or g['kind'].startswith('in ') or g['kind'].startswith('with'):
            continue
        names = sorted(n for v in g['names'].values() for n, p in v)
        out[g['number']] = dict(qty=g['qty'], names=names, kind=g['kind'])
    return b, out


def tag(des, occ, lines):
    """Each piece's answer as an attribute and a colour; the weldment's pieces take the weldment's answer."""
    test = {l[0]: l for l in DF.BAG}
    unfixed = {u[0] for u in DF.UNFIXED}
    weld = {n for n, w in BD.WELDMENTS.items()}
    tagged = []
    for num, g in lines.items():
        t = test.get(num)
        if not t:
            continue
        names = g['names']
        if num in weld:                                           # the weldment: its tubes stand for the pieces
            names = sorted(n for n in occ if n.startswith('Tube '))
        ess = t[4]
        for k, name in enumerate(names):
            ans = t[5] if k < ess else 'candidate'
            for o in occ.get(name, []):
                o.attributes.add(GROUP, 'answer', ans)
                o.attributes.add(GROUP, 'number', num)
                col = 'unfixed' if num in unfixed else ('candidate' if ans == 'candidate' else 'essential')
                mp.colour(des, o, COLOURS[col])
                tagged.append((num, name, ans))
                if num in weld:                                   # the stubs and welds of the same jaw
                    jaw = name.split()[-1]
                    for other, os_ in occ.items():
                        if other.endswith(' ' + jaw) and (other.startswith('Stub shaft') or other.startswith('Fillet weld')):
                            for o2 in os_:
                                mp.colour(des, o2, COLOURS['essential'])
    return tagged


def read_back(des):
    """The answers as the model holds them: pieces, essential pieces by question."""
    pieces, ess, by_q = {}, 0, {}
    for o in des.rootComponent.allOccurrences:
        a = o.attributes.itemByName(GROUP, 'answer')
        n = o.attributes.itemByName(GROUP, 'number')
        if not a or not n:
            continue
        pieces[n.value] = pieces.get(n.value, 0) + 1
        if a.value != 'candidate':
            ess += 1
            by_q[a.value] = by_q.get(a.value, 0) + 1
    return dict(pieces=pieces, N=sum(pieces.values()), Nmin=ess, by_q=by_q)


# ---------- the three fixings ----------
def _native(o):
    return o.component.bRepBodies.item(0)


def _world(o):
    tb = mp.tbm().copy(_native(o))
    mp.tbm().transform(tb, mp.world_matrix(o))
    return tb


def _replace(o, tb):
    """The occurrence's body replaced by tb (world coordinates), in its base feature."""
    m = mp.world_matrix(o)
    m.invert()
    mp.tbm().transform(tb, m)
    name = _native(o).name
    for bf in o.component.features.baseFeatures:
        if bf.bodies.count and bf.bodies.item(0).name == name:
            bf.startEdit()
            bf.updateBody(bf.bodies.item(0), tb)
            bf.finishEdit()
            bf.bodies.item(0).name = name
            return
    raise RuntimeError('no base feature for %s' % o.component.name)


def xcyl(x0, x1, r, y, z):
    return mp.cyl([x0, y, z], [x1, y, z], r)


def ycyl(y0, y1, r, x, z):
    return mp.cyl([x, y0, z], [x, y1, z], r)


def _turn(tb):
    b = mp.tbm().copy(tb)
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.pi, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    mp.tbm().transform(b, m)
    return b


def fix(des, occ):
    L = BG.LAY
    root = des.rootComponent
    done = dict(tines=0, blocks=0, removed=[], added=[])
    # 1. the tines joined to their jaw's tube: the jaw weldment grows by its eight tines
    for jaw in ('A', 'B'):
        tube = occ['Tube ' + jaw][0]
        tb = _world(tube)
        for k in range(1, 9):
            for o in occ.get('Tine %d %s' % (k, jaw), []):
                mp.union(tb, _world(o))
                o.deleteMe()
                done['tines'] += 1
        _replace(tube, tb)
    # 2. the end blocks joined to the end plates, the eight M8 x 30 between them gone and their holes filled
    hx = L['beam_x']
    holes = ((50.0, -40.0), (-50.0, -40.0), (50.0, -92.0), (-50.0, -92.0))
    for k, s in ((1, 1), (2, -1)):
        plate = occ['End plate %d' % k][0]
        tb = _world(plate)
        for o in occ.get('End block %d' % k, []):
            mp.union(tb, _world(o))
            o.deleteMe()
            done['blocks'] += 1
        for y, z in holes:
            c = xcyl(hx - 16.0, L['plate_x'][1], 4.5, y, z)
            mp.union(tb, c if s > 0 else _turn(c))
        _replace(plate, tb)
    for k in range(1, 9):
        for o in occ.get('Cap screw M8x30 %d' % k, []):
            done['removed'].append(o.component.name)
            o.deleteMe()
    # 3. two M8 x 20 through the beam's side walls into each spigot, at the spigot's mid-depth and the beam's mid-height
    beam = occ['Beam'][0]
    btb = _world(beam)
    xs, zs = hx - L['block'] / 2.0, (L['beam'][0] + L['beam'][1]) / 2.0
    yw = L['beam_y']
    plates = {k: occ['End plate %d' % k][0] for k in (1, 2)}
    ptb = {k: _world(o) for k, o in plates.items()}
    group = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    group.component.name = 'Fixings (Chapter 65)'
    n = 0
    for k, s in ((1, 1), (2, -1)):
        for side in (1, -1):
            n += 1
            wall = ycyl(side * (yw - L['beam_t'] - 0.01), side * (yw + 0.01), 4.5, xs, zs)
            tap = ycyl(side * (yw - 20.0), side * (yw - L['beam_t']), 4.0, xs, zs)
            sc = mp.cap_screw(8.0, 20.0, (xs, side * yw, zs), (0, -side, 0))
            if s < 0:
                wall, tap, sc = _turn(wall), _turn(tap), _turn(sc)
            mp.cut(btb, wall)
            mp.cut(ptb[k], tap)
            o = mp.new_occ(group.component, 'Cap screw M8x20 spigot %d' % n)
            mp.add_body(o, sc, 'Cap screw M8x20 spigot %d' % n, 'Steel')
            mp.colour(des, o, COLOURS['new'])
            done['added'].append(o.component.name)
    _replace(beam, btb)
    # 4. two M8 x 30 from inside each end plate into its cylinder bracket's root, 30 apart up the arm
    YP, ZP = BG.cylinder_geometry()['pin']
    yb = YP - 42.5
    n = 0
    for k, s in ((1, 1), (2, -1)):
        br = occ['Bracket %d' % k][0]
        btb2 = _world(br)
        for dz in (-15.0, 15.0):
            n += 1
            z = ZP + dz
            hole = xcyl(L['plate_x'][0] - 0.01, L['plate_x'][1] + 0.01, 4.5, yb, z)
            tap = xcyl(L['plate_x'][1], L['plate_x'][1] + 16.0, 4.0, yb, z)
            sc = mp.cap_screw(8.0, 30.0, (L['plate_x'][0], yb, z), (1, 0, 0))
            if s < 0:
                hole, tap, sc = _turn(hole), _turn(tap), _turn(sc)
            mp.cut(ptb[k], hole)
            mp.cut(btb2, tap)
            o = mp.new_occ(group.component, 'Cap screw M8x30 bracket %d' % n)
            mp.add_body(o, sc, 'Cap screw M8x30 bracket %d' % n, 'Steel')
            mp.colour(des, o, COLOURS['new'])
            done['added'].append(o.component.name)
        _replace(br, btb2)
    for k, o in plates.items():
        _replace(o, ptb[k])
    return done, group


def interference(des, group):
    """The new screws against every other piece of the gripper (the bag and the station left out)."""
    coll = adsk.core.ObjectCollection.create()
    for o in des.rootComponent.allOccurrences:
        if o.bRepBodies.count and not o.fullPathName.startswith(('Bag', 'Station')):
            coll.add(o)
    inp = des.createInterferenceInput(coll)
    inp.areCoincidentFacesIncluded = False
    res = des.analyzeInterference(inp)
    new = {o.component.name for o in group.childOccurrences}
    hits = []
    for r in res:
        a, b = r.entityOne, r.entityTwo
        na = a.assemblyContext.component.name if a.assemblyContext else a.parentComponent.name
        nb = b.assemblyContext.component.name if b.assemblyContext else b.parentComponent.name
        if na in new or nb in new:
            hits.append((na, nb, r.interferenceBody.volume * 1000.0))
    return hits, res.count


def build(which='bag'):
    if which != 'bag':
        raise ValueError('No answers for %s: Chapter 65 answers the test for the bag gripper.' % which)
    app = adsk.core.Application.get()
    RS = _load('RangeSweep')
    msg, _ = RS.machine('bag')
    doc = app.activeDocument
    try:
        doc.attributes.add('mdfc', 'run', 'DesignForAssembly')
    except Exception:
        pass
    des = adsk.fusion.Design.cast(app.activeProduct)
    rows, occ = census(des)
    b0, lines0 = lines_of(rows)
    tagged = tag(des, occ, lines0)
    before = read_back(des)
    lib = DF.bag_test()
    m0 = des.rootComponent.physicalProperties.mass
    done, group = fix(des, occ)
    hits, n_all = interference(des, group)
    rows2, occ2 = census(des)
    b1, lines1 = lines_of(rows2)
    m1 = des.rootComponent.physicalProperties.mass
    after_pieces = sum(g['qty'] for g in lines1.values())
    out = dict(version=VERSION, tagged=len(tagged), before=before, lib=dict(N=lib['N'], Nmin=lib['Nmin'],
               t=lib['t'], E=lib['E']), lines_before={k: v['qty'] for k, v in lines0.items()},
               lines_after={k: v['qty'] for k, v in lines1.items()}, pieces_after=after_pieces, fixed=done,
               interference=hits, interferences_all=n_all, mass_before=m0, mass_after=m1,
               made_after=b1['made_pieces'], bought_after=b1['bought_pieces'])
    json.dump(out, io.open(os.path.join(PROJECT, 'results', 'dfa.json'), 'w', encoding='utf-8'), indent=1, default=str)
    _last.clear()
    _last.update(out)
    return ('Design for assembly: the bag gripper, %d pieces tagged; %d essential (%s); after the three fixings %d '
            'pieces (%d tines and %d end blocks joined, %d screws removed, %d added); the new screws: %d '
            'interferences; %.3f kg to %.3f kg.\n\n[%s]' % (
                before['N'], before['Nmin'], ', '.join('%s %d' % kv for kv in sorted(before['by_q'].items())),
                after_pieces, done['tines'], done['blocks'], len(done['removed']), len(done['added']), len(hits),
                m0, m1, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.command.commandInputs.itemById('machine').selectedItem.name))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('DesignForAssembly failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('machine', 'Machine', adsk.core.DropDownStyles.TextListDropDownStyle)
        dd.listItems.add('bag', True)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcDesignForAssembly'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Design for assembly', 'Chapter 65: the bag gripper\'s test')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('DesignForAssembly failed:\n%s' % traceback.format_exc())
