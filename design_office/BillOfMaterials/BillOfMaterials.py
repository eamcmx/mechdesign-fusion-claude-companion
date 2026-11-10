# =============================================================================
# BillOfMaterials.py  -  Chapter 64 of "Mechanical Design in Autodesk Fusion with Claude": the bill of materials of
# each of the nine assemblies as Parts 9 and 10 left them. It builds each machine through RangeSweep.py, takes Fusion's
# census of every occurrence that has a body (its name, mass, materials and volume), and hands it to lib/bom_detail.py,
# which makes the lines: the part number, the description as it is ordered, the quantity, the material or the maker,
# the mass. It writes each part number and description into its components, reads them back, and writes the lists to
# results/bom/<machine>.csv and results/bom.json. The last machine's design stays open, its components numbered.
# Written from PROMPT.txt beside it; checked by checks/bill_of_materials_check.py.
#
#   INPUTS        the dialog: the machine (all by default)
#   DERIVED       lib/bom_detail.py (the lines, the designations, the makers' masses), lib/materials_fits.py
#   HARD RULES    nothing of the machines' scripts copied; built through RangeSweep.machine(); nothing saved
#   CONVENTIONS   a new design document for each machine; kg and mm, the API's cm inside
#   ACCEPTANCE    the message; the check: the lines against Fusion's census and masses, the part numbers read back
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import csv
import importlib
import importlib.util
import io
import json
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
importlib.reload(BD)
import materials_fits as MF      # noqa: E402

VERSION = 'BillOfMaterials 1.0 (2026-11-04)'
MACHINES = ('wrist', 'flap', 'axle', 'robot', 'changer', 'clamp', 'vacuum', 'bag', 'unit')
_last = {}


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_bom64_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _mark(doc):
    try:
        doc.attributes.add('mdfc', 'run', 'BillOfMaterials')
    except Exception:
        pass


def census(des):
    """Fusion's census: every occurrence that has a body, (name, mass kg, materials, volume cm3), and the occurrences
    by name."""
    rows, occ = [], {}
    for o in des.rootComponent.allOccurrences:
        bodies = o.bRepBodies                       # the occurrence's own bodies, not its children's
        if not bodies.count:
            continue
        mats = ', '.join(sorted({b.material.name for b in bodies if b.material}))
        rows.append((o.component.name, sum(b.physicalProperties.mass for b in bodies), mats, sum(b.volume for b in bodies)))
        occ.setdefault(o.component.name, []).append(o)
    return rows, occ


def material(key, line):
    """A made line's material, Chapter 51's; a bought line's maker or standard, from its description."""
    if line['kind'] == 'bought':
        d = line['description']
        return d.split()[0] + (' ' + d.split()[1] if d.split()[0] in ('ISO', 'DIN') else '')
    kinds = list(line['names']) or [line['description']]
    alloy, cond, src = MF.material(key, kinds[0], (line['mats'] or ['Steel'])[0])
    return alloy + (', ' + cond if cond else '')


def number_components(b, occ):
    """Each component carries its line's part number and description; returns the lines whose components read back
    otherwise."""
    wrong = []
    for g in b['lines']:
        if not g['number']:
            continue
        for setn, names in g['names'].items():
            for name, piece in names:
                for o in occ.get(name, []):
                    c = o.component
                    c.partNumber = g['number']
                    c.description = g['description'][:250]
    for g in b['lines']:
        if not g['number'] or not g['names']:             # the weldment: its pieces carry their own numbers
            continue
        got = {o.component.partNumber for names in g['names'].values() for name, piece in names for o in occ.get(name, [])}
        if got != {g['number']}:
            wrong.append((g['number'], sorted(got)))
    return wrong


def write_csv(key, b):
    os.makedirs(os.path.join(PROJECT, 'results', 'bom'), exist_ok=True)
    path = os.path.join(PROJECT, 'results', 'bom', '%s.csv' % key)
    with io.open(path, 'w', encoding='utf-8', newline='') as f:
        w = csv.writer(f)
        w.writerow(['item', 'level', 'part number', 'description', 'qty', 'material or maker', 'kg each', 'kg',
                    'mass from', 'kg as drawn', 'superseded'])
        for g in b['lines']:
            lvl = 2 if g['kind'].startswith('in ') else 1
            src = ('maker: ' + g['maker'][1]) if g.get('maker') else 'Fusion'
            w.writerow([g.get('item', ''), lvl, g['number'] or '', g['description'], g['qty'], g.get('material', ''),
                        '%.4f' % (g['book'] / g['qty']), '%.4f' % g['book'], src, '%.4f' % g['drawn'],
                        ' '.join(g.get('was', []))])
    return path


def one(key, RS, close=True):
    app = adsk.core.Application.get()
    msg, _ = RS.machine(key)
    doc = app.activeDocument
    _mark(doc)
    des = adsk.fusion.Design.cast(app.activeProduct)
    rows, occ = census(des)
    b = BD.finish(BD.bom(key, rows))
    for g in b['lines']:
        g['material'] = material(key, g)
    wrong = number_components(b, occ)
    path = write_csv(key, b)
    fusion_mass = des.rootComponent.physicalProperties.mass     # Fusion's own mass of the whole design
    out = dict(key=key, occurrences=len(rows), fusion_mass=fusion_mass, census_mass=sum(r[1] for r in rows), loads=b['loads'],
               load_mass=sum(m for n, m in b['loads']), made_kinds=b['made_kinds'], made_pieces=b['made_pieces'],
               bought_kinds=b['bought_kinds'], bought_pieces=b['bought_pieces'], with_maker=b['with_maker'],
               drawn=b['drawn'], book=b['book'], bought_drawn=b['bought_drawn'], bought_book=b['bought_book'],
               wrong_numbers=wrong, csv=os.path.basename(path),
               lines=[dict(item=g.get('item'), number=g['number'], description=g['description'], kind=g['kind'],
                           qty=g['qty'], material=g['material'], drawn=g['drawn'], book=g['book'],
                           maker=list(g['maker']) if g.get('maker') else None, note=g.get('note'),
                           was=g.get('was', []), per=g.get('per'), parts=g.get('parts'),
                           names=sorted(n for v in g['names'].values() for n, p in v),
                           with_=[w['name'] for w in g.get('with', [])]) for g in b['lines']])
    if close:
        doc.close(False)
    return out


def build(which='all'):
    if which != 'all' and which not in MACHINES:
        raise ValueError('No machine called %s: the bills of materials are of %s.' % (which, ', '.join(MACHINES)))
    RS = _load('RangeSweep')
    keys = [k for k in MACHINES if which in ('all', k)]
    out = [one(k, RS, close=(k != keys[-1])) for k in keys]     # the last stays open, numbered
    json.dump(dict(version=VERSION, machines=out, superseded=BD.superseded()),
              io.open(os.path.join(PROJECT, 'results', 'bom.json'), 'w', encoding='utf-8'), indent=1, default=str)
    _last.clear()
    _last.update(machines=out)
    lines = ['%-8s made %2d kinds, %3d pieces; bought %2d kinds, %3d pieces (%d with the maker\'s mass); %.2f kg as drawn, '
             '%.2f kg with the makers\' masses' % (m['key'], m['made_kinds'], m['made_pieces'], m['bought_kinds'],
                                                    m['bought_pieces'], m['with_maker'], m['drawn'], m['book']) for m in out]
    return 'Bills of materials: %d machines, results/bom/.\n%s\n\n[%s]' % (len(out), '\n'.join(lines), VERSION)


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
            ui.messageBox('BillOfMaterials failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('machine', 'Machine', adsk.core.DropDownStyles.TextListDropDownStyle)
        dd.listItems.add('all', True)
        for k in MACHINES:
            dd.listItems.add(k, False)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcBillOfMaterials'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Bills of materials', 'Chapter 64: the nine bills of materials')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('BillOfMaterials failed:\n%s' % traceback.format_exc())
