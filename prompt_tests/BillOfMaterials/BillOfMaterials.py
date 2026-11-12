# =============================================================================
# BillOfMaterials.py  -  Chapter 64 of "Mechanical Design in Autodesk Fusion with Claude": the bills of materials.
# It builds each of the nine detailed assemblies through RangeSweep.py's machine() (the bag gripper with its stops),
# takes Fusion's census of every occurrence that has a body (its name, Fusion's mass, its materials and volume), and
# hands it to lib/bom_detail.py, which gives each component its line: the part number of Chapter 49's scheme
# (lib/detail_inventory.py) and the numbers the detail added, the description a shop or a buyer orders by, the
# quantity counted from the model, and the mass (Fusion's for a made part, the maker's for a bought part where a saved
# page gives it). Each made kind's material comes from lib/materials_fits.py. The part number and the description are
# written into every component, so that Fusion's own parts lists carry them, and read back.
# Written from design_office/BillOfMaterials/PROMPT.txt.
#
#   INPUTS        the dialog: the machine (all by default)
#   DERIVED       RangeSweep.machine(); bom_detail: the lines, descriptions, weldment, superseded numbers, makers'
#                 masses; materials_fits: each made kind's material
#   HARD RULES    the machines built through RangeSweep.py, loaded; the census from Fusion; a bought part once per
#                 set; the loads left out and counted; the weldment one line; the lower number stands; the numbers
#                 written into the components and read back; the lists to results/bom/<machine>.csv and bom.json
#   CONVENTIONS   each machine's own new design document; masses in kg, lengths in mm, the API's cm inside; the
#                 bill adds no joint and moves nothing
#   ACCEPTANCE    the message; the check: the lines and loads against Fusion's mass, the numbers, the sets, the
#                 weldment, the loads, the files, the refusal
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import csv
import importlib
import importlib.util
import io
import json
import os
import re
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
import companion_paths as CP                            # noqa: E402  every lib/ folder on sys.path
import detail_inventory as DI                           # noqa: E402
import materials_fits as MF                             # noqa: E402
import bom_detail as BD                                 # noqa: E402

VERSION = 'BillOfMaterials 1.0 (prompt test, 2026-11-12)'
MACHINES = ('wrist', 'flap', 'axle', 'robot', 'changer', 'clamp', 'vacuum', 'bag', 'unit')
CHOICES = ('all',) + MACHINES
RESULTS = os.path.join(CP.ROOT, 'design_office', 'results')
_last = {}


class Refused(ValueError):
    """A machine that is not one of the nine."""


def _reload():
    """Fusion keeps the modules it has imported: the libraries read again, so that an edit to them counts."""
    for m in (DI, MF, BD):
        importlib.reload(m)


def _range_sweep():
    """RangeSweep.py loaded as it stands (not copied): its machine() builds each machine with its own script."""
    spec = importlib.util.spec_from_file_location('mdfc_bom64_RangeSweep', CP.script('RangeSweep'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ---------- Fusion's census ----------
def _props(x, how):
    """Fusion's physical properties of a body or a component, all of them at one accuracy."""
    if how['acc'] is None:
        try:
            how['acc'] = adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy
            return x.getPhysicalProperties(how['acc'])
        except Exception:
            how['acc'] = False
    if how['acc'] is False:
        return x.physicalProperties
    return x.getPhysicalProperties(how['acc'])


def _segments(o):
    return [re.sub(r':\d+$', '', s) for s in o.fullPathName.split('+')]


def census(des, how):
    """Every occurrence that has a body: its component's name, Fusion's mass (kg), its materials, its volume (cm3), and
    whether it sits inside a load (the station's groups)."""
    out = []
    for o in des.rootComponent.allOccurrences:
        if not o.component.bRepBodies.count:
            continue
        bodies = list(o.bRepBodies)
        mats = []
        for b in bodies:
            nm = b.material.name if b.material else ''
            if nm and nm not in mats:
                mats.append(nm)
        out.append(dict(name=o.component.name, mass=sum(_props(b, how).mass for b in bodies),
                        vol=sum(b.volume for b in bodies), mats=mats,
                        under_load=any(s.startswith(BD.LOADS) for s in _segments(o)[:-1])))
    return out


# ---------- the lines ----------
def _attach(key, lines, pl):
    """A body that comes with a bought part is no line of its own: where bom_detail left one apart (its machine has
    more than one part of that name), it goes under the one bought line it can belong to, by the part's name and then
    by the words its name carries after the kind (a copy's number, a side). Those that still match no single line
    are kept, one line for each kind of body, so that no mass is lost."""
    sup = BD.superseded()
    bought = [g for g in lines if g['kind'] == 'bought']
    out, left, attached = [], [], 0
    for g in lines:
        if not (g['kind'].startswith('with ') and g['number'] is None):
            out.append(g)
            continue
        grp = g['kind'][5:]
        nm = list(g['names'])[0]
        if re.match(r'^[A-Z]+\d*-\d+$', grp):
            cand = [L for L in bought if grp in (L['number'], sup.get(L['number'])) or L['number'] == sup.get(grp)]
        else:
            cand = [L for L in bought if any(s.startswith(grp) for s in L['names'])]
            if len(cand) > 1:
                base = BD._base(nm)
                kd = DI.kind_of(base, pl) or ''
                tail = base[len(kd):].split()
                if tail:
                    cand = [L for L in cand if any(BD._base(n).split()[-len(tail):] == tail
                                                   for v in L['names'].values() for n, _p in v)]
        if cand and len({sup.get(L['number'], L['number']) for L in cand}) == 1:   # one designation, maybe two numbers
            L = cand[0]
            L.setdefault('with', []).append(dict(name=nm, mass=g['mass'], parent=L['number'], description=g['description']))
            attached += 1
        else:
            left.append(g)
    kept = {}
    for g in left:
        k = (g['kind'], g['description'])
        if k in kept:
            m = kept[k]
            m['qty'] += g['qty']
            m['mass'] += g['mass']
            for n, v in g['names'].items():
                m['names'].setdefault(n, []).extend(v)
        else:
            kept[k] = g
            out.append(g)
    return out, attached, len(left)


def _supersede_made(lines):
    """The lower number stands for a made part too: a made number the designations supersede takes the one that
    stands, merged into its line if the machine has one."""
    sup = BD.superseded()
    out, by = [], {}
    for g in lines:
        n = g['number']
        if g['kind'] != 'bought' and n in sup:
            g['was'] = sorted(set(g.get('was', []) + [n]))
            g['number'] = n = sup[n]
        if n is not None and g['kind'] != 'bought' and n in by and by[n]['kind'] == g['kind']:
            m = by[n]
            m['qty'] += g['qty']
            m['mass'] += g['mass']
            m['mats'] = sorted(set(m['mats']) | set(g['mats']))
            m['was'] = sorted(set(m.get('was', []) + g.get('was', [])))
            for k, v in g['names'].items():
                m['names'].setdefault(k, []).extend(v)
            continue
        if n is not None and g['kind'] != 'bought':
            by[n] = g
        out.append(g)
    return out


def _totals(b):
    """The counts and masses of the first level (as bom_detail.finish counts them)."""
    tl = BD.top(b['lines'])
    made = [g for g in tl if g['kind'] == 'made']
    bought = [g for g in tl if g['kind'] == 'bought']
    b.update(made_kinds=len(made), made_pieces=sum(g['qty'] for g in made), bought_kinds=len(bought),
             bought_pieces=sum(g['qty'] for g in bought), with_maker=sum(1 for g in bought if g.get('maker')),
             drawn=sum(g['drawn'] for g in tl), book=sum(g['book'] for g in tl),
             bought_drawn=sum(g['drawn'] for g in bought), bought_book=sum(g['book'] for g in bought))


_INV = {}
for _lab, _lib in MF.FUSION.items():
    _INV.setdefault(_lib, []).append(_lab)


def _material(key, kind, model):
    """Chapter 51's material of one made kind, the model's material (Fusion's library name turned back into the
    chapter's label where that is one alloy) for the default."""
    lab = _INV.get(model)
    try:
        a, c, src = MF.material(key, kind, lab[0] if lab and len(lab) == 1 else model)
    except Exception:
        a, c, src = model or 'not given', None, 'the model'
    return '%s%s (%s)' % (a, ', ' + c if c else '', src)


def _materials(key, b, mats_of):
    """Each made line's material, each bought line the maker's; a weldment's from its pieces; the welds the model's."""
    for g in b['lines']:
        if g['kind'] == 'bought' or g['kind'].startswith('with'):
            g['material'] = ''
            continue
        if g['kind'].startswith('in ') and g['number'] is None:
            ms = sorted({m for v in g['names'].values() for n, _p in v for m in mats_of.get(n, [])})
            g['material'] = 'weld (the model: %s)' % ', '.join(ms) if ms else 'weld'
            continue
        out = []
        for kind, v in g['names'].items():
            model = next((m for n, _p in v for m in mats_of.get(n, [])), '')
            s = _material(key, kind, model)
            if s not in out:
                out.append(s)
        g['material'] = '; '.join(out)
    for g in b['lines']:                                     # a weldment: the material of its pieces
        if g.get('parts') is not None:
            ps = [p for p in b['lines'] if p['kind'] == 'in ' + g['number'] and p['number']]
            g['material'] = '; '.join(sorted({p['material'] for p in ps if p['material']}))


def _mass_from(b):
    """Each line says whose its mass is; a maker's page that is not on disk is no saved page: Fusion's mass stands."""
    missing = []
    for g in b['lines']:
        mk = g.get('maker')
        if mk and not os.path.exists(os.path.join(BD.REFS, mk[1])):
            missing.append((g['number'], mk[1]))
            g['maker'] = None
            g['book'] = g['drawn']
            g['note'] = 'the page %s is not on disk' % mk[1]
    if missing:
        _totals(b)
    for g in b['lines']:
        if g['kind'] == 'bought' and g.get('maker'):
            kg, page, what = g['maker'][:3]
            g['mass_from'] = "the maker's, %.4g kg each: %s (%s)" % (kg, page, what)
        elif g['kind'] == 'bought':
            g['mass_from'] = "Fusion's: %s" % (g.get('note') or 'no saved page gives it')
        else:
            g['mass_from'] = "Fusion's"
    return missing


def bill(key, des, how):
    """One machine's bill from Fusion's census."""
    pl = DI.parts([m for m in DI.MACHINES if m[0] == key][0][2])[0]
    nmap = DI.number_map(key)
    cs = census(des, how)
    occs, inside, unknown, mats_of = [], [], {}, {}
    for e in cs:
        mats_of.setdefault(e['name'], [])
        for m in e['mats']:
            if m not in mats_of[e['name']]:
                mats_of[e['name']].append(m)
        if e['under_load']:                                  # a part of the station's groups: a load
            inside.append((e['name'], e['mass']))
            continue
        try:
            BD.line_of(key, e['name'], pl, nmap)
        except KeyError:                                     # no number for this name: kept, so that its mass counts
            nm = BD._base(e['name'])
            u = unknown.setdefault(nm, dict(number=None, description='no number found for %s' % nm, kind='unknown',
                                            names={}, mass=0.0, mats=[], qty=0))
            u['names'].setdefault(nm, []).append((e['name'], ''))
            u['mass'] += e['mass']
            u['qty'] += 1
            u['mats'] = sorted(set(u['mats']) | set(e['mats']))
            continue
        occs.append((e['name'], e['mass'], ', '.join(e['mats']), e['vol']))
    b = BD.bom(key, occs)
    b['loads'] = list(b['loads']) + inside
    lines, attached, apart = _attach(key, b['lines'], pl)
    b['lines'] = _supersede_made(lines + list(unknown.values()))
    BD.finish(b)
    missing = _mass_from(b)
    _materials(key, b, mats_of)
    root = des.rootComponent
    fusion = _props(root, how).mass
    n_acc = sum(len(v) for g in b['lines'] for v in g['names'].values()) + sum(len(g.get('with', [])) for g in b['lines'])
    title = [m for m in DI.MACHINES if m[0] == key][0][1]
    b.update(title=title, csv=key + '.csv', occurrences=len(cs), accounted=n_acc + len(b['loads']),
             load_mass=sum(m for _n, m in b['loads']), fusion_mass=fusion, census_mass=sum(e['mass'] for e in cs),
             root_bodies=root.bRepBodies.count, unknown=sorted(unknown), attached=attached, apart=apart,
             pages_missing=missing, superseded_merged=sorted({w for g in b['lines'] for w in g.get('was', [])}),
             accuracy='low' if how['acc'] is False else 'medium', version=VERSION)
    return b


# ---------- the numbers in the components ----------
def _targets(lines):
    """What each component carries: its line's number and description; a body that comes with a bought part, and a
    weld, the number of the line it is under."""
    t = {}
    for g in lines:
        if g['kind'].startswith('in ') and g['number'] is None:
            num, desc = g['kind'][3:], '%s, in %s' % (g['description'], g['kind'][3:])
        else:
            num, desc = g['number'], g['description']
        for v in g['names'].values():
            for n, _p in v:
                t[n] = (num, desc)
        for w in g.get('with', []):
            t[w['name']] = (g['number'], 'comes with %s, %s' % (g['number'], g['description']))
    return t


def _cid(c):
    try:
        return c.id
    except Exception:
        return c.name


def number(key, des, b):
    """Write each component's part number and description, then read every one back from Fusion."""
    root = des.rootComponent
    t = _targets(b['lines'])
    sub = (DI.SUB[key] + '-000', b['title'][4:].capitalize())
    want, wrong = {}, []
    for o in root.allOccurrences:
        c = o.component
        if c.bRepBodies.count and c.name in t:
            want[_cid(c)] = (c, c.name) + t[c.name]
    for c, nm, num, desc in want.values():
        try:
            if num:
                c.partNumber = num
            c.description = desc
        except Exception as e:
            wrong.append((nm, num, 'not written: %s' % e))
    try:
        root.partNumber, root.description = sub
    except Exception as e:
        wrong.append((root.name, sub[0], 'not written: %s' % e))
    adsk.doEvents()
    seen = set()
    for o in root.allOccurrences:                        # read back, a fresh pass over the design
        c = o.component
        if not c.bRepBodies.count or c.name not in t or _cid(c) in seen:
            continue
        seen.add(_cid(c))
        num, desc = t[c.name]
        if (num and c.partNumber != num) or c.description != desc:
            wrong.append((c.name, num, c.partNumber))
    if root.partNumber != sub[0]:
        wrong.append((root.name, sub[0], root.partNumber))
    b.update(wrong_numbers=wrong, components_numbered=len(seen), assembly_number=sub[0])


# ---------- the files ----------
COLS = ('item', 'level', 'part number', 'description', 'kind', 'qty', 'per weldment', 'material', 'mass each kg',
        'mass as drawn kg', 'mass kg', 'mass from', 'supersedes', 'comes with', 'source')


def _n(v):
    return '' if v in ('', None) else ('%d' % v if float(v) == int(v) else '%g' % v)


def _row(g, level):
    q = g['qty'] or 1
    return [g.get('item', ''), level, g['number'] or '', g['description'], g['kind'], _n(g['qty']), _n(g.get('per')),
            g.get('material', ''), '%.4f' % (g['book'] / q), '%.4f' % g['drawn'], '%.4f' % g['book'],
            g.get('mass_from', ''), ' '.join(g.get('was', [])), '; '.join(w['name'] for w in g.get('with', [])),
            g.get('source', '')]


def write_csv(b):
    path = os.path.join(RESULTS, 'bom', b['csv'])
    ins = {}
    for g in b['lines']:
        if g['kind'].startswith('in '):
            ins.setdefault(g['kind'][3:], []).append(g)
    rows = []
    for g in BD.top(b['lines']):
        rows.append(_row(g, 1))
        for p in sorted(ins.get(g['number'], []) if g['number'] else [], key=lambda p: p['number'] or 'zzz'):
            rows.append(_row(p, 2))                      # a weldment's pieces under it
    blank = [''] * len(COLS)
    names = ', '.join(sorted({n.split(' ')[0] for n, _m in b['loads']}))
    tail = [blank,
            ['', '', '', "Total, Fusion's masses", '', '', '', '', '', '%.4f' % b['drawn'], '', '', '', '', ''],
            ['', '', '', "Total, the makers' masses for the bought parts", '', '', '', '', '', '', '%.4f' % b['book'],
             '', '', '', ''],
            ['', '', '', 'Loads left out: %d (%s), %.4f kg' % (len(b['loads']), names or 'none', b['load_mass']),
             '', '', '', '', '', '', '', '', '', '', ''],
            ['', '', '', "Fusion's mass of the machine %.4f kg" % b['fusion_mass'], '', '', '', '', '', '', '', '', '',
             '', '']]
    with io.open(path, 'w', encoding='utf-8', newline='') as f:
        w = csv.writer(f)
        w.writerow(COLS)
        w.writerows(rows + tail)
    return path


def write_json(done, everything):
    """All the machines' lists in results/bom.json; a single machine replaces its own entry and keeps the others."""
    path = os.path.join(RESULTS, 'bom.json')
    data = dict(version=VERSION, machines={})
    if not everything and os.path.exists(path):
        try:
            old = json.load(io.open(path, encoding='utf-8'))
            if isinstance(old, dict) and isinstance(old.get('machines'), dict):
                data['machines'].update(old['machines'])
        except Exception:
            pass
    for m in done:
        data['machines'][m['key']] = m
    data['order'] = [k for k in MACHINES if k in data['machines']]
    with io.open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=1, ensure_ascii=False, default=str)
    return path


# ---------- the run ----------
def _line(m):
    s = ("%s: made %d kinds in %d pieces, bought %d kinds in %d; %.3f kg as drawn, %.3f kg with the makers' masses "
         "(%d of %d bought kinds from a maker's page); %d loads left out (%.2f kg); Fusion's mass %.3f kg; %d components "
         "numbered and read back" % (m['key'], m['made_kinds'], m['made_pieces'], m['bought_kinds'], m['bought_pieces'],
                                     m['drawn'], m['book'], m['with_maker'], m['bought_kinds'], len(m['loads']),
                                     m['load_mass'], m['fusion_mass'], m['components_numbered']))
    notes = []
    acc = m['drawn'] + m['load_mass']
    if abs(acc - m['fusion_mass']) >= 1e-3 * max(1.0, m['fusion_mass']):
        notes.append("the lines and loads %.4f kg against Fusion's %.4f kg" % (acc, m['fusion_mass']))
    if m['accounted'] != m['occurrences']:
        notes.append('%d of %d occurrences accounted for' % (m['accounted'], m['occurrences']))
    if m['root_bodies']:
        notes.append('%d bodies in the root component, outside the census' % m['root_bodies'])
    if m['unknown']:
        notes.append('no number for %s' % ', '.join(m['unknown']))
    if m['apart']:
        notes.append('%d bodies that come with a bought part matched no single line' % m['apart'])
    if m['pages_missing']:
        notes.append("makers' pages not on disk: %s" % ', '.join(p for _n, p in m['pages_missing']))
    if m['wrong_numbers']:
        notes.append('%d components read back wrong: %s' % (len(m['wrong_numbers']), m['wrong_numbers'][:3]))
    if m['superseded_merged']:
        s += '; superseded and merged: %s' % ', '.join(m['superseded_merged'])
    return s + ('. CHECK: ' + '; '.join(notes) + '.' if notes else '.')


def build(machine='all'):
    if machine not in CHOICES:
        raise Refused('No machine called %s: the bills of materials are of %s, or all of them.'
                      % (machine, ', '.join(MACHINES)))
    keys = MACHINES if machine == 'all' else (machine,)
    _reload()
    rs = _range_sweep()
    app = adsk.core.Application.get()
    os.makedirs(os.path.join(RESULTS, 'bom'), exist_ok=True)
    done, last = [], None
    for i, key in enumerate(keys):
        before = list(app.documents)
        rs.machine(key)                                  # the machine as its detail script builds it, its own design
        adsk.doEvents()
        doc = app.activeDocument
        if doc is None or any(doc == d for d in before):
            raise RuntimeError('%s: its script opened no new design; nothing is closed' % key)
        des = adsk.fusion.Design.cast(app.activeProduct)
        how = dict(acc=None)
        b = bill(key, des, how)
        number(key, des, b)
        write_csv(b)
        done.append(b)
        if i < len(keys) - 1:
            doc.close(False)                             # closed without saving: the next machine's design follows
            adsk.doEvents()
        else:
            last = doc
    _last.clear()
    _last.update(machines=done, version=VERSION, machine=machine)
    write_json(done, machine == 'all')
    if last is not None:
        last.activate()
    head = ('Bills of materials of %s, counted from Fusion (%d machine%s):'
            % ('the nine machines' if machine == 'all' else 'the ' + done[0]['title'][4:], len(done),
               '' if len(done) == 1 else 's'))
    tot = ('All: made %d kinds in %d pieces, bought %d kinds in %d; %.3f kg as drawn, %.3f kg with the makers\' masses.'
           % (sum(m['made_kinds'] for m in done), sum(m['made_pieces'] for m in done),
              sum(m['bought_kinds'] for m in done), sum(m['bought_pieces'] for m in done),
              sum(m['drawn'] for m in done), sum(m['book'] for m in done))) if len(done) > 1 else ''
    tail = ('Written to design_office/results/bom/<machine>.csv and design_office/results/bom.json; %s stays open, '
            'its components numbered (%s).' % (done[-1]['title'][0].lower() + done[-1]['title'][1:],
                                               done[-1]['assembly_number']))
    return '\n'.join([head] + [_line(m) for m in done] + ([tot] if tot else []) + [tail, '', '[%s]' % VERSION])


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            key = args.command.commandInputs.itemById('machine').selectedItem.name
            ui.messageBox(build(key))
        except Refused as e:
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
        for k in CHOICES:
            dd.listItems.add(k, k == 'all')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcBillOfMaterials'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Bill of materials',
                                                         'Chapter 64: the bills of materials of the nine machines')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('BillOfMaterials failed:\n%s' % traceback.format_exc())
