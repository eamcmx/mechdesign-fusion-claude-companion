# -*- coding: utf-8 -*-
# =============================================================================
# MaterialsFits.py  -  Chapter 51, materials, fits and surface finish, written from its PROMPT.txt alone (a prompt
# test). One of the nine assemblies is built by its own script; every made part is given the material
# materials_fits.py decides for it, as the entry of Fusion's library that stands for it (FUSION); every seat and fit
# found on the model by fusion_joints.records() is given its tolerance class and roughness by materials_fits.decide(),
# its face coloured by its kind of joint and tagged with an attribute 'mdfc' 'fit'.
#
#   INPUTS        the dialog: the assembly, one of detail_inventory.MACHINES (the bag gripper by default)
#   DERIVED       fusion_joints: build(), parts(), records(), links(); materials_fits: material(), FUSION, decide()
#   HARD RULES    an assembly not one of the nine refused; no class, roughness or material typed here; a material
#                 the library lacks reported and the model's kept; the bought parts left as their scripts made them
#   CONVENTIONS   the assembly's own script makes the document; nothing moved, no joint added; mm in the records
#   ACCEPTANCE    the message: the assembly, the materials given and lacking, the made parts' mass both ways, the
#                 faces coloured by kind, the version tag
# =============================================================================
import importlib
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
import companion_paths as CP   # noqa: E402,F401  every lib/ folder on sys.path
import detail_inventory as D   # noqa: E402
import materials_fits as M     # noqa: E402
import fusion_joints as J      # noqa: E402
importlib.reload(D)
importlib.reload(M)
importlib.reload(J)

VERSION = 'MaterialsFits (prompt test) 1.0 (2026-11-11)'

# the colour of each kind of joint decide() gives, from Fusion's appearance library
RED, BLUE, GREEN, BLACK = ['Paint - Enamel Glossy (%s)' % c for c in ('Red', 'Blue', 'Green', 'Black')]
COLOURS = {'bearing seat': RED, 'plain bearing seat': RED, 'sliding': BLUE, 'located': GREEN, 'keyed': GREEN,
           'pressed': GREEN, 'dowel hole': BLACK, 'key seat': BLACK}
ORDER = ('bearing seat', 'plain bearing seat', 'sliding', 'pressed', 'keyed', 'located', 'dowel hole', 'key seat')
SHOWN = ((RED, 'red (bearing seats)'), (BLUE, 'blue (sliding fits)'), (GREEN, 'green (located, keyed and pressed fits)'),
         (BLACK, 'black (dowel holes and key seats)'))
JOINT_WORDS = (('bearing', 'bearings'), ('key seat', 'keys'), ('dowel hole', 'dowel holes'),
               ('sliding', 'sliding fits'), ('located', 'located fits'), ('keyed', 'keyed fits'),
               ('pressed', 'pressed fits'), ('spline', "splines (the standard's fit)"),
               ('thread', "threads (the thread's fit)"))
NUMBER = {9: 'nine'}

_last = {}


# ---------- the assembly ----------
def machine(name):
    """The key of one of detail_inventory's MACHINES, given by its key or its title; anything else refused."""
    s = str(name).strip()
    low = s.lower()
    for k, title, _f, _ch in D.MACHINES:
        t = title.lower()
        if low in (k, t, t[4:] if t.startswith('the ') else t):
            return k
    n = len(D.MACHINES)
    raise ValueError("'%s' is not one of the %s assemblies: %s."
                     % (s, NUMBER.get(n, str(n)), ', '.join('%s (%s)' % (t, k) for k, t, _f, _ch in D.MACHINES)))


def title_of(key):
    return [t for k, t, _f, _ch in D.MACHINES if k == key][0]


# ---------- Fusion's libraries ----------
def _library(word):
    """Fusion's own material or appearance library ('Material' or 'Appearance')."""
    libs = adsk.core.Application.get().materialLibraries
    lib = libs.itemByName('Fusion %s Library' % word)
    if lib is None:
        for i in range(libs.count):
            li = libs.item(i)
            if li.name.startswith('Fusion') and li.name.endswith('%s Library' % word):
                return li
    return lib


def _appearance(des, name):
    """The appearance of that name, copied into the design from Fusion's library when the design lacks it."""
    ap = des.appearances.itemByName(name)
    if ap is None:
        lib = _library('Appearance')
        src = lib.appearances.itemByName(name) if lib is not None else None
        if src is None:
            return None
        ap = des.appearances.addByCopy(src, name)
    return ap


def weigh(made):
    """The made parts' mass in kg, body by body, as Fusion gives it."""
    return sum(b.physicalProperties.mass for p in made for b in p['bodies'])


# ---------- the materials ----------
def give_materials(key, made):
    """Every made part's bodies given FUSION's entry for the alloy materials_fits.material() decides; a material the
    library lacks reported and the model's kept. Returns (parts given, {entry, alloy: parts}, {lacking: kinds})."""
    lib = _library('Material')
    found, given, used, missing = {}, 0, {}, {}
    for p in made:
        alloy, _cond, _src = M.material(key, p['kind'], p['material'])
        entry = M.FUSION.get(alloy)
        if entry not in found:
            found[entry] = lib.materials.itemByName(entry) if (lib is not None and entry) else None
        mat = found[entry]
        if mat is None:
            what = alloy if entry is None else '%s (%s not in the library)' % (alloy, entry)
            missing.setdefault(what, set()).add(p['kind'])
            continue
        for b in p['bodies']:
            b.material = mat
        given += 1
        used[(entry, alloy)] = used.get((entry, alloy), 0) + 1
    return given, used, missing


# ---------- the fits ----------
def finish(cls, rec):
    """A face's class and roughness as the attribute carries it: 'k5 Ra 0.8', 'N7 Rz 16.0'."""
    rz, ra = rec.get('rz'), rec.get('ra')
    if rz is not None:
        return '%s Rz %s' % (cls, float(rz))
    if ra is not None:
        return '%s Ra %s' % (cls, float(ra))
    return cls


def decided_faces(seats, fits):
    """{face token: [face, {class and roughness}, {kind of joint}]} for every face of a seat or fit decide() gave a
    class: a seat's made face (or a key's faces) its class; a fit's shaft face the shaft's class, its bore the hole's."""
    rows = {}

    def add(f, value, kind):
        r = rows.setdefault(f.entityToken, [f, set(), set()])
        r[1].add(value)
        r[2].add(kind)
    for x in seats:
        if not x.get('cls'):
            continue
        faces = [x['_face']] if x.get('_face') is not None else list(x.get('_faces') or [])
        for f in faces:
            add(f, finish(x['cls'], x), x.get('joint'))
    for x in fits:
        if not x.get('cls'):
            continue
        hole_cls, shaft_cls = x['cls'].split('/')
        f_shaft, f_bore = x['_faces']
        add(f_shaft, finish(shaft_cls, dict(ra=x.get('ra'), rz=x.get('rz'))), x.get('joint'))
        add(f_bore, finish(hole_cls, dict(ra=x.get('ra_hole'), rz=None)), x.get('joint'))
    return rows


def mark_faces(des, rows):
    """Each decided face tagged 'mdfc' 'fit' with its classes (two zones joined by ' + ') and coloured by its kind of
    joint, a bearing seat's colour first. A face shared by several occurrences of one component carries every class
    any of them gives it. Returns ({appearance: faces}, faces with two zones, appearances the library lacks)."""
    native = {}
    for t, (f, vals, _k) in rows.items():
        n = f.nativeObject
        native.setdefault(n.entityToken if n is not None else t, set()).update(vals)
    aps, counts, zones, lacking = {}, {}, 0, set()
    for t, (f, vals, kinds) in rows.items():
        n = f.nativeObject
        allv = native[n.entityToken if n is not None else t]
        value = ' + '.join(sorted(allv))
        zones += len(allv) > 1
        a = f.attributes.itemByName('mdfc', 'fit')
        if a is not None:
            a.value = value
        else:
            f.attributes.add('mdfc', 'fit', value)
        kind = next((k for k in ORDER if k in kinds), None)
        if kind is None:
            continue
        name = COLOURS[kind]
        if name not in aps:
            aps[name] = _appearance(des, name)
        if aps[name] is None:
            lacking.add(name)
            continue
        f.appearance = aps[name]
        counts[name] = counts.get(name, 0) + 1
    return counts, zones, sorted(lacking)


# ---------- the run ----------
def build(key='bag'):
    key = machine(key)
    J.build(key)                                       # the assembly's own script: its own new design
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    ps = J.parts(root, key)
    made = [p for p in ps if p['status'] == 'made']
    # the materials, the made parts weighed before and after
    before = weigh(made)
    given, used, missing = give_materials(key, made)
    after = weigh(made)
    # the joints found on the model and decided by rule, the model's own materials for the defaults
    seats, fits = J.records(ps, app.measureManager)
    rec = dict(key=key, components=[dict(name=p['name'], occ=p['occ'], kind=p['kind'], status=p['status'],
                                         designation=p['designation'], group=p['group'], material=p['material'],
                                         top=p['top']) for p in ps],
               seats=seats, fits=fits, tops={o.component.name: o.isGrounded for o in root.occurrences},
               links=J.links(root), gaps=[])
    dec = M.decide(key, rec)
    rows = decided_faces(seats, fits)
    counts, zones, lacking = mark_faces(des, rows)
    _last.clear()
    _last.update(key=key, parts=ps, made=made, before=before, after=after, given=given, used=used,
                 missing=sorted(missing), seats=seats, fits=fits, record=rec, decided=dec, faces=len(rows),
                 colours=counts, zones=zones, lacking_appearances=lacking)
    # the message
    by_kind = {}
    for j in dec['joints']:
        by_kind[j['kind']] = by_kind.get(j['kind'], 0) + 1
    joints = ', '.join('%s %d' % (w, by_kind[k]) for k, w in JOINT_WORDS if by_kind.get(k))
    script = 'StandardParts' if key == 'bag' else D.SCRIPTS[key]
    lines = ['%s, built by %s.py.' % (title_of(key), script),
             'Materials: %d of %d made parts given the library\'s material; the library lacks %s.'
             % (given, len(made), '; '.join('%s (%s, the model\'s material kept)' % (m, ', '.join(sorted(k)))
                                            for m, k in sorted(missing.items())) or 'none'),
             '   ' + '; '.join('%s for %s: %d' % (e, a, n) for (e, a), n in sorted(used.items())),
             'The made parts weigh %.3f kg with the library\'s materials and %.3f kg with the model\'s generic ones.'
             % (after, before),
             'Joints decided: %s; %d seats the model leaves open by up to 0.2 mm, decided as touching (the model has '
             'to be closed).' % (joints or 'none', len(dec['gapped'])),
             'Faces coloured by kind: %s; %d faces carry \'mdfc\' \'fit\', %d of them two classes, one for each zone.'
             % (', '.join('%d %s' % (counts.get(ap, 0), w) for ap, w in SHOWN), len(rows), zones)]
    if lacking:
        lines.append('Fusion\'s appearance library lacks %s: those faces carry their attribute, uncoloured.'
                     % ', '.join(lacking))
    return '\n'.join(lines) + '\n\n[%s]' % VERSION


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            chosen = ins.itemById('assembly').selectedItem.name
            ui.messageBox(build(chosen), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('assembly', 'The assembly', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k, title, _f, _ch in D.MACHINES:
            dd.listItems.add(title, k == 'bag', '')
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'MaterialsFitsCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Materials and fits',
                                                         'Chapter 51: materials, fits and surface finish')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('MaterialsFits could not start:\n' + traceback.format_exc())
