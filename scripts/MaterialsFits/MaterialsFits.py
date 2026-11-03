# =============================================================================
# MaterialsFits.py  -  Chapter 51 of "Mechanical Design in Autodesk Fusion with Claude": an assembly with every made
# part given the material Chapter 51 decided and every fitting surface given its tolerance class and roughness. The
# assembly is built by its own script (the bag gripper by StandardParts.py, as Chapter 50 detailed it); its joints are
# found on the model as the census finds them (lib/fusion_joints.py) and decided by lib/materials_fits.py's rules.
# Written from PROMPT.txt beside it; checked by checks/materials_fits_check.py.
#
#   INPUTS        the dialog: the assembly (one of the nine)
#   DERIVED       materials_fits: material(), decide(), FUSION; fusion_joints: seats(), fits(); detail_inventory
#   HARD RULES    a material Fusion's library lacks is reported, not guessed; an assembly not one of the nine refused
#   CONVENTIONS   the assembly's own; each decided face coloured by its kind of joint and given an attribute
#                 ('mdfc', 'fit') with its class and Ra
#   ACCEPTANCE    the message; the check: the materials, the classes on the faces, the masses
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import os
import sys
import traceback

import adsk.core
import adsk.fusion

SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(os.path.dirname(SCRIPTS), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import detail_inventory as D   # noqa: E402
import materials_fits as M     # noqa: E402
import fusion_joints as J      # noqa: E402
import standard_parts as S     # noqa: E402
importlib.reload(D)
importlib.reload(M)
importlib.reload(J)
importlib.reload(S)

VERSION = 'MaterialsFits 1.0 (2026-11-03)'
COLOURS = {'bearing seat': 'Paint - Enamel Glossy (Red)', 'plain bearing seat': 'Paint - Enamel Glossy (Red)',
           'sliding': 'Paint - Enamel Glossy (Blue)',
           'located': 'Paint - Enamel Glossy (Green)', 'keyed': 'Paint - Enamel Glossy (Green)',
           'pressed': 'Paint - Enamel Glossy (Green)', 'dowel hole': 'Paint - Enamel Glossy (Black)',
           'key seat': 'Paint - Enamel Glossy (Black)'}


def library_material(name):
    """The material of that name in Fusion's Material Library, or None."""
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    try:
        return lib.materials.itemByName(name)
    except RuntimeError:
        return None


def appearance(design, name):
    """The design's copy of an appearance from Fusion's Appearance Library, made on first use (as mdfc_parts.colour)."""
    ap = design.appearances.itemByName(name)
    if not ap:
        lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
        ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
    return ap


def finish(cls, x):
    """The attribute's value: the class and the roughness, Ra or (SKF's plain bearings) Rz, in um."""
    return '%s %s %.1f' % (cls, 'Rz' if x.get('rz') else 'Ra', x.get('rz') or x['ra'])


def mass(ps):
    return sum(b.physicalProperties.mass for p in ps for b in p['bodies'])


_last = {}


def build(key='bag'):
    if key not in D.SCRIPTS:
        raise ValueError('No assembly called %s: the nine are %s.' % (key, ', '.join(D.SCRIPTS)))
    J.build(key, SCRIPTS)                                   # the bag gripper as Chapter 50 detailed it
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    mm_ = app.measureManager
    ps = J.parts(root, key)
    made = [p for p in ps if p['status'] == 'made']
    before = mass(made)
    # the materials
    missing, given = set(), 0
    for p in made:
        alloy = M.material(key, p['kind'], p['material'])[0]
        lib_name = M.FUSION.get(alloy)
        m = library_material(lib_name) if lib_name else None
        if m is None:
            missing.add(alloy)
            continue
        for b in p['bodies']:
            b.material = m
        given += 1
    after = mass(made)
    # the joints, found on the model as the census finds them and decided by the rules
    seats, fits = J.records(ps, mm_)
    rec = dict(key=key, components=[dict(name=p['name'], occ=p['occ'], kind=p['kind'], status=p['status'],
                                         designation=p['designation'], group=p['group'], material=p['material'],
                                         top=p['top']) for p in ps],
               seats=seats, fits=fits, tops={o.component.name: o.isGrounded for o in root.occurrences},
               links=J.links(root), gaps=[])
    dec = M.decide(key, rec)
    on_face = {}                                         # one face, every class the rules give it
    for x in seats:
        if 'cls' in x:
            for f in ([x['_face']] if x['_face'] is not None else x.get('_faces', [])):
                on_face.setdefault(f.entityToken, [f, set(), x['joint']])[1].add(finish(x['cls'], x))
    for x in fits:
        if 'cls' in x:
            h, sh = x['cls'].split('/')
            for f, cls, ra, rz in zip(x['_faces'], (sh, h), (x['ra'], x['ra_hole']), (x.get('rz'), None)):
                on_face.setdefault(f.entityToken, [f, set(), x['joint']])[1].add(finish(cls, dict(ra=ra, rz=rz)))
    coloured = {}
    for f, vals, joint in on_face.values():
        f.appearance = appearance(des, COLOURS[joint])
        f.attributes.add('mdfc', 'fit', ' + '.join(sorted(vals)))      # two values: two zones of one face
        coloured[joint] = coloured.get(joint, 0) + 1
    _last.update(key=key, parts=ps, made=made, before=before, after=after, missing=missing, given=given,
                 decided=dec, seats=seats, fits=fits, coloured=coloured)
    return ('%s: %d made parts given their materials (%s not in Fusion\'s library); the made parts weigh %.2f kg, '
            '%.2f kg with Fusion\'s generic materials; faces given a class and a roughness: %s.\n\n[%s]'
            % (key, given, ', '.join(sorted(missing)) or 'none', after, before,
               ', '.join('%d %s' % (v, k) for k, v in sorted(coloured.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            title = ins.itemById('assembly').selectedItem.name
            key = [m[0] for m in D.MACHINES if m[1] == title][0]
            ui.messageBox(build(key), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('assembly', 'The assembly', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k, title, fname, ch in D.MACHINES:
            dd.listItems.add(title, k == 'bag')
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
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Materials and fits', 'Chapter 51: materials, fits, roughness')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('MaterialsFits could not start:\n' + traceback.format_exc())
