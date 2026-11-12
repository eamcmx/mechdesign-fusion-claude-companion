# =============================================================================
# PartNumbers.py  -  builds one of the nine assemblies of design_office/lib/detail_inventory.py with its own script's
# build() and writes every part's number and description into its component, so that a parts list drawn from the
# model carries them.
#
#   INPUTS        the dialog: the assembly, one of the nine (the bag gripper by default)
#   DERIVED       detail_inventory: MACHINES, SCRIPTS and OWN; numbering() and number_map() for each name on the
#                 assembly's parts list; kind_of() for the name a component stands for
#   HARD RULES    build() refuses an assembly that is not one of the nine; the assembly is built by its own script,
#                 never redrawn; a body that comes with a bought part gets a description and no number; a component
#                 on no parts list is left as it is and named in the message
#   CONVENTIONS   the assembly's script opens its own design; Component.partNumber and Component.description are
#                 written once for each component, however many occurrences it has; nothing is moved, no joint added
#   ACCEPTANCE    the message: the assembly and its number, the components numbered, the made kinds with their first
#                 and last numbers, the bought kinds, the bodies that come with a bought part, the names left
#                 unnumbered and the version tag
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
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
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import detail_inventory as D    # noqa: E402
importlib.reload(D)                                     # Fusion keeps old modules

VERSION = 'PartNumbers 1.0 (2026-11-11)'
DEFAULT = 'bag'

_last = {}


# ---------- the assembly ----------
def _key(assembly):
    """The key of MACHINES for an assembly given by its key or its title (with or without 'The')."""
    s = str(assembly).strip().lower()
    for k, title, _fname, _ch in D.MACHINES:
        t = title.lower()
        if s in (k, t, t[4:] if t.startswith('the ') else t):
            return k
    raise ValueError('No assembly called %r. The nine are: %s.'
                     % (assembly, '; '.join('%s (%s)' % (m[1], m[0]) for m in D.MACHINES)))


def _load(key):
    """The assembly's own script, loaded from its folder as a module (its source is not copied here)."""
    name = D.SCRIPTS[key]
    spec = importlib.util.spec_from_file_location('mdfc_' + name, CP.script(name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _ident(c):
    try:
        return c.id
    except Exception:
        return c.entityToken


def _components(root):
    """Each component under the root once, however many occurrences it has: [(component, occurrences)]."""
    seen, order = {}, []
    for o in root.allOccurrences:
        c = o.component
        t = _ident(c)
        if t not in seen:
            seen[t] = [c, 0]
            order.append(t)
        seen[t][1] += 1
    return [tuple(seen[t]) for t in order]


def _names(names):
    """Component names for the message, Fusion's (1), (2) copies counted rather than repeated."""
    count = {}
    for n in names:
        b = re.sub(r' \(\d+\)$', '', n)
        count[b] = count.get(b, 0) + 1
    return ', '.join(b if v == 1 else '%s (x%d)' % (b, v) for b, v in sorted(count.items()))


# ---------- the numbers ----------
def build(key=DEFAULT):
    k = _key(key)
    title = [m[1] for m in D.MACHINES if m[0] == k][0]
    rows = D.all_machines()
    nb, _snum = D.numbering(rows)
    nmap = D.number_map(k, rows)
    sort_of = {num: what for num, _name, _qty, what in nb[k]}       # 'assembly', 'made' or 'bought'
    num0, name0 = nb[k][0][0], nb[k][0][1]

    mod = _load(k)
    mod.build()                                         # its own defaults; it opens its own design
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    root.partNumber = num0                              # the assembly itself, item 000
    root.description = name0

    numbered, with_part, left = [], {}, []
    occs, groups = 0, 0
    made, bought = {}, {}                               # number -> (description, the kinds that carry it)
    for c, n in _components(root):
        if c.bRepBodies.count == 0:                     # a group that only holds other components
            groups += 1
            continue
        kind = D.kind_of(c.name, nmap)
        if kind is None:                                # on no parts list: not a part
            left.append(c.name)
            continue
        num, desc = nmap[kind]
        c.description = desc
        if num is None:                                 # comes with a bought part: no number of its own
            with_part.setdefault(desc, []).append(c.name)
            continue
        c.partNumber = num
        numbered.append(c.name)
        occs += n
        (bought if sort_of.get(num) == 'bought' else made).setdefault(num, (desc, set()))[1].add(kind)

    found = set(k2 for v in list(made.values()) + list(bought.values()) for k2 in v[1])
    missing = sorted(nm for nm, (num, _d) in nmap.items() if num is not None and nm not in found)
    mk = sorted(made)
    lines = ['%s, %s: %d components numbered (%d occurrences), the root %s %s.'
             % (title, num0, len(numbered), occs, num0, name0)]
    lines.append('Made: %d kinds, %s to %s.' % (len(mk), mk[0], mk[-1]) if mk else 'Made: none.')
    bl = []
    for num in sorted(bought):
        desc, kinds = bought[num]
        extra = sorted(x for x in kinds if x != desc)
        bl.append('%s %s%s' % (num, desc, ' (as %s)' % ', '.join(extra) if extra else ''))
    lines.append('Bought: %d kinds: %s.' % (len(bought), '; '.join(bl)) if bl else 'Bought: none.')
    if with_part:
        lines.append('Come with a bought part, no number: %s.'
                     % '; '.join('%s (%s)' % (_names(v), d) for d, v in sorted(with_part.items())))
    else:
        lines.append('Come with a bought part: none.')
    lines.append('Left unnumbered, on no parts list: %s.' % (_names(left) if left else 'none'))
    if missing:
        lines.append('On the parts list but not found in the model: %s.' % ', '.join(missing))
    lines.append('Groups without a body of their own, not numbered: %d. Built by %s.py\'s own build() with its '
                 'defaults [%s].' % (groups, D.SCRIPTS[k], getattr(mod, 'VERSION', D.SCRIPTS[k])))
    lines.append('[%s]' % VERSION)
    _last.update(key=k, numbered=numbered, made=made, bought=bought, with_part=with_part, left=left,
                 missing=missing)
    return '\n'.join(lines)


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            item = args.command.commandInputs.itemById('assembly').selectedItem
            ui.messageBox(build(item.name if item else DEFAULT), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('assembly', 'The assembly', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k, title, _fname, _ch in D.MACHINES:
            dd.listItems.add(title, k == DEFAULT)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'PartNumbersCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Part numbers',
                                                         'Builds one of the nine assemblies and numbers its parts')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PartNumbers could not start:\n' + traceback.format_exc())
