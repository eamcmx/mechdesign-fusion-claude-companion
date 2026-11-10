# =============================================================================
# PartNumbers.py  -  Chapter 49 of "Mechanical Design in Autodesk Fusion with Claude": every part of an assembly given
# its part number and description in Fusion, so that a parts list drawn from the model carries them. The assembly is
# built by its own Volume 2 script; each component's name is matched to the name on that assembly's parts list, and
# lib/detail_inventory.py gives the number: a made part <machine><subassembly>-<item>, a bought part S-<serial> with
# its designation as the description. Written from PROMPT.txt beside it; checked by checks/part_numbers_check.py.
#
#   INPUTS        the dialog: the assembly (one of the nine of Volume 2 that Volume 3 details)
#   DERIVED       detail_inventory: numbering(), number_map(), kind_of(); the assembly's script and its build()
#   HARD RULES    a component whose name is on no parts list (the bag, the station's rollers) is left unnumbered and
#                 reported; a body that comes with a bought part gets no number of its own
#   CONVENTIONS   the assembly's own; the root component is the subassembly, item 000
#   ACCEPTANCE    the message; the check: every part numbered, one number for one kind, the counts of Chapter 49
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
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
import detail_inventory as D   # noqa: E402
importlib.reload(D)

VERSION = 'PartNumbers 1.1 (2026-11-03)'


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_' + name, CP.script(name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def number(root, key):
    """Write the part numbers into the components under root. Returns the components numbered, those that come with
    a bought part, and the names left unnumbered."""
    nmap = D.number_map(key)
    title = [m[1] for m in D.MACHINES if m[0] == key][0]
    root.partNumber = D.SUB[key] + '-000'
    root.description = title[4:].capitalize()
    done, part_of, left = {}, [], []
    for o in root.allOccurrences:
        comp = o.component
        if comp.bRepBodies.count == 0 or comp.entityToken in done:
            continue
        kind = D.kind_of(comp.name, nmap)
        if kind is None:
            left.append(comp.name)
            continue
        num, desc = nmap[kind]
        comp.description = desc
        if num is None:
            part_of.append(comp.name)
        else:
            comp.partNumber = num
        done[comp.entityToken] = (num, kind)
    return done, part_of, left


_last = {}


def build(key='bag'):
    if key not in D.SCRIPTS:
        raise ValueError('No assembly called %s: the nine are %s.' % (key, ', '.join(D.SCRIPTS)))
    mod = _load(D.SCRIPTS[key])
    mod.build()
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    done, part_of, left = number(root, key)
    nums = {v[0] for v in done.values() if v[0]}
    made = sorted(n for n in nums if not n.startswith('S-'))
    bought = sorted(n for n in nums if n.startswith('S-'))
    _last.update(key=key, done=done, part_of=part_of, left=left, made=made, bought=bought)
    return ('%s numbered %s-000: %d components; %d made kinds (%s to %s), %d bought kinds; %d bodies with a bought '
            'part; left unnumbered: %s.\n\n[%s]'
            % (root.description, D.SUB[key], len(done), len(made), made[0] if made else '-', made[-1] if made else '-',
               len(bought), len(part_of), ', '.join(sorted(set(left))) or 'none', VERSION))


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
        cid = 'PartNumbersCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Part numbers', 'Chapter 49: number an assembly\'s parts')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PartNumbers could not start:\n' + traceback.format_exc())
