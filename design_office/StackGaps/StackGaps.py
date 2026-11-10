# =============================================================================
# StackGaps.py  -  Chapter 61 of "Mechanical Design in Autodesk Fusion with Claude": the small gaps Chapter 56's sweep
# found, measured with their direction. It builds each machine through RangeSweep.py's builders, puts it in the state of
# its closest approach, and asks Fusion for the minimum distance between the two parts and its two points; then in the
# states either side, to see how fast the gap closes with the motion. The stack-ups of lib/stackups.py take the gap's
# direction to know which dimensions close it. The results go to results/stack_gaps.json.
# Written from PROMPT.txt beside it; checked by checks/stack_gaps_check.py.
#
#   INPUTS        the dialog: the gap (all by default)
#   DERIVED       stackups.GAPS: each gap's machine, its state, its two parts and the states either side
#   HARD RULES    nothing of the machines' scripts copied; built through RangeSweep.machine()
#   CONVENTIONS   each machine's own new design document; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the check: each gap measured, its direction, its change with the motion
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
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

import stackups as ST     # noqa: E402
importlib.reload(ST)


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_sg61_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


VERSION = 'StackGaps 1.1 (2026-11-03)'
_last = {}


def measure(leaves, a, b, b_not=()):
    """The least distance between any body of a part named a... and any of a part named b... (not b_not...), and its
    two points."""
    mm = adsk.core.Application.get().measureManager
    A = [o for o in leaves if o.component.name.startswith(a)]
    B = [o for o in leaves if o.component.name.startswith(b) and not o.component.name.startswith(tuple(b_not))]
    best = None
    for oa in A:
        for ob in B:
            for ba in oa.bRepBodies:
                for bb in ob.bRepBodies:
                    r = mm.measureMinimumDistance(ba, bb)
                    d = r.value * 10.0
                    if best is None or d < best['d']:
                        p1, p2 = r.positionOne, r.positionTwo
                        best = dict(d=d, a=oa.component.name, b=ob.component.name,
                                    p1=[p1.x * 10, p1.y * 10, p1.z * 10], p2=[p2.x * 10, p2.y * 10, p2.z * 10])
    if best is None:
        raise RuntimeError('no parts named %s and %s' % (a, b))
    v = [best['p2'][i] - best['p1'][i] for i in range(3)]
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    best['dir'] = [x / n for x in v]
    return best


def build(which='all'):
    gaps = [g for g in ST.GAPS if which in ('all', g['key'])]
    if not gaps:
        raise ValueError('No gap called %s: Chapter 61 measures %s.' % (which, ', '.join(g['key'] for g in ST.GAPS)))
    RS = _load('RangeSweep')
    out = []
    machine, setter = None, None
    for g in gaps:
        if g['machine'] != machine:
            msg, setter = RS.machine(g['machine'])
            machine = g['machine']
        des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
        leaves = [o for o in des.rootComponent.allOccurrences if o.component.bRepBodies.count]
        rows = []
        for st in [g['state']] + g['around']:
            setter(tuple(st) if isinstance(st, list) else st)
            adsk.doEvents()
            m = measure(leaves, g['a'], g['b'], g.get('b_not', ()))
            m['state'] = st
            rows.append(m)
        setter(g['state'])
        out.append(dict(key=g['key'], machine=g['machine'], a=g['a'], b=g['b'], rows=rows))
    json.dump(out, open(os.path.join(PROJECT, 'results', 'stack_gaps.json'), 'w'), indent=1)
    _last.clear()
    _last.update(gaps=out)
    lines = ['%s: %s / %s %.3f mm along (%.2f, %.2f, %.2f)' % (o['key'], o['rows'][0]['a'], o['rows'][0]['b'], o['rows'][0]['d'],
                                                                *o['rows'][0]['dir']) for o in out]
    return 'Stack gaps: %d measured.\n%s\n\n[%s]' % (len(out), '\n'.join(lines), VERSION)


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.command.commandInputs.itemById('key').selectedItem.name))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('StackGaps failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('key', 'Gap', adsk.core.DropDownStyles.TextListDropDownStyle)
        dd.listItems.add('all', True)
        for g in ST.GAPS:
            dd.listItems.add(g['key'], False)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcStackGaps'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Stack gaps', 'Chapter 61: the sweep\'s small gaps measured')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StackGaps failed:\n%s' % traceback.format_exc())
