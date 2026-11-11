# =============================================================================
# StackGaps.py  -  Chapter 61 of "Mechanical Design in Autodesk Fusion with Claude": the small gaps Chapter 56's range
# sweep found, measured with their direction so that the tolerance stack-ups know which dimensions close them. For each
# gap of lib/stackups.py's GAPS it builds the gap's machine through RangeSweep.py's machine() (loaded, not copied; each
# machine once for all its gaps), puts it in the state of the closest approach, measures Fusion's minimum distance
# between every body of the two parts, keeps the least with its two points and the unit vector from the first to the
# second, then measures again in the states either side, and returns the machine to the gap's own state.
# Written from PROMPT.txt (design_office/StackGaps/PROMPT.txt).
#
#   INPUTS        the dialog: the gap (all by default)
#   DERIVED       stackups: GAPS, FLAP_STOP
#   HARD RULES    machines built by RangeSweep.py's machine(); every body of every part by its name's start; the
#                 machine back in the gap's state; every gap's rows in design_office/results/stack_gaps.json
#   CONVENTIONS   each machine's own new design document, made by its own script; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; each gap as the sweep found it; the states either side; a gap called crane refused
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
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script

import stackups as SU      # noqa: E402
importlib.reload(SU)

VERSION = 'StackGaps 1.0 (2026-11-10)'
KEYS = tuple(g['key'] for g in SU.GAPS)                 # the six gaps, in the order of GAPS
CHOICES = ('all',) + KEYS
OUT = os.path.join(CP.ROOT, 'design_office', 'results', 'stack_gaps.json')
UNIT = {'float': 'mm'}                                  # the motion's unit by the state's name; the others turn, in deg
_last = {}


def _load(name):
    """A script of the companion loaded as a module from its own file (CP.script finds it)."""
    spec = importlib.util.spec_from_file_location('mdfc_sg61_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _design():
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


# ---------- the parts of a gap, and the least distance between them ----------
def _named(o, start, leave_out):
    """An occurrence whose part's name starts with `start` and neither it nor a part it sits in starts with a name
    left out."""
    if not o.component.name.startswith(start):
        return False
    if not leave_out:
        return True
    lo = tuple(leave_out)
    if o.component.name.startswith(lo):
        return False
    return not any(p.startswith(lo) for p in o.fullPathName.split('+'))


def _measure(g, state):
    """Fusion's minimum distance between every body of every part named g['a'] and every body of every part named
    g['b'], in the machine's present state: the least, its two points (mm) and the unit vector from the first to the
    second."""
    app = adsk.core.Application.get()
    mm = app.measureManager
    leaves = [o for o in _design().rootComponent.allOccurrences if o.component.bRepBodies.count]
    A = [o for o in leaves if _named(o, g['a'], g.get('a_not', ()))]
    B = [o for o in leaves if _named(o, g['b'], g.get('b_not', ()))]
    if not A or not B:
        raise RuntimeError('The %s: no part named %s...' % (g['key'], g['a'] if not A else g['b']))
    best, n, failed = None, 0, 0
    for oa in A:
        for ob in B:
            if oa.fullPathName == ob.fullPathName:
                continue
            for ba in oa.bRepBodies:
                for bb in ob.bRepBodies:
                    try:
                        r = mm.measureMinimumDistance(ba, bb)
                    except Exception:
                        failed += 1
                        continue
                    n += 1
                    d = r.value * 10.0
                    if best is None or d < best[0]:
                        best = (d, oa, ob, r.positionOne, r.positionTwo)
    if best is None:
        raise RuntimeError('The %s: Fusion measured none of the %d body pairs' % (g['key'], failed))
    d, oa, ob, q1, q2 = best
    p1 = (q1.x * 10.0, q1.y * 10.0, q1.z * 10.0)
    p2 = (q2.x * 10.0, q2.y * 10.0, q2.z * 10.0)
    v = [p2[i] - p1[i] for i in range(3)]
    L = math.sqrt(sum(c * c for c in v))
    u = tuple(c / L for c in v) if L > 1e-9 else (0.0, 0.0, 0.0)
    return dict(state=state, a=oa.component.name, b=ob.component.name, a_path=oa.fullPathName, b_path=ob.fullPathName,
                d=d, p1=p1, p2=p2, dir=u, parts=(len(A), len(B)), pairs=n, failed=failed)


# ---------- the states either side: how fast the gap closes ----------
def _coords(s):
    v = s[1]
    return list(v) if isinstance(v, (list, tuple)) else [v]


def _rate(s0, s1, d0, d1):
    """The gap's change per unit of the motion between two states that differ in one number only; None otherwise
    (a lock against an unlock, or two motions at once)."""
    c0, c1 = _coords(s0), _coords(s1)
    if len(c0) != len(c1):
        return None
    moved = []
    for a, b in zip(c0, c1):
        if isinstance(a, bool) or isinstance(b, bool):
            if a != b:
                return None
            continue
        if abs(float(b) - float(a)) > 1e-12:
            moved.append(float(b) - float(a))
    if len(moved) != 1:
        return None
    return (d1 - d0) / moved[0]


def _st(s):
    vs = _coords(s)
    return '%s %s' % (s[0], ', '.join(str(x) if isinstance(x, bool) else '%g' % x for x in vs))


def _sweep_found(g):
    """The least gap the range sweep reported for the same two parts (results/range_sweep.json), or None."""
    try:
        rs = json.load(open(CP.result('range_sweep.json'), encoding='utf-8'))
        rows = rs.get(g['machine'], {}).get('rows', [])
    except Exception:
        return None
    a_not, b_not = tuple(g.get('a_not', ())), tuple(g.get('b_not', ()))

    def isa(n):
        return n.startswith(g['a']) and not (a_not and n.startswith(a_not))

    def isb(n):
        return n.startswith(g['b']) and not (b_not and n.startswith(b_not))
    hits = [(r['min'], r.get('at')) for r in rows
            if (isa(r.get('a', '')) and isb(r.get('b', ''))) or (isb(r.get('a', '')) and isa(r.get('b', '')))]
    return min(hits, key=lambda x: x[0]) if hits else None


# ---------- the results file ----------
def _write(gaps):
    """Every gap's rows to design_office/results/stack_gaps.json, in the order of GAPS; a run of one gap keeps the
    other gaps already in the file."""
    keep = []
    if len(gaps) < len(KEYS) and os.path.isfile(OUT):
        try:
            old = json.load(open(OUT, encoding='utf-8'))
            mine = set(g['key'] for g in gaps)
            if isinstance(old, list):
                keep = [g for g in old if isinstance(g, dict) and g.get('key') in KEYS and g.get('key') not in mine]
        except Exception:
            keep = []
    rows = sorted(keep + list(gaps), key=lambda g: KEYS.index(g['key']))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8') as f:
        json.dump(rows, f, indent=1)
    return OUT


# ---------- the build ----------
def build(gap='all'):
    if gap not in CHOICES:
        raise ValueError('No gap called %s: Chapter 61 measures the %s gaps, or all.' % (gap, ', '.join(KEYS)))
    todo = [g for g in SU.GAPS if gap == 'all' or g['key'] == gap]
    machines = []
    for g in todo:                                      # each machine once, in the order of GAPS
        if g['machine'] not in machines:
            machines.append(g['machine'])
    RS56 = _load('RangeSweep')
    out, built = [], []
    for mk in machines:
        msg, setter = RS56.machine(mk)                  # its own script makes its own new design document
        adsk.doEvents()
        built.append((mk, msg.split('\n')[0]))
        for g in [x for x in todo if x['machine'] == mk]:
            rows = []
            for s in [g['state']] + list(g.get('around', [])):
                setter(s)
                adsk.doEvents()
                rows.append(_measure(g, s))
            setter(g['state'])                          # the machine back in the gap's own state
            adsk.doEvents()
            r0 = rows[0]
            for r in rows[1:]:
                r['change'] = r['d'] - r0['d']
                r['rate'] = _rate(r0['state'], r['state'], r0['d'], r['d'])
            sw = _sweep_found(g)
            out.append(dict(key=g['key'], machine=mk, a=g['a'], b=g['b'], b_not=list(g.get('b_not', ())),
                            state=g['state'], d=r0['d'], dir=r0['dir'], sweep=sw[0] if sw else None,
                            sweep_at=sw[1] if sw else None, rows=rows))
    path = _write(out)
    _last.clear()
    _last.update(gaps=out, machines=built, file=path, flap_stop=SU.FLAP_STOP)
    lines = []
    for g in out:
        r0 = g['rows'][0]
        lines.append('The %s (%s / %s%s): %.3f mm at %s, between %s and %s, from (%.2f, %.2f, %.2f) to '
                     '(%.2f, %.2f, %.2f) mm, along (%.4f, %.4f, %.4f)%s.'
                     % (g['key'], g['a'], g['b'], ', not %s' % ', '.join(g['b_not']) if g['b_not'] else '',
                        r0['d'], _st(r0['state']), r0['a'], r0['b'], r0['p1'][0], r0['p1'][1], r0['p1'][2],
                        r0['p2'][0], r0['p2'][1], r0['p2'][2], r0['dir'][0], r0['dir'][1], r0['dir'][2],
                        '; the sweep found %.3f mm' % g['sweep'] if g['sweep'] is not None else ''))
        for r in g['rows'][1:]:
            unit = UNIT.get(r['state'][0], 'deg')
            lines.append('    at %s: %.3f mm (%+.3f mm%s), between %s and %s, along (%.4f, %.4f, %.4f)'
                         % (_st(r['state']), r['d'], r['change'],
                            ', %+.4f mm per %s' % (r['rate'], unit) if r['rate'] is not None else '',
                            r['a'], r['b'], r['dir'][0], r['dir'][1], r['dir'][2]))
        if r0['failed']:
            lines.append('    (%d body pairs Fusion could not measure)' % r0['failed'])
    head = ('Stack-up gaps: %d gap%s on %d machine%s (%s), each measured at the range sweep\'s closest approach and in '
            'the states either side; the flap\'s end stop at %.2f deg. Rows in %s.'
            % (len(out), '' if len(out) == 1 else 's', len(built), '' if len(built) == 1 else 's',
               ', '.join(k for k, _ in built), SU.FLAP_STOP, os.path.relpath(path, CP.ROOT)))
    return '%s\n\n%s\n\n[%s]' % (head, '\n'.join(lines), VERSION)


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            key = args.command.commandInputs.itemById('gap').selectedItem.name
            ui.messageBox(build(key))
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
        dd = cmd.commandInputs.addDropDownCommandInput('gap', 'Gap', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in CHOICES:
            dd.listItems.add(k, k == 'all')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcStackGaps'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Stack gaps',
                                                         'Chapter 61: the range sweep\'s small gaps, measured with their direction')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StackGaps failed:\n%s' % traceback.format_exc())
