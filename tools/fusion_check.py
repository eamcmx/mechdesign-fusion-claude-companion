# Shared helpers for the checks in each project's checks/ and controls/. A check runs inside Fusion and imports this module with
#     import sys, importlib; sys.path.insert(0, '__COMPANION__/tools'); import fusion_check as fc; importlib.reload(fc)
# (run_check.py replaces __COMPANION__ with the repository's folder; reload picks up edits, Fusion keeps modules).
import math
import time

import adsk.core
import adsk.fusion

fails = []


def ok(cond, msg):
    """Print one check line and remember a failure."""
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        fails.append(msg)
    return cond


messages = []          # what the script under test showed in its message boxes, in order
loaded = []            # the scripts loaded by load(), the one under test first


def _message(*args, **kwargs):
    """The script's ui.messageBox, recorded and printed instead of shown."""
    text = str(args[0]) if args else ''
    messages.append(text)
    print(text)
    return adsk.core.DialogResults.DialogOK


exempt = []           # why the script's message is not tested here (a helper that calls its functions, not its dialog)


def no_message(why):
    """For a helper file kept with the checks that runs a script's functions but never its dialog."""
    exempt.append(why)


def message_ok():
    """Every prompt's first acceptance criterion: a message shows what was built and the script's version tag."""
    if exempt:
        print('  --   no message tested: ' + exempt[0])
        return True
    if not loaded or 'VERSION' not in loaded[0]:
        return True
    v = loaded[0]['VERSION']
    return ok(any(v in m for m in messages), 'a message gives the version tag %s' % v)


def verdict():
    message_ok()
    print('ALL PASS' if not fails else '%d FAILED' % len(fails))


def load(path, edits=()):
    """Load a script's source as a namespace, its message boxes printed instead of shown. `edits` = (old, new) pairs
    applied first; a control uses them to break one line on purpose (each old text must be found exactly once)."""
    src = open(path, encoding='utf-8').read()
    for old, new in edits:
        if src.count(old) != 1:
            raise RuntimeError('control edit not found exactly once: %r' % old[:60])
        src = src.replace(old, new)
    src = src.replace('ui.messageBox(', '_mdfc_message(')
    ns = {'__name__': 'mdfc_check', '__file__': path, '_mdfc_message': _message}
    exec(compile(src, path, 'exec'), ns)
    if callable(ns.get('build')):              # a check that calls build() itself sees the message run() would show
        built = ns['build']

        def build(*a, **k):
            r = built(*a, **k)
            if isinstance(r, str):
                messages.append(r)
            return r
        ns['build'] = build
    loaded.append(ns)
    return ns


def run_dialog(ns, values=None):
    """Run the script the way a user does: run(), its dialog opens, values typed in (by input id), OK pressed.
    Returns the new design's document, marked for cleanup."""
    app = adsk.core.Application.get()
    seen = []
    base = ns['_Created']

    class Seen(base):
        def notify(self, args):
            base.notify(self, args)
            seen.append(args.command)
    ns['_Created'] = Seen
    before = app.activeDocument
    if before is not None:
        before.attributes.add('mdfc', 'before', 'yes')     # Fusion may close an empty start-up design, so count is no test
    ns['run'](None)
    t_open = time.time()                    # the first command a Fusion session makes can take seconds to open
    while not seen and time.time() - t_open < 30.0:
        adsk.doEvents()
        time.sleep(0.02)
    ok(bool(seen), 'Run opens the dialog')
    if not seen:
        return None
    ins = seen[0].commandInputs
    for k, v in (values or {}).items():
        ci = ins.itemById(k)
        if hasattr(ci, 'expression'):
            ci.expression = str(v)
        else:
            ci.value = v
    t0 = time.time()
    seen[0].doExecute(True)
    for _ in range(30):
        adsk.doEvents()
    print('  OK pressed, built in %.0f s' % (time.time() - t0))
    new = app.activeDocument
    ok(new is not None and new.attributes.itemByName('mdfc', 'before') is None, 'a new design opened')
    if before is not None and before.isValid:
        a = before.attributes.itemByName('mdfc', 'before')
        if a is not None:
            a.deleteMe()
    mark()
    return app.activeDocument


def mark():
    """Mark the active design so that run_check.py --cleanup closes it (and nothing else)."""
    adsk.core.Application.get().activeDocument.attributes.add('mdfc', 'run', 'check')


def design():
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


def occ(name):
    """The top-level occurrence whose component has this name."""
    for o in design().rootComponent.occurrences:
        if o.component.name == name:
            return o
    raise RuntimeError('no component named %r' % name)


def position_mm(o):
    t = o.transform2.translation
    return (t.x * 10, t.y * 10, t.z * 10)


def volume_mm3(o):
    return sum(b.volume for b in o.component.bRepBodies) * 1000.0


def interference(occs=None):
    """Clashes among the given occurrences (all top-level ones by default): [(name, name, mm^3)], touching faces ignored."""
    des = design()
    col = adsk.core.ObjectCollection.create()
    for o in (occs if occs is not None else list(des.rootComponent.occurrences)):
        col.add(o)
    inp = des.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = False
    res = des.analyzeInterference(inp)
    return [(r.entityOne.name, r.entityTwo.name, round(r.interferenceBody.volume * 1000, 3)) for r in res] if res else []


def rotation(o):
    t = o.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def turn_deg(R1, R0, axis):
    """Angle in degrees of the rotation R1 R0^T about a world axis (right-handed)."""
    D = [[sum(R1[a][k] * R0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
    w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
    s = sum(w[i] * axis[i] for i in range(3)) / 2.0
    c = (D[0][0] + D[1][1] + D[2][2] - 1.0) / 2.0
    return math.degrees(math.atan2(s, c))


def joint(name):
    root = design().rootComponent
    for j in list(root.asBuiltJoints) + list(root.joints):
        if j.name == name:
            return j
    raise RuntimeError('no joint named %r' % name)


def refused(ns, word, **args):
    """build(**args) must raise ValueError mentioning `word`."""
    try:
        ns['build'](**args)
        mark()
        return ok(False, '%s refused' % args)
    except ValueError as e:
        return ok(word in str(e), '%s refused: %s' % (args, e))
