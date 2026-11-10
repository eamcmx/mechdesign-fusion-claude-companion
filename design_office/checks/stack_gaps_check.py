# StackGaps (Chapter 61) against the acceptance criteria of its prompt: run by the dialog, as a user runs it, all the
# gaps; each measured at its state as Chapter 56's sweep found it, with its direction and its change in the states
# either side; the results kept in results/stack_gaps.json; the refusal.
import sys, importlib, json
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/design_office/StackGaps/StackGaps.py'
SWEEP = {'unit': 0.707, 'flap': 0.859, 'wrist': 1.000, 'changer': 1.500, 'axle_casting': 1.000, 'axle_cap': 1.000}


def run(_context):
    import adsk.core
    app = adsk.core.Application.get()
    before = [app.documents.item(i) for i in range(app.documents.count)]
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    for i in range(app.documents.count):           # each machine's design, built in its own document, closed after
        d = app.documents.item(i)
        if not any(d == b for b in before):
            d.attributes.add('mdfc', 'run', 'check')
    for g in ns['_last']['gaps']:
        r0 = g['rows'][0]
        fc.ok(abs(r0['d'] - SWEEP[g['key']]) < 0.002,
              'the %s: %s / %s %.3f mm at %s, as the sweep found, along (%.3f, %.3f, %.3f) from (%.1f, %.1f, %.1f)'
              % (g['key'], r0['a'], r0['b'], r0['d'], r0['state'], r0['dir'][0], r0['dir'][1], r0['dir'][2], *r0['p1']))
        for r in g['rows'][1:]:
            print('       at %s: %.3f mm, %s / %s' % (r['state'], r['d'], r['a'], r['b']))
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown gap refused')
    except ValueError as e:
        fc.ok('No gap called' in str(e), 'refused: %s' % e)
    fc.verdict()
