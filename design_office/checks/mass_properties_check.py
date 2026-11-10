# MassProperties (Chapter 57) against the acceptance criteria of its prompt: the robot by the dialog, as a user runs it,
# then the other eight machines by build(); each one weighed, its groups' masses summing to its total, its file
# written; the robot's eleven moving bodies written in Chapter 43's form, each within 10 % of the concept's mass and of
# its moments of inertia; the refusal.
import sys, importlib, json, os
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/design_office/MassProperties/MassProperties.py'
ORDER = ['robot', 'wrist', 'flap', 'axle', 'bag', 'clamp', 'vacuum', 'changer', 'unit']
INERTIA = ('Ia', 'It', 'In', 'Izz')


def report(ns, key):
    r = dict(ns['_last'])
    gs = sum(g['m'] for g in r['groups'].values())
    fc.ok(r['total']['m'] > 0 and abs(gs - r['total']['m']) < 1e-3 * r['total']['m']
          and os.path.exists('__COMPANION__/design_office/results/mass_%s.json' % key),
          'the %s: %d parts, %.3f kg, its %d groups %.3f kg; centre (%.1f, %.1f, %.1f) mm'
          % (key, r['total']['parts'], r['total']['m'], len(r['groups']), gs, *r['total']['c']))
    return r


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    r = report(ns, 'robot')
    b = r.get('bodies') or []
    fc.ok(len(b) == 11 and all(x['m'] > 0 for x in b),
          'the robot\'s moving bodies in Chapter 43\'s form: %s' % ['%s %.1f' % (x['name'], x['m']) for x in b])
    concept = {x['name']: x for x in json.load(open('__COMPANION__/projects/palletizer/lib/palletizer_weighed.json'))['bodies']} \
        if os.path.exists('__COMPANION__/projects/palletizer/lib/palletizer_weighed.json') else {}
    worst = []
    for x in b:
        c = concept.get(x['name'])
        if not c:
            continue
        for k in ('m',) + INERTIA:
            if k in c and k in x and c[k] > 0.05:
                worst.append((abs(x[k] / c[k] - 1.0), x['name'], k))
    worst.sort(reverse=True)
    fc.ok(concept and worst and worst[0][0] < 0.10,
          'each body within 10 %% of the concept\'s mass and moments of inertia; the most changed: %s'
          % ['%s %s %+.1f %%' % (n, k, 100 * d) for d, n, k in worst[:3]])
    for k in ORDER[1:]:
        print('  ' + ns['build'](k).split('\n')[0][:160])
        fc.mark()
        report(ns, k)
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown machine refused')
    except ValueError as e:
        fc.ok('No machine called' in str(e), 'refused: %s' % e)
    fc.verdict()
