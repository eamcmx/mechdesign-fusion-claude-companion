# RangeSweep (Chapter 56) against the acceptance criteria of its prompt: the wrist by the dialog, as a user runs it,
# then the other machines by build(), the bag gripper twice, as Part 9 left it (bag0) and with its stops (bag); for
# each, no interference at any state, the contacts made in motion and the closest approaches; for the bag gripper's
# stops, each absorber touching its contact at its end of travel and nothing else; every result kept in
# results/range_sweep.json for the chapter. This check sweeps the first six machines, checks/range_sweep_rest_check.py
# the other four, so that neither run keeps Fusion busy for more than half an hour.
import sys, importlib, json
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/design_office/RangeSweep/RangeSweep.py'
OUT = '__COMPANION__/design_office/results/range_sweep.json'
ORDER = ['wrist', 'flap', 'axle', 'robot', 'bag0', 'bag', 'clamp', 'vacuum', 'changer', 'unit']
KEYS = ORDER[:6]        # the rest, ORDER[6:], by checks/range_sweep_rest_check.py: each half within half an hour


def report(ns, key, out):
    r = dict(ns['_last'])
    rows = [dict(a=x['pair'][0], b=x['pair'][1], min=round(x['min'], 3), at=list(x['at']) if isinstance(x['at'], tuple) else x['at'],
                 max=round(x['max'], 3), seen=x['seen']) for x in r['rows']]
    out[key] = dict(states=r['states'], parts=r['parts'], groups=r['groups'], hits=[[str(s), h] for s, h in r['hits']],
                    rows=rows, seconds=round(r['seconds'], 1), pairs=r.get('pairs'))
    fc.ok(not r['hits'], 'the %s: %d states, %d parts in %d groups, no interference %s'
          % (key, r['states'], r['parts'], r['groups'], r['hits'][:2] or ''))
    con = [x for x in rows if x['min'] < 0.01]
    near = [x for x in rows if x['min'] >= 0.01][:3]
    print('       contacts made in motion: %d %s' % (len(con), ['%s / %s at %s' % (x['a'], x['b'], x['at'][1]) for x in con[:6]]))
    print('       closest: %s' % ['%s / %s %.3f mm at %s' % (x['a'], x['b'], x['min'], x['at'][1]) for x in near])
    print('       pairs: %s, %.0f s' % (r.get('pairs'), r['seconds']))
    if key == 'bag':
        st = lambda x, side: x['a'].startswith('Shock absorber') and (' %s ' % side) in x['a'] or \
            x['b'].startswith('Shock absorber') and (' %s ' % side) in x['b']
        lev = lambda x: x['a'].startswith('Lever') or x['b'].startswith('Lever')
        cl = [x for x in con if st(x, 'closed') and lev(x) and x['at'][1] == 0.0]
        op = [x for x in con if st(x, 'open') and lev(x) and x['at'][1] == 45.0]
        other = [x for x in con if (x['a'].startswith(('Shock', 'Stop', 'Lock')) or x['b'].startswith(('Shock', 'Stop', 'Lock')))
                 and x not in cl and x not in op]
        fc.ok(len(cl) == 2 and len(op) == 2 and not other,
              'the stops: the closing absorbers on the strikers closed, the opening ones on the bars at 45 deg, '
              'touching nothing else %s' % ['%s / %s at %s' % (x['a'], x['b'], x['at'][1]) for x in other])
        sn = [x for x in rows if x['min'] >= 0.01 and (x['a'].startswith(('Shock', 'Stop', 'Lock', 'Cap screw ISO 4762 M8x40 stop'))
                                                     or x['b'].startswith(('Shock', 'Stop', 'Lock', 'Cap screw ISO 4762 M8x40 stop')))]
        print('       the stops\' closest: %s' % ['%s / %s %.3f mm at %s' % (x['a'], x['b'], x['min'], x['at'][1]) for x in sn[:4]])


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    try:                                         # the other half's machines kept: the file holds all ten
        out = json.load(open(OUT))
    except Exception:
        out = {}
    for k in KEYS:
        if k == 'wrist':
            fc.run_dialog(ns)                    # the default machine, by the dialog, as a user runs it
        else:
            print('  ' + ns['build'](k).split('\n')[0][:160])
            fc.mark()
        report(ns, k, out)
        json.dump({m: out[m] for m in ORDER if m in out}, open(OUT, 'w'), indent=1)
    if KEYS[-1] == ORDER[-1]:
        try:
            ns['build']('crane')
            fc.mark()
            fc.ok(False, 'an unknown machine refused')
        except ValueError as e:
            fc.ok('No machine called' in str(e), 'refused: %s' % e)
    fc.verdict()
