# FatigueSpots (Chapter 60) against the acceptance criteria of its prompt: run by the dialog, as a user runs it, all
# the spots; each measure as the hand calculations of lib/fatigue_detail.py take it; the results kept in
# results/fatigue_spots.json; the refusal.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/FatigueSpots/FatigueSpots.py'


def run(_context):
    import adsk.core
    app = adsk.core.Application.get()
    before = [app.documents.item(i) for i in range(app.documents.count)]
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    for i in range(app.documents.count):           # every design the run built, closed after
        d = app.documents.item(i)
        if not any(d == b for b in before):
            d.attributes.add('mdfc', 'run', 'check')
    r = ns['_last']
    b = r['bolts']
    fc.ok(abs(b['grip'] - 28.6) < 0.01 and abs(b['head'] - 8.0) < 0.01 and abs(b['d'] - 8.0) < 0.01,
          'the bolts: the grip %.2f mm, the head %.2f, the shank %.2f across' % (b['grip'], b['head'], b['d']))
    for k, v in r['roots'].items():
        fc.ok(v['sharp'] and v['holes'] == 2, '%s: %d bolt holes, fillets %s, %s' % (
            k, v['holes'], v['fillets'] or 'none', 'sharp' if v['sharp'] else 'filleted'))
    k = r['keyseat']
    fc.ok(abs(k['d'] - 30) < 0.02 and abs(k['b'] - 8) < 0.02 and abs(k['h'] - 7) < 0.02 and abs(k['L'] - 25) < 0.02
          and abs(k['t1'] - 4.0) < 0.02,
          'the keyseat: the stub %.2f across, the key %.2f x %.2f x %.2f, the keyseat %.2f deep' % (k['d'], k['b'], k['h'], k['L'], k['t1']))
    g = r['groove']
    fc.ok(abs(g['d'] - 28.6) < 0.02 and g['beyond'],
          'the groove: %.2f across, from %.2f to %.2f mm out, the bearing %.2f to %.2f' % (g['d'], g['groove'][0], g['groove'][1],
                                                                                       g['bearing'][0], g['bearing'][1]))
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown spot refused')
    except ValueError as e:
        fc.ok('No spot called' in str(e), 'refused: %s' % e)
    fc.verdict()
