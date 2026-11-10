# BillOfMaterials (Chapter 64) against the acceptance criteria of its prompt: run by the dialog, as a user runs it, all
# nine machines; each list set against Fusion's own census and masses, the numbers and the components read back, the
# sets, the weldment, the loads, the CSV files, the refusal.
import sys, importlib, os
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import bom_detail as BD
importlib.reload(BD)

PATH = '__COMPANION__/design_office/BillOfMaterials/BillOfMaterials.py'
EDITS = ()                                  # a control replaces this with its one broken line


def run(_context):
    import adsk.core
    app = adsk.core.Application.get()
    ns = fc.load(PATH, EDITS)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    ms = {m['key']: m for m in ns['_last']['machines']}
    sup = BD.superseded()
    fc.ok(sorted(ms) == sorted(ns['MACHINES']) and all(os.path.exists(os.path.join('__COMPANION__', 'design_office', 'results', 'bom', m['csv']))
                                                     for m in ms.values()),
          'nine lists, each in results/bom: %s' % ', '.join(m['csv'] for m in ms.values()))
    for key in ns['MACHINES']:
        m = ms[key]
        top = [l for l in m['lines'] if not l['kind'].startswith('in ')]
        acc = sum(l['drawn'] for l in top) + m['load_mass']
        fc.ok(abs(acc - m['fusion_mass']) < 1e-3 * max(1.0, m['fusion_mass']),
              '%s: %d occurrences, the lines and %d loads %.4f kg, Fusion\'s design %.4f kg' % (
                  key, m['occurrences'], len(m['loads']), acc, m['fusion_mass']))
        bad = [l for l in m['lines'] if not l['number'] and not (l['kind'].startswith('with') or
                                                                (l['kind'].startswith('in ') and 'weld' in l['description']))]
        old = [l['number'] for l in m['lines'] if l['number'] in sup]
        fc.ok(not bad and not old and not m['wrong_numbers'],
              '%s: every line numbered, no superseded number, every component read back (%s %s %s)' % (
                  key, [l['description'] for l in bad], old, m['wrong_numbers']))
    c = ms['changer']
    fc.ok((c['made_kinds'], c['made_pieces'], c['bought_kinds'], c['bought_pieces']) == (7, 12, 3, 9),
          'the tool changer: made %d kinds in %d pieces, bought %d kinds in %d' % (
              c['made_kinds'], c['made_pieces'], c['bought_kinds'], c['bought_pieces']))
    b = ms['bag']
    wl = [l for l in b['lines'] if l['number'] == 'P13-016']
    parts = {l['number'] or 'welds': (l['qty'], l['per']) for l in b['lines'] if l['kind'] == 'in P13-016'}
    fc.ok(len(wl) == 1 and wl[0]['qty'] == 2 and parts == {'P13-013': (2, 1.0), 'P13-010': (2, 1.0), 'P13-011': (2, 1.0),
                                                           'welds': (4, 2.0)},
          'the bag gripper: the weldment P13-016 x%s, under it %s' % (wl[0]['qty'] if wl else None, parts))
    fc.ok(len(b['loads']) == 10 and sorted({n.split(' ')[0] for n, _ in b['loads']}) == ['Bag', 'Roller'],
          'the bag gripper: %d loads left out, %.2f kg (%s)' % (len(b['loads']), b['load_mass'],
                                                              ', '.join(sorted({n.split(' ')[0] for n, _ in b['loads']}))))
    q = lambda key, num: sum(l['qty'] for l in ms[key]['lines'] if l['number'] == num)
    fc.ok((q('axle', 'S-0017'), q('axle', 'S-0008'), q('axle', 'S-0009'), q('axle', 'S-0010')) == (2, 2, 1, 1),
          "the axle's bearings counted by sets: 6213 x%d, 32213 x%d, 32308 x%d, 32309 x%d" % (
              q('axle', 'S-0017'), q('axle', 'S-0008'), q('axle', 'S-0009'), q('axle', 'S-0010')))
    fc.ok((q('robot', 'S-0078'), q('robot', 'S-0079'), q('robot', 'S-0080')) == (2, 1, 1),
          "the robot's reducers counted by sets: RV-500N x%d, RV-700N x%d, RV-80N x%d" % (
              q('robot', 'S-0078'), q('robot', 'S-0079'), q('robot', 'S-0080')))
    mk = [(l['number'], l['maker'][1]) for m in ms.values() for l in m['lines'] if l.get('maker')]
    gone = sorted({p for n, p in mk if not os.path.exists(os.path.join(BD.REFS, p))})
    fc.ok(not gone, "the makers' masses: %d lines, every page on disk %s" % (len(mk), gone or ''))
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown machine refused')
    except ValueError as e:
        fc.ok('No machine called' in str(e), 'refused: %s' % e)
    fc.verdict()
