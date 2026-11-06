# DesignForAssembly (Chapter 65) against the acceptance criteria of its prompt: run by the dialog, as a user runs it;
# the answers read back from the model against lib/dfma.py, every line of the bill answered, the three fixings in the
# model, no interference of the new screws, the bill after the fixings.
import sys, importlib, os, json
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import dfma as DF
importlib.reload(DF)

PATH = '__COMPANION__/scripts/DesignForAssembly/DesignForAssembly.py'
EDITS = ()                                  # a control replaces this with its one broken line


def run(_context):
    ns = fc.load(PATH, EDITS)
    print(ns['VERSION'])
    lib = DF.bag_test()                     # the lib's answers before the script runs: a control's edit stays in it
    after = DF.redesign()
    fc.run_dialog(ns)
    r = ns['_last']
    b = r['before']
    fc.ok(b['N'] == lib['N'] == 114 and r['tagged'] == b['N'],
          'every piece answered in the model: %d tagged, %d read back, the lib %d' % (r['tagged'], b['N'], lib['N']))
    fc.ok(sorted(r['lines_before']) == sorted(l[0] for l in DF.BAG),
          'every first-level line of the bill has its answer (%d lines)' % len(r['lines_before']))
    fc.ok(b['Nmin'] == lib['Nmin'] == 19, 'the theoretical minimum read back: %d, the lib %d' % (b['Nmin'], lib['Nmin']))
    fc.ok(b['by_q'] == lib['by_q'], 'the questions that keep them: %s, the lib %s' % (b['by_q'], lib['by_q']))
    E = DF.T0 * b['Nmin'] / lib['t']
    fc.ok(abs(E - lib['E']) < 1e-9 and abs(lib['t'] - 640.2) < 0.01,
          'design efficiency 3 x %d / %.2f s = %.4f, the lib %.4f' % (b['Nmin'], lib['t'], E, lib['E']))
    f = r['fixed']
    fc.ok(f['tines'] == 16 and f['blocks'] == 2, 'joined: %d tines to their tubes, %d end blocks to the end plates'
          % (f['tines'], f['blocks']))
    fc.ok(sorted(f['removed']) == sorted('Cap screw M8x30 %d' % k for k in range(1, 9)) and len(f['added']) == 8,
          'the eight M8 x 30 between plates and blocks removed, %d screws added: %s' % (len(f['added']), f['added']))
    la = r['lines_after']
    fc.ok('P13-012' not in la and 'P13-004' not in la and la.get('S-0031') == 10 and la.get('S-0039') == 4,
          'the bill after: no tine or end block line, M8 x 20 %s, M8 x 30 %s' % (la.get('S-0031'), la.get('S-0039')))
    fc.ok(r['pieces_after'] == after['N'] == 96, 'pieces after the fixings: %d in the model, the lib %d'
          % (r['pieces_after'], after['N']))
    fc.ok(not r['interference'], 'the new screws touch nothing they should not: %s (of %d interferences in the gripper)'
          % (r['interference'], r['interferences_all']))
    fc.ok(abs(r['mass_after'] - r['mass_before']) < 0.2, 'mass %.3f kg before, %.3f kg after'
          % (r['mass_before'], r['mass_after']))
    fc.ok(os.path.exists(os.path.join('__COMPANION__', 'results', 'dfa.json')), 'results/dfa.json written')
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown machine refused')
    except ValueError as e:
        fc.ok('No answers for' in str(e), 'refused: %s' % e)
    fc.verdict()
