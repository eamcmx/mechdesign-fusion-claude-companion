# StopBracket (Chapter 62) against the acceptance criteria of its prompt: run by the dialog, as a user runs it, with
# its default 2.5 mm root; on each opening bracket the arm's sides against the absorber's axis, the root's faces, the
# volume against lib/lugs_brackets.py and the material, the gaps from its two M8 heads to the arm; the bag gripper's
# sweep: no interference, and Chapter 56's four contacts made in motion and no others; the refusal.
import sys, importlib, json, os
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import lugs_brackets as LB
importlib.reload(LB)

PATH = '__COMPANION__/scripts/StopBracket/StopBracket.py'
OUT = os.path.join('__COMPANION__', 'results', 'stop_bracket.json')
EDITS = ()                                  # a control replaces this with its one broken line
CONTACTS = sorted([('Shock absorber DYSC-25-25-Y1F closed 1', 'Lever A'), ('Shock absorber DYSC-25-25-Y1F closed 2', 'Lever B'),
                   ('Shock absorber DYSC-25-25-Y1F open 1', 'Lever A'), ('Shock absorber DYSC-25-25-Y1F open 2', 'Lever B')])


def run(_context):
    ns = fc.load(PATH, EDITS)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    r = dict(ns['_last'])
    R = 2.5
    fc.ok(abs(r['r'] - R) < 1e-9, "the dialog's root radius: %g mm" % r['r'])
    hv = LB.volume(True, R)
    cl = LB.clearance(True, R)
    for b in r['brackets']:
        fc.ok(b['sides'] == 4 and b['worst'] is not None and b['worst'] < 0.01,
              '%s: %d sides of the arm, the worst %.4f deg off square to the push' % (b['name'], b['sides'], b['worst'] if b['worst'] is not None else -1))
        fc.ok(b['fillets'] == 4, '%s: the root, %d faces of radius %g' % (b['name'], b['fillets'], R))
        fc.ok(abs(b['volume'] - hv) <= 1e-4 * hv and '1045' in (b['material'] or ''),
              '%s: %.1f mm3 in the model, %.1f by hand (%.4f %%), %.4f kg, %s' % (
                  b['name'], b['volume'], hv, 100 * (b['volume'] - hv) / hv, b['mass'], b['material']))
        fc.ok(min(b['gaps']) >= cl['toe_drawn'] - 0.01,
              '%s: the M8 heads %.2f and %.2f mm from the arm (by hand %.2f, the heads drawn 12 across)' % (
                  b['name'], b['gaps'][0], b['gaps'][1], cl['toe_drawn']))
    sw = r['sweep']
    fc.ok(not sw['hits'], 'the sweep: %d states, %d parts, %s' % (
        sw['states'], sw['parts'], 'no interference' if not sw['hits'] else 'interference: %s' % sw['hits'][:2]))
    got = sorted(tuple(c) for c in sw['contacts'])
    fc.ok(got == CONTACTS, 'contacts made in motion: %s' % '; '.join('%s / %s' % c for c in got))
    print('closest to the opening brackets: %s' % '; '.join('%s / %s %.2f mm' % (x['pair'][0], x['pair'][1], x['min'])
                                                            for x in sw['near']))
    with open(OUT, 'w', encoding='utf-8') as f:
        json.dump(r, f, indent=1, default=str)
    try:
        ns['build'](7.0)
        fc.mark()
        fc.ok(False, 'a 7 mm root refused')
    except ValueError as e:
        fc.ok('refused' in str(e), 'refused: %s' % e)
    fc.verdict()
