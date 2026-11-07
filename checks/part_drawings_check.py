# PartDrawings (Chapter 63) against the acceptance criteria of its prompt: run by the dialog, as a user runs it, all
# the parts, into a folder of the check's own; each part's design read back (the bracket's root radius in the model),
# each sheet read back from Fusion (ISO A3, its views, the dimensions and symbols Part 10 decided, its table), each PDF
# on disk; nothing saved over another file; the refusal.
import sys, importlib, os, time
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import drawings_detail as DD
importlib.reload(DD)
import lugs_brackets as LB
importlib.reload(LB)

PATH = '__COMPANION__/scripts/PartDrawings/PartDrawings.py'
WHERE = ('MechDesignFusionClaude Vol3', 'Chapter 63 check %s' % time.strftime('%Y-%m-%d %H%M'))
EDITS = ()                                  # a control replaces this with its one broken line


def run(_context):
    import adsk.core
    app = adsk.core.Application.get()
    before = [app.documents.item(i) for i in range(app.documents.count)]
    ns = fc.load(PATH, EDITS)
    print(ns['VERSION'])
    ns['DD'].FOLDER = WHERE                 # the dialog's 'all', into the check's own folder
    fc.run_dialog(ns)
    for i in range(app.documents.count):    # every document the run opened, closed after
        d = app.documents.item(i)
        if not any(d == b for b in before):
            d.attributes.add('mdfc', 'run', 'check')
    r = ns['_last']
    parts = {p['key']: p for p in r['parts']}
    fc.ok(sorted(parts) == sorted(p['key'] for p in DD.PARTS), 'six drawings: %s' % ', '.join(sorted(parts)))
    for key, p in parts.items():
        sh = p['sheet']
        # a part: one sheet of at least two views; the assembly: its first sheet the view with Fusion's parts list,
        # its second the projections
        views, sheets = (1, 2) if key == 'stop' else (2, 1)
        fc.ok(abs(sh['size'][0] - 420) < 0.5 and abs(sh['size'][1] - 297) < 0.5 and len(sh['views']) >= views
              and p['sheets'] == sheets,
              '%s %s: A3 %g x %g, %d sheet(s), %d views on the first' % (p['number'], p['title'], sh['size'][0],
                                                                       sh['size'][1], p['sheets'], len(sh['views'])))
        fc.ok(p['pdf'] is not None and os.path.exists(os.path.join('__COMPANION__', 'results', 'drawings', p['pdf'])),
              '%s: the PDF %s' % (p['number'], p['pdf']))
        want = ns['WANT'][key]
        sym = want.get('datums', 0) + want.get('frames', 0)
        got = p['read']                                  # counted on the sheet after the script, not as it claims
        fc.ok(got['dims'] == want.get('dims', 0) and got['symbols'] == sym,
              '%s: on the sheet %d dimensions and %d symbols more (asked %d and %d)' % (
                  p['number'], got['dims'], got['symbols'], want.get('dims', 0), sym))
    b, bd = parts['bracket'], DD.bracket()
    rr = {float(k): v for k, v in b['model']['radii'].items()}
    n_root, n_hole = rr.get(bd['r_root'], 0), rr.get(bd['H'] / 2, 0)
    fc.ok(n_root == 4, "the bracket's root in the model: %d faces of radius %g" % (n_root, bd['r_root']))
    fc.ok(n_hole == 2, "the bracket's holes in the model: %d of %g across" % (n_hole, bd['H']))
    hv = LB.volume(True, (b['changes'].get('root_fillet') or {}).get('r'))
    fc.ok('turn_arm' in b['changes'] and abs(b['volume'] - hv) <= 1e-4 * hv,
          "the bracket's model: its arm turned %s deg (Ch 62), %.1f mm3 against %.1f by hand" % (
              (b['changes'].get('turn_arm') or {}).get('turn'), b['volume'], hv))
    st = parts['stop']
    sr = st['changes'].get('root_fillet') or {}
    fc.ok(sr.get('edges') == 4 and sr.get('r') == bd['r_root'] and 'turn_arm' in st['changes'],
          "the opening stop's bracket as its sheet draws it: the arm turned, the root filleted on %s edges, r %s" % (sr.get('edges'), sr.get('r')))
    fc.ok(sorted((i['number'], i['n']) for i in st['items']) == sorted((n, q) for n, _t, _d, _nm, q in DD.STOP_ITEMS)
          and any(r[0] == 'S-0121' and 'N m' in r[1] for r in st['rows']),
          'the opening stop: %s; %s' % (', '.join('%s x%d' % (i['number'], i['n']) for i in st['items']),
                                         [r[1] for r in st['rows'] if r[0] == 'S-0121']))
    for key, p in parts.items():
        if key == 'stop':
            continue
        fc.ok(len(p['rows']) >= 3 and p['rows'][0][0] == 'Material' and p['rows'][-1][1] == 'ISO 2768-mK',
              '%s: its table, %d rows: %s' % (p['number'], len(p['rows']), '; '.join('%s %s' % (a, b_) for a, b_, _ in p['rows'][1:-1])))
    names = r['names']
    fc.ok(len(names) == len(set(names)), 'every file in %s once: %d files' % (r['folder'], len(names)))
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown part refused')
    except ValueError as e:
        fc.ok('No part called' in str(e), 'refused: %s' % e)
    fc.verdict()
