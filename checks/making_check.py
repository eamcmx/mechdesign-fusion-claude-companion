# Making (Chapter 66) against the acceptance criteria of its prompt: run by the dialog, as a user runs it, for its
# default part, the bracket, then for the spacer; each part the machine's own (the bracket's volume against
# lib/lugs_brackets.py, the spacer's length against lib/drawings_detail.py); every operation with a toolpath; the
# bracket's root cut by a cutter whose corner radius is the drawing's 2.5; each setup timed and posted, its NC file on
# disk with the tools it lists; the spacer turned long by the grinding allowance; the refusal.
import sys, importlib, os, re
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import lugs_brackets as LB
importlib.reload(LB)
import drawings_detail as DD
importlib.reload(DD)

PATH = '__COMPANION__/scripts/Making/Making.py'
EDITS = ()                                  # a control replaces this with its one broken line


def run(_context):
    ns = fc.load(PATH, EDITS)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    done = {}
    r = dict(ns['_last'])
    done[r['part']] = r
    for p in ns['PARTS']:
        if p in done:
            continue
        ns['build'](p)
        fc.mark()
        done[p] = dict(ns['_last'])
    fc.ok(sorted(done) == sorted(ns['PARTS']), 'the parts made: %s' % ', '.join(sorted(done)))
    b, s = done['bracket'], done['spacer']
    hv = LB.volume(True, 2.5)
    fc.ok(abs(b['info']['volume'] - hv) <= 1e-4 * hv and (b['info']['holes'], b['info']['bore'], b['info']['root_edges']) == (2, 1, 4),
          'the bracket: %.1f mm3 against %.1f by hand (the arm turned); 2 holes, the bore, 4 root edges found' % (b['info']['volume'], hv))
    sp = DD.spacer()
    fc.ok(abs(s['info']['length'] - sp['L']) < 1e-6, 'the spacer: %.2f mm long as drawn' % s['info']['length'])
    nc = os.path.join('__COMPANION__', 'results', 'nc')
    for part, r in sorted(done.items()):
        for st in r['setups']:
            # a warning passes only where Fusion moved its own lead-in or lead-out off the stock left: printed, not failed
            safe = lambda o: (not o['warning']) or ('has been modified due to a gouge with the remaining stock' in o.get('message', ''))
            for o in st['ops']:
                if o['warning']:
                    print('  note %s / %s: %s' % (part, o['name'], o.get('message', '').strip()))
            fc.ok(all(o['toolpath'] and safe(o) for o in st['ops']),
                  '%s / %s: %s' % (part, st['setup'], ', '.join('%s (%s)%s' % (o['name'], o['tool'], ' WARNING' if o['warning'] else '' if o['toolpath'] else ' NO TOOLPATH') for o in st['ops'])))
            path = os.path.join(nc, st['nc'] or '-')
            txt = open(path, encoding='latin-1').read() if st['nc'] and os.path.exists(path) else ''
            tools = re.findall(r'^\((T\d+ D=[^)]*)\)', txt, re.M) or sorted(set(re.findall(r'(T\d{4})', txt)))
            fc.ok(bool(txt) and st['nc_lines'] > 10 and st['seconds'] > 0 and len(tools) >= 1,
                  '%s / %s: %.0f s, %s, %d lines, tools %s' % (part, st['setup'], st['seconds'], st['nc'], st['nc_lines'], '; '.join(tools)))
    root = [o for st in b['setups'] for o in st['ops'] if o['strategy'] == 'contour2d']
    txt = open(os.path.join(nc, b['setups'][1]['nc']), encoding='latin-1').read()
    fc.ok(len(root) == 1 and 'CR=2.5' in txt, "the arm's root: a 2D contour with %s, CR=2.5 in the program" % (root[0]['tool'] if root else None))
    face = [o for o in s['setups'][0]['ops'] if o['strategy'] == 'turning_face']
    i = s['info']
    fc.ok(len(face) == 1 and abs(i['front'] - i['leave']) < 1e-6 and abs(i['back'] + i['leave']) < 1e-6
          and abs(sp['L'] + 2 * i['leave'] - sp['stock']) < 1e-6,
          'the spacer faced %.2f long and parted %.2f long: turned to %.1f for grinding to %.2f-%.2f' % (
              i['front'], -i['back'], sp['stock'], sp['range'][0], sp['range'][1]))
    fc.ok(abs(b['total'] - sum(st['seconds'] for st in b['setups'])) < 1e-6,
          'the bracket: %.1f min of cutting in %d setups; the spacer %.1f min' % (b['total'] / 60, len(b['setups']), s['total'] / 60))
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown part refused')
    except ValueError as e:
        fc.ok('No part called' in str(e), 'refused: %s' % e)
    fc.verdict()
