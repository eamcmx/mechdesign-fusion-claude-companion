# Making (Chapter 66) against the acceptance criteria of its prompt: run by the dialog, as a user runs it, for its
# default part, the bracket, then for the spacer and the tine; each part the machine's own (the bracket's volume against
# lib/lugs_brackets.py, the spacer's length against lib/drawings_detail.py, the tine's against lib/making.py); every
# operation with a toolpath; the bracket's root cut by a cutter whose corner radius is the drawing's 2.5; each setup
# timed and posted, its NC file on disk with the tools it lists; the spacer turned long by the grinding allowance; the
# tine's corner rounded, the sixteen nested on one plate and cut, one pierce each; the refusal.
import sys, importlib, os, re
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import lugs_brackets as LB
importlib.reload(LB)
import drawings_detail as DD
importlib.reload(DD)
import making as MK
importlib.reload(MK)

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
            tools = tools or re.findall(r'^\(((?:Laser|Plasma|Waterjet) cutting)\)', txt, re.M)   # a cutting program names its head
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
    # the tine: the bag gripper's own, its inside corner rounded, against lib/making.py's hand figures
    t, ti = done['tine'], done['tine']['info']
    h0, h = MK.tine(0.0), MK.tine()
    fc.ok(abs(ti['sharp']['volume'] - h0['V']) < 1e-4 * h0['V'] and abs(ti['volume'] - h['V']) < 1e-4 * h['V']
          and abs(ti['thickness'] - h['t']) < 1e-6 and ti['material'] == 'Steel AISI 1020 107 HR',
          "the tine from the gripper: %.1f mm3 sharp (by hand %.1f), %.1f with the corner R%g (by hand %.1f), %g thick, %s"
          % (ti['sharp']['volume'], h0['V'], ti['volume'], h['r'], h['V'], ti['thickness'], ti['material']))
    fc.ok(ti['corner_edges'] == 1 and len(ti['rounded']) == 1 and abs(ti['rounded'][0] - h['r']) < 1e-6
          and abs(ti['outline'] - h['P']) < 1e-4 and h['clear'] > 0,
          "its inside corner: one edge found, rounded R%.1f; outline %.3f mm (by hand %.3f); the round starts %.0f from the "
          "jaw's middle, %.0f clear of the bag's side" % (ti['rounded'][0] if ti['rounded'] else 0, ti['outline'] or 0, h['P'], h['round_from'], h['clear']))
    nt = MK.nest(ti['nest'])
    fc.ok(ti['arranged'] == MK.TINE['n'] and ti['unarranged'] == 0 and abs(ti['efficiency'] - nt['eff_plate']) < 1e-4
          and ti['nest'][0] + 2 * MK.TINE['frame'] <= MK.TINE['plate'][0] and ti['nest'][1] + 2 * MK.TINE['frame'] <= MK.TINE['plate'][1]
          and min(ti['nest_lo']) >= MK.TINE['frame'] - 1e-3,
          'nested: %d of %d on the %g x %g plate, in %.1f x %.1f (turned %s deg); %.1f %% of that rectangle is tine, %.1f %% of '
          'the plate (Fusion %.1f %%)' % (ti['arranged'], MK.TINE['n'], MK.TINE['plate'][0], MK.TINE['plate'][1], ti['nest'][0],
                                         ti['nest'][1], ', '.join(str(a) for a in ti['turns']), 100 * nt['eff_used'],
                                         100 * nt['eff_plate'], 100 * ti['efficiency']))
    st = t['setups'][0]
    txt = open(os.path.join(nc, st['nc']), encoding='latin-1').read() if st['nc'] else ''
    ct = MK.cut_time(ti['feed'], ti['pierce'])
    pierces = len(re.findall(r'PIERCE', txt))
    feeds = sorted(set(re.findall(r'F(\d+\.?\d*)', txt)))
    fc.ok(pierces == MK.TINE['n'] and feeds == ['%g' % ti['feed']] and ct['L'] <= st['feed'] <= 1.02 * ct['L']
          and ct['cutting'] <= t['total'] <= 1.05 * ct['cutting'],
          'cut: %d pierces, F%s, %.0f mm cut (the outlines %.0f by hand, the rest lead-ins), %.0f s in Fusion against %.0f s for '
          "the outlines alone at the sample laser's %g mm/min; the pierces' %g s each come on top" % (
              pierces, '/'.join(feeds), st['feed'], ct['L'], t['total'], ct['cutting'], ti['feed'], ti['pierce']))
    # the census: every made line of the bill measured in Fusion, then sorted by lib/making.py
    ns['build']('census')
    fc.mark()
    import json, io
    bom = json.load(io.open(os.path.join('__COMPANION__', 'results', 'bom.json'), encoding='utf-8'))
    want = sorted((m['key'], l['number']) for m in bom['machines'] for l in m['lines'] if l['kind'] == 'made')
    cs = MK.census()
    got = sorted((r['machine'], r['number']) for r in cs['rows'])
    fc.ok(got == want and all(r['bodies'] >= 1 for r in cs['rows']) and cs['pieces'] == sum(l['qty'] for m in bom['machines']
                                                                                          for l in m['lines'] if l['kind'] == 'made'),
          'the census: %d made kinds measured, as the bill has them, %d pieces, %.1f kg' % (cs['kinds'], cs['pieces'], cs['kg']))
    fc.ok(sum(g['kinds'] for g in cs['by']['route'].values()) == cs['kinds'] == sum(g['kinds'] for g in cs['by']['shape'].values()),
          'by route: %s; by shape: %s' % ('; '.join('%s %d' % (k, g['kinds']) for k, g in sorted(cs['by']['route'].items())),
                                           '; '.join('%s %d' % (k, g['kinds']) for k, g in sorted(cs['by']['shape'].items()))))
    row = {r['number']: r for r in cs['rows']}
    known = [('A01-016', 'machined', 'turned'), ('P13-012', 'welded', 'plate profile'), ('P13-018', 'machined', '3D'),
             ('P13-016', 'welded', 'weldment'), ('A01-004', 'cast', '3D'), ('A01-007', 'gear', '3D')]
    fc.ok(all((row[n]['route'], row[n]['shape']) == (rt, sh) for n, rt, sh in known) and row['P13-016']['bodies'] == 4
          and abs(row['P13-012']['shapes'][0][2]['t'] - 10.0) < 1e-3,
          'known answers: the spacer turned, the tine a plate profile 10 thick, the turned bracket 3D, the jaw a weldment '
          'measured on %d (its tube, two stubs and welds), the carrier cast, the crown wheel a gear' % row['P13-016']['bodies'])
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown part refused')
    except ValueError as e:
        fc.ok('No part called' in str(e), 'refused: %s' % e)
    fc.verdict()
