# StudyParts (Chapters 58, 59 and 62) against the acceptance criteria of its prompt: run by the dialog, as a user runs
# it, for its default part, then built for the other five; each part's volume against the hand volume in
# lib/fea_studies.py, its material, its faces, the faces its study picks found on the body; the refusal.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import fea_studies as FS
importlib.reload(FS)

PATH = '__COMPANION__/design_office/StudyParts/StudyParts.py'
EDITS = ()                                  # a control replaces this with its one broken line
MATERIAL = dict(cantilever='Steel', keyseat='Steel', tine='Steel', bracket_sharp='Steel', bracket_r25='Steel',
                lug='Aluminum 7075')
FACES = dict(cantilever=6, keyseat=12, tine=8, bracket_sharp=11, bracket_r25=15, lug=7)


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
    fc.ok(sorted(done) == sorted(FS.VOLUMES), 'the six parts built: %s' % ', '.join(sorted(done)))
    for p, r in done.items():
        v, h = r['volume'], FS.VOLUMES[p]
        fc.ok(abs(v - h) <= 1e-4 * h, '%s: %.1f mm3 in the model, %.1f by hand (%.4f %%)' % (p, v, h, 100 * (v - h) / h))
        fc.ok(r['material'] == MATERIAL[p] and r['faces'] == FACES[p],
              '%s: %s, %d faces' % (p, r['material'], r['faces']))
        fc.ok(all(r['found'].values()), '%s: faces to pick %s' % (p, ', '.join(
            '%s %s' % (k, 'found' if f else 'MISSING') for k, f in r['found'].items())))
    t = done['tine']
    fc.ok(abs(t['mass'] - FS.RHO_ST * FS.VOLUMES['tine']) < 1e-4, 'the tine weighs %.4f kg' % t['mass'])
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown part refused')
    except ValueError as e:
        fc.ok('No part' in str(e), 'refused: %s' % e)
    fc.verdict()
