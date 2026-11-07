# StudyAssemblies (Chapters 59 and 62) against the acceptance criteria of its prompt: run by the dialog, as a user runs
# it, for its default assembly, then built for the other four; each assembly's parts present with the machine's
# materials, each part's volume the machine's own (the opening bracket's against lib/lugs_brackets.py, turned or on its
# diagonal; the spar web's patch against its hand volume), every pick found on a face; the loads; the arms at the
# cycle's high point and their give by hand; the refusal.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import lugs_brackets as LB
importlib.reload(LB)

PATH = '__COMPANION__/scripts/StudyAssemblies/StudyAssemblies.py'
EDITS = ()                                  # a control replaces this with its one broken line
PARTS = dict(stop=4, stop_diagonal=4, fitting=6, hinge=6, arms=27)
MATERIALS = {'Stop bracket open 1': 'Steel AISI 1045 225 ANLD', 'End plate 1': 'Aluminum 6061', 'Spar fitting': 'Aluminum 7075',
             'Rear spar web': 'Aluminum 6061', 'Trunnion R': 'Steel AISI 4142 380 QT', 'Hinge bracket R': 'Aluminum 7075',
             'Hinge rib R': 'Aluminum 7075', 'Hinge pin R': 'Steel AISI 4142 380 QT',
             'Lower arm box': 'Steel AISI 1020 107 HR', 'Upper arm box': 'Steel AISI 1020 107 HR',
             'Fork plate 1': 'Steel AISI 1045 225 ANLD', 'Tool flange': 'Steel AISI 1045 225 ANLD'}


def run(_context):
    ns = fc.load(PATH, EDITS)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    done = {}
    r = dict(ns['_last'])
    done[r['assembly']] = r
    import adsk.core
    app = adsk.core.Application.get()
    for a in ns['ASSEMBLIES']:
        if a in done:
            continue
        ns['build'](a)
        fc.mark()
        done[a] = dict(ns['_last'])
    fc.ok(sorted(done) == sorted(PARTS), 'the five assemblies built: %s' % ', '.join(sorted(done)))
    for a, r in sorted(done.items()):
        names = [p['part'] for p in r['parts']]
        fc.ok(len(names) == PARTS[a] and all(p['material'] == MATERIALS[p['part']] for p in r['parts'] if p['part'] in MATERIALS),
              '%s: %d parts, %s' % (a, len(names), '; '.join('%s %s' % (p['part'], p['material']) for p in r['parts'])))
        same = [p for p in r['parts'] if p['part'] != 'Rear spar web']
        fc.ok(all(abs(p['volume'] - p['machine_volume']) <= 1e-6 * p['machine_volume'] for p in same),
              "%s: every part the machine's own volume" % a)
        fc.ok(all(r['found'].values()), '%s: %d picks found on a face' % (a, len(r['found'])))
    for a, turned in (('stop', True), ('stop_diagonal', False)):
        b = [p for p in done[a]['parts'] if p['part'] == 'Stop bracket open 1'][0]
        h = LB.volume(turned, 2.5)
        fc.ok(abs(b['volume'] - h) <= 1e-4 * h, '%s: the bracket %.1f mm3, by hand %.1f, its arm %s' % (
            a, b['volume'], h, 'turned' if turned else 'on its diagonal'))
    w = [p for p in done['fitting']['parts'] if p['part'] == 'Rear spar web'][0]
    h = LB.web_patch()
    fc.ok(abs(w['volume'] - h) <= 1e-4 * h, 'fitting: the spar web patch %.1f mm3, by hand %.1f' % (w['volume'], h))
    st, fi, hi = done['stop']['loads'], done['fitting']['loads'], done['hinge']['loads']
    fc.ok(abs(st['F'] - 2179.963) < 1e-3 and abs((st['design'][1] ** 2 + st['design'][2] ** 2) ** 0.5 - st['F']) < 1e-6
          and st['F_max'] == 4000.0
          and abs(fi['F'] - 20848.42) < 0.01 and abs(hi['F'] - 13955.97) < 0.01,
          'the loads: the stop %.1f N along the absorber and the DYSC%ss %.0f, the fitting %.1f N, the hinge %.1f N' % (st['F'], "'", st['F_max'], fi['F'], hi['F']))
    # the arms: the robot at the cycle's high point, so the tool flange (250 across, 30 thick) where the high point's
    # flange is; the hand's give at the flange's outer face from palletizer_links.lateral()
    import palletizer_design as PD
    import palletizer_links as PL
    importlib.reload(PD)
    importlib.reload(PL)
    q = PD.waypoints(PD.brief())['hi']
    fx, fz = PD.pts(q['q2'], q['q3'])['fl']
    tf = [p for p in done['arms']['parts'] if p['part'] == 'Tool flange'][0]
    (x0, y0, z0), (x1, y1, z1) = tf['box']
    fc.ok(max(abs(x0 - (fx - 125.0)), abs(x1 - (fx + 125.0)), abs(y0 + 125.0), abs(y1 - 125.0), abs(z0 - (fz - 30.0)),
              abs(z1 - fz)) < 1e-3,
          'arms: at q2 %.2f, q3 %.2f deg the tool flange spans x %.1f to %.1f and z %.1f to %.1f, the flange at (%.1f, %.1f)'
          % (q['q2'], q['q3'], x0, x1, z0, z1, fx, fz))
    ar = done['arms']['loads']
    h = PL.lateral(at=(fx / 1000.0, (fz - 30.0) / 1000.0))
    fc.ok(ar['F'] == 1000.0 and ar['lateral'] == (0.0, 1000.0, 0.0) and abs(ar['c_hand'] - h['c_flange']) < 1e-15
          and abs(ar['give_hand'] - 1e6 * h['c_flange']) < 1e-9,
          'arms: %.0f N across the arms\' plane at the flange\'s outer face, which the hand model gives %.4f mm (%.4f at the '
          'flange\'s own point)' % (ar['F'], ar['give_hand'], ar['c_flange'] * 1e6))
    try:
        ns['build']('crane')
        fc.mark()
        fc.ok(False, 'an unknown assembly refused')
    except ValueError as e:
        fc.ok('No assembly called' in str(e), 'refused: %s' % e)
    fc.verdict()
