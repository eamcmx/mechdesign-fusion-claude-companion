# MaterialsFits (Chapter 51) against the acceptance criteria of its prompt, measured on what Fusion holds after the run:
# the bag gripper built as Chapter 50 detailed it; every made part's bodies in the library's material for its alloy;
# the joints found again on the model and decided again, and every decided face carrying its class and roughness and
# its colour, no other face carrying the attribute; the bag gripper's bearing seats, the lever's hub, the key seats
# and the dowel's hole; the made parts' mass against volume times the library's density; the refusal.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import detail_inventory as D
import materials_fits as M
import fusion_joints as J
importlib.reload(D)
importlib.reload(M)
importlib.reload(J)
import adsk.core

PATH = '__COMPANION__/design_office/MaterialsFits/MaterialsFits.py'


def attr(f):
    a = f.attributes.itemByName('mdfc', 'fit')
    return a.value if a else None


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    app = adsk.core.Application.get()
    root = fc.design().rootComponent
    mm_ = app.measureManager
    last = ns['_last']
    ps = J.parts(root, 'bag')
    made = [p for p in ps if p['status'] == 'made']
    # the materials
    bad, lib = [], {}
    was = {q['occ']: q['material'] for q in last['parts']}      # the model's materials before the script gave its own
    for p in made:
        alloy = M.material('bag', p['kind'], was[p['occ']])[0]
        want = M.FUSION.get(alloy)
        got = {b.material.name for b in p['bodies']}
        lib[p['kind']] = (alloy, want)
        if want is None or got != {want}:
            bad.append((p['name'], alloy, sorted(got)))
    fc.ok(not bad and not last['missing'],
          'every made part\'s bodies in the library\'s material for its alloy: %d parts, wrong %s; the library lacks %s'
          % (len(made), bad[:3], sorted(last['missing']) or 'none'))
    print('  ' + '; '.join(sorted({'%s: %s' % v for v in lib.values()})))
    # the joints found and decided again, and the faces that carry them
    seats, fits = J.records(ps, mm_)
    rec = dict(key='bag', components=[dict(name=p['name'], occ=p['occ'], kind=p['kind'], status=p['status'],
                                           designation=p['designation'], group=p['group'], material=p['material'],
                                           top=p['top']) for p in ps],
               seats=seats, fits=fits, tops={o.component.name: o.isGrounded for o in root.occurrences},
               links=J.links(root), gaps=[])
    dec = M.decide('bag', rec)
    want = {}
    for x in seats:
        if 'cls' in x:
            for f in ([x['_face']] if x['_face'] is not None else x.get('_faces', [])):
                want.setdefault(f.entityToken, set()).add(ns['finish'](x['cls'], x))
    for x in fits:
        if 'cls' in x:
            h, sh = x['cls'].split('/')
            for f, cls, ra, rz in zip(x['_faces'], (sh, h), (x['ra'], x['ra_hole']), (x.get('rz'), None)):
                want.setdefault(f.entityToken, set()).add(ns['finish'](cls, dict(ra=ra, rz=rz)))
    faces = {f.entityToken: f for p in made for b in p['bodies'] for f in b.faces}
    carried = {t: attr(f) for t, f in faces.items() if attr(f)}
    wrong = [(t, carried.get(t), sorted(w)) for t, w in want.items() if carried.get(t) != ' + '.join(sorted(w))]
    extra = [t for t in carried if t not in want]
    fc.ok(not wrong and not extra and want,
          'every decided face carries its class and roughness: %d faces, wrong %s, carried and not decided %d'
          % (len(want), wrong[:3], len(extra)))
    red = [f for f in faces.values() if attr(f) and f.appearance and f.appearance.name == ns['COLOURS']['bearing seat']]
    fc.ok(len(red) == len({x['_face'].entityToken for x in seats if x.get('joint') == 'bearing seat'}),
          'the bearing seats coloured %s: %d faces' % (ns['COLOURS']['bearing seat'], len(red)))
    # the bag gripper's joints, by name
    brg = [x for x in seats if x['designation'].startswith('Bearing 6206')]
    sh = sorted({(x['made'], x['cls']) for x in brg if x['face'] == 'bore'})
    hs = sorted({(x['made'], x['cls']) for x in brg if x['face'] == 'outside'})
    fc.ok(len(brg) == 8 and sh == [('Stub shaft long', 'k5'), ('Stub shaft short', 'k5')],
          'the four 6206-2RS1 on the stub shafts k5: %s' % sh)
    fc.ok(hs == [('End plate', 'J7')], 'the four 6206-2RS1 in the aluminium end plates J7: %s' % hs)
    hub = sorted({(x['shaft'], x['bore'], x['cls']) for x in fits if x['bore'].startswith('Lever') and 'cls' in x})
    fc.ok(any(s.startswith('Stub shaft') and c == 'H7/k5' for s, b, c in hub),
          'the lever\'s hub H7 on the bearing\'s k5 seat: %s' % hub)
    keys = sorted({(x['made'], x['cls']) for x in seats if x['face'] == 'sides'})
    kf = [f for x in seats if x['face'] == 'sides' for f in x['_faces']]
    fc.ok(kf and all(f.appearance and f.appearance.name == ns['COLOURS']['key seat'] for f in kf),
          "the key seats' faces coloured %s: %d faces" % (ns['COLOURS']['key seat'], len(kf)))
    fc.ok(keys and all((c == 'N9') == m.startswith('Stub shaft') for m, c in keys) and {c for m, c in keys} == {'N9', 'JS9'},
          'the key seats N9 in the shafts and JS9 in the hubs: %s' % keys)
    dw = sorted({(x['made'], x['cls']) for x in seats if x['designation'].startswith('Dowel')})
    fc.ok(dw == [('Adapter plate', 'H7')], 'the dowel\'s hole H7: %s' % dw)
    # the mass against volume times the library's density
    calc = sum(b.physicalProperties.volume * 1e-6 * M.FUSION_DENSITY[b.material.name] for p in made for b in p['bodies'])
    fc.ok(abs(calc - last['after']) < 1e-6 * calc,
          'the made parts weigh %.4f kg, volume times the library\'s density %.4f kg (%.4f kg with the generic '
          'materials)' % (last['after'], calc, last['before']))
    fc.refused(ns, 'nine', key='gearbox')
    fc.verdict()
