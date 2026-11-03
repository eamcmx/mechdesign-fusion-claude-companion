# The joints of the nine assemblies (Chapter 51), measured on what Fusion builds: each assembly built by its own script
# with its defaults (the bag gripper as Chapter 50 detailed it), each component's kind read as Chapter 49 reads it,
# and then (lib/fusion_joints.py):
#   - every component's material as the model holds it, and its occurrence;
#   - every seat of a bought part on a made one: the made faces on a bearing's bore or outside diameter or a dowel's
#     or pin's cylinder, on one axis, of one diameter, touching or within 0.2 mm (the gap recorded); the made parts a
#     key's body touches;
#   - every cylindrical fit between two made parts: an outside cylinder in a bore of the same diameter, on one axis,
#     touching or drawn clear by up to 0.2 mm (the gap recorded).
# Written to results/census_<assembly>.json for lib/materials_fits.py. An assembly already counted is skipped unless
# FORCE names it. The check: every bearing has its seats on made faces, touching or with its gap recorded.
import json, os, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import detail_inventory as D
importlib.reload(D)
import standard_parts as S
importlib.reload(S)
import fusion_joints as J
importlib.reload(J)
from fusion_joints import boxes_meet, TOL
import adsk.core, adsk.fusion

OUT = '__COMPANION__/results/census_%s.json'
SCRIPTS = '__COMPANION__/scripts'
FORCE = ()            # count again
ONLY = ()             # the assemblies to count; () for all nine


def census(key):
    J.build(key, SCRIPTS)
    app = adsk.core.Application.get()
    doc = app.activeDocument
    root = adsk.fusion.Design.cast(app.activeProduct).rootComponent
    mm_ = app.measureManager
    parts = J.parts(root, key)
    made = [p for p in parts if p['status'] == 'made']
    seats, fits = J.records(parts, mm_)
    # a bearing that sits on no made face: the nearest made part, and how far
    gaps = []
    for b in [p for p in parts if p['status'] == 'bought' and p['designation'].startswith(('Bearing', 'Rod end'))
              and 'balls' not in p['name']]:
        if any(s['occ'] == b['occ'] for s in seats):
            continue
        near = [(mm_.measureMinimumDistance(b['bodies'][0], m['bodies'][0]).value * 10, m['kind']) for m in made
                if boxes_meet(b['box'], m['box'], pad=0.5)]
        gaps.append(dict(piece=b['name'], occ=b['occ'], nearest=min(near) if near else None))
    strip = lambda r: {k: v for k, v in r.items() if not k.startswith('_')}
    rec = dict(key=key, components=[dict(name=p['name'], occ=p['occ'], kind=p['kind'], status=p['status'],
                                         designation=p['designation'], group=p['group'], material=p['material'],
                                         top=p['top']) for p in parts],
               seats=[strip(s) for s in seats], fits=[strip(f) for f in fits],
               tops={o.component.name: o.isGrounded for o in root.occurrences}, links=J.links(root), gaps=gaps)
    open(OUT % key, 'w').write(json.dumps(rec, indent=1))
    try:
        doc.close(False)
    except RuntimeError:                       # Fusion has once refused the handle ('invalid name'): close it as active
        adsk.core.Application.get().activeDocument.close(False)
    return rec


def run(_context):
    for key in [k for k in D.SCRIPTS if not ONLY or k in ONLY]:
        if os.path.exists(OUT % key) and key not in FORCE:
            rec = json.load(open(OUT % key))
            print('  %s: counted before' % key)
        else:
            rec = census(key)
        seated = [x for x in rec['seats'] if x.get('gap', 0) < TOL * 10]
        gapped = [x for x in rec['seats'] if x.get('gap', 0) >= TOL * 10]
        rings = sorted({c['occ'] for c in rec['components'] if c['status'] == 'bought'
                        and c['designation'].startswith(('Bearing', 'Rod end')) and 'balls' not in c['name']})
        found = {x['occ'] for x in rec['seats'] if x['designation'].startswith(('Bearing', 'Rod end'))}
        keys = {x['occ'] for x in rec['seats'] if x['face'] == 'sides'}
        print('  %s: %d components, %d made; %d seats of bought parts (%d with a gap, %d key seats), %d made-in-made '
              'fits; bearings with a seat %d of %d' % (key, len(rec['components']),
                                                       sum(1 for c in rec['components'] if c['status'] == 'made'),
                                                       len(seated), len(gapped), len(keys), len(rec['fits']),
                                                       len(found & set(rings)), len(rings)))
        for x in gapped:
            print('      a gap: %s %s %.3f mm off %s, %.4f mm' % (x['piece'], x['face'], x['d'], x['made'], x['gap']))
        for x in [f for f in rec['fits'] if f.get('gap', 0) >= TOL * 10]:
            print('      drawn clear: %s in %s, %.3f mm, %.4f mm' % (x['shaft_name'], x['bore_name'], x['d'], x['gap']))
        lone = [n for n in rings if n not in found]
        fc.ok(not lone, '%s: every bearing has its seats on made faces, touching or with its gap recorded: %d bearings, '
              'alone %s; the nearest %s' % (key, len(rings), lone[:4], [g['nearest'] for g in rec.get('gaps', [])][:4]))
    fc.verdict()
