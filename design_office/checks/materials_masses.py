# The nine assemblies weighed with the library's materials (Chapter 51): MaterialsFits.py run on each, its message
# kept; for each, the made parts' mass with the model's generic materials and with the library's, the whole
# assembly's mass after, the materials the library lacks and the faces given a class, by kind. Written to
# results/materials_masses.json for the book. The check: every made part given a material but the ones the library
# lacks, and every assembly's faces coloured.
import json, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import detail_inventory as D
import materials_fits as M
importlib.reload(D)
importlib.reload(M)
import adsk.core

PATH = '__COMPANION__/design_office/MaterialsFits/MaterialsFits.py'
OUT = '__COMPANION__/design_office/results/materials_masses.json'
ONLY = ()             # the assemblies to weigh again, the others kept from the last run; () for all nine


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    import os
    out = json.load(open(OUT)) if ONLY and os.path.exists(OUT) else {}
    for key in [k for k in D.SCRIPTS if not ONLY or k in ONLY]:
        msg = ns['build'](key)
        fc.mark()
        last = ns['_last']
        app = adsk.core.Application.get()
        total = sum(b.physicalProperties.mass for o in app.activeProduct.rootComponent.allOccurrences for b in o.bRepBodies)
        out[key] = dict(made=len(last['made']), given=last['given'], missing=sorted(last['missing']),
                        before=last['before'], after=last['after'], total=total, coloured=last['coloured'])
        print('  ' + msg.split('\n')[0])
        lacking = [p for p in last['made'] if M.FUSION.get(M.material(key, p['kind'], p['material'])[0]) is None]
        fc.ok(last['given'] == len(last['made']) - len(lacking) and sum(last['coloured'].values()) > 0,
              '%s: %d of %d made parts given a library material, %d lacking one (%s); %d faces given a class'
              % (key, last['given'], len(last['made']), len(lacking), sorted(last['missing']) or 'none',
                 sum(last['coloured'].values())))
        app.activeDocument.close(False)
    open(OUT, 'w').write(json.dumps(out, indent=1))
    fc.verdict()
