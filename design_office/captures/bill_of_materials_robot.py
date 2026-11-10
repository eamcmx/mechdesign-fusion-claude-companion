# Pictures for Chapter 64: the palletizer's robot as Part 9 detailed it, built by RangeSweep.py, its bill made by
# BillOfMaterials.py's census and lib/bom_detail.py, and coloured by what the bill does to its masses: the made parts
# grey, a bought part red where Fusion draws it more than 10 % heavier than its maker makes it, blue where more than
# 10 % lighter, yellow where the two agree within 10 % or no saved page gives the maker's mass. Chapter 49's camera.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import bom_detail as BD
importlib.reload(BD)
import adsk.core, adsk.fusion

EYE = (1.0, -1.35, 0.55)
TARGET = (60.0, 0.0, 140.0)          # cm
GREY, YELLOW = 'Paint - Enamel Glossy (Grey)', 'Paint - Enamel Glossy (Yellow)'
RED, BLUE = 'Paint - Enamel Glossy (Red)', 'Paint - Enamel Glossy (Blue)'


def shade(g):
    if g['kind'] != 'bought':
        return GREY
    if not g.get('maker'):
        return YELLOW
    r = (g['drawn'] / g['qty']) / g['maker'][0]
    return RED if r > 1.10 else BLUE if r < 0.90 else YELLOW


def run(_context):
    import mdfc_parts as mp
    rs = fc.load('__COMPANION__/design_office/RangeSweep/RangeSweep.py')
    msg, _ = rs['machine']('robot')
    print(msg.splitlines()[0])
    fc.mark()
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    bm = fc.load('__COMPANION__/design_office/BillOfMaterials/BillOfMaterials.py')
    rows, occ = bm['census'](des)
    b = BD.finish(BD.bom('robot', rows))
    for o in root.occurrences:
        o.appearance = None
    count = {}
    for g in b['lines']:
        col = shade(g)
        names = [n for v in g['names'].values() for n, p in v] + [w['name'] for w in g.get('with', [])]
        for n in names:
            for o in occ.get(n, []):
                mp.colour(des, o, col)
        if g['kind'] == 'bought' and col != YELLOW:
            count.setdefault(col.split('(')[1][:-1], []).append('%s x%d' % (g['number'], g['qty']))
    print('coloured: %s' % count)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 900.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
