# CONTROL for the prompt line "each half shaft's splines from 35 mm, deep in its side gear": the half shafts are drawn
# as AxleHousing 1.2 had them, their splines starting 100 mm from the middle, and the refusal is switched off so that
# the axle is built. Everything fits and turns, and no part touches another; but each shaft holds its side gear by
# 6.75 mm of spline, and the engagement check fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/rear_axle/RearAxle/RearAxle.py'


def run(_context):
    ns = fc.load(PATH)
    ns['HSG'].Y_TIP = 100.0
    ns['refuse'] = lambda A: None
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    c = fc.interference(list(leaves.values()))
    print('  interference at rest: %s' % (c or 'none'))
    sh = leaves['Half shaft L'].bRepBodies.item(0).boundingBox
    gr = leaves['Side gear L'].bRepBodies.item(0).boundingBox
    l = gr.maxPoint.y * 10 - sh.minPoint.y * 10
    print('  the splines reach %.2f mm into the side gear' % l)
    fc.ok(l >= 35.3, 'at least 35.3 mm of spline in the side gear')
    print('CONTROL: the splines start at 100 mm; the engagement check MUST fail')
