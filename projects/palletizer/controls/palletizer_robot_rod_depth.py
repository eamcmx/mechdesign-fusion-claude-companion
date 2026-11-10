# CONTROL for the prompt line "The parallel rod a flat bar 40 mm deep in the arm's plane: at full reach it passes
# 190 mm from J3's axis, beside the reducer's case of 162.5 mm": the rod is drawn 60 mm deep. Expected: at full reach
# and at the range's two corners with psi = 25 deg the rod cuts into J3's reducer; the interference check and the
# clearance check fail.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import palletizer_design as pd
import adsk.core

PATH = '__COMPANION__/projects/palletizer/PalletizerRobot/PalletizerRobot.py'
OLD = "HOME['C'], HOME['D'], S['rod_p'], YR, '22213 E', 75.0, flat=True)"
NEW = "HOME['C'], HOME['D'], pl.flat(0.060, 0.070), YR, '22213 E', 75.0, flat=True)"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    G = ns['_last']['groups']
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    ws = pd.workspace()
    ns['set_pose'](G, 0.0, ws['reach_q2'], ws['reach_q3'])
    c = fc.interference(leaves)
    fc.ok(not c, 'full reach: no interference %s' % (c or ''))
    rod = [o for o in G['Parallel rod'].childOccurrences if o.component.name == 'Parallel rod body'][0].bRepBodies.item(0)
    rv3 = [o for o in G['Turret'].childOccurrences if o.component.name == 'RV-500N case J3'][0].bRepBodies.item(0)
    d = adsk.core.Application.get().measureManager.measureMinimumDistance(rod, rv3).value * 10.0
    fc.ok(d >= 5.0, 'full reach: the parallel rod clear of J3\'s reducer by at least 5 mm: %.1f mm' % d)
    print('CONTROL: the parallel rod 60 mm deep; the checks above MUST fail')
