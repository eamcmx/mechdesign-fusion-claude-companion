# Chapter 57: the wrist built by RangeSweep.py, its heaviest parts with their materials and volumes, for the
# difference between the detailed wrist and Chapter 52's estimate. Run with tools/fusion_mcp.py script <this> --readonly.
import importlib.util, os, sys
import adsk.core, adsk.fusion

COMP = r'C:\Users\eamer\Documents\MechDesignFusionClaude\companion'
sys.path.insert(0, os.path.join(COMP, 'lib'))


def run(_context):
    spec = importlib.util.spec_from_file_location('probe_rs', os.path.join(COMP, 'scripts', 'RangeSweep', 'RangeSweep.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    msg, setter = m.machine('wrist')
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    rows = []
    for o in des.rootComponent.allOccurrences:
        if not o.component.bRepBodies.count:
            continue
        pp = o.physicalProperties
        mats = sorted(set(b.material.name for b in o.component.bRepBodies))
        rows.append((pp.mass, o.component.name, ', '.join(mats), sum(b.volume for b in o.component.bRepBodies)))
    rows.sort(reverse=True)
    print('total %.4f kg, %d parts' % (sum(r[0] for r in rows), len(rows)))
    for mass, name, mat, vol in rows[:25]:
        print('%8.4f kg  %-50s %-28s %9.1f cm3' % (mass, name[:50], mat[:28], vol))
    adsk.core.Application.get().activeDocument.close(False)
