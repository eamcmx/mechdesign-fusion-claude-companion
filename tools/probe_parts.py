# Chapter 57: one machine built by RangeSweep.py, every part that has bodies with Fusion's mass, its materials and its
# volume, the heaviest first, for setting the drawn bought parts against their makers' masses. The machine is
# __MACHINE__, replaced before the run; the design is closed unsaved at the end.
#     sed s/__MACHINE__/bag/ tools/probe_parts.py > <scratch>.py; python tools/fusion_mcp.py script <scratch>.py --readonly
import importlib.util, os, sys
import adsk.core, adsk.fusion

COMP = r'C:\Users\eamer\Documents\MechDesignFusionClaude\companion'
sys.path.insert(0, os.path.join(COMP, 'common'))
import companion_paths as CP                 # every lib/ folder on sys.path
MACHINE = '__MACHINE__'


def run(_context):
    spec = importlib.util.spec_from_file_location('probe_rs', CP.script('RangeSweep'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    msg, setter = m.machine(MACHINE)
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    rows = []
    for o in des.rootComponent.allOccurrences:
        if not o.component.bRepBodies.count:
            continue
        pp = o.physicalProperties
        mats = sorted(set(b.material.name for b in o.component.bRepBodies))
        rows.append((pp.mass, o.component.name, ', '.join(mats), sum(b.volume for b in o.component.bRepBodies)))
    rows.sort(reverse=True)
    print('%s: total %.4f kg, %d parts' % (MACHINE, sum(r[0] for r in rows), len(rows)))
    for mass, name, mat, vol in rows:
        print('%8.4f kg  %-50s %-28s %9.1f cm3' % (mass, name[:50], mat[:28], vol))
    adsk.core.Application.get().activeDocument.close(False)
