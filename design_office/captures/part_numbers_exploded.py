# Pictures for Chapter 49: the bag gripper numbered by PartNumbers.py, taken apart as Chapter 47 takes it
# (projects/palletizer/captures/bag_gripper_exploded.py: its moves, its camera, its anchors), made parts blue and bought parts yellow,
# and one balloon for each kind of part in the order of Table 49.3 (lib/detail_inventory.py, numbering('bag')): item k
# is the k-th row after the subassembly's own. A body that comes with a bought part (the piston rods) gets no balloon.
import sys, importlib, json
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

COMP = '__COMPANION__'
TOKEN = '__' + 'COMPANION' + '__'           # the placeholder itself, which capture.py would otherwise replace here
MADE, BOUGHT = 'Paint - Enamel Glossy (Blue)', 'Paint - Enamel Glossy (Yellow)'
# one component of each kind, as Chapter 47's exploded view names them
SHOWN = {'Adapter plate': 'Adapter plate', 'Beam': 'Beam', 'Bracket': 'Bracket 1', 'End block': 'End block 1',
         'End plate': 'End plate 1', 'Lever': 'Lever B', 'Press plate': 'Press plate', 'Press rod': 'Press rod 1',
         'Rod end': 'Rod end A', 'Stub shaft long': 'Stub shaft long B', 'Stub shaft short': 'Stub shaft short B',
         'Tine': 'Tine 1 B', 'Tube': 'Tube B', 'Bearing 6206': 'Bearing 6206 2', 'Cap screw M10x25': 'Cap screw M10x25 2',
         'Cap screw M6x16': 'Cap screw M6x16 1', 'Cap screw M8x20': 'Cap screw M8x20 1',
         'Cap screw M8x30': 'Cap screw M8x30 1', 'Cylinder DSBC-32-50': 'Cylinder DSBC-32-50 1',
         'Cylinder DSBC-80-125-C': 'Cylinder DSBC-80-125-C A', 'Dowel pin 10x17': 'Dowel pin 10x17'}


def exploded():
    """Chapter 47's exploded capture as a namespace, building the gripper through PartNumbers instead."""
    src = open(COMP + '/projects/palletizer/captures/bag_gripper_exploded.py', encoding='utf-8').read().replace(TOKEN, COMP)
    old = ("ns = fc.load('%s/projects/palletizer/BagGripper/BagGripper.py')\n    print(ns['build']().splitlines()[0])" % COMP)
    new = ("ns = fc.load('%s/design_office/PartNumbers/PartNumbers.py')\n    print(ns['build']('bag').splitlines()[0])" % COMP)
    if src.count(old) != 1:
        raise RuntimeError('the exploded capture no longer builds the gripper as expected')
    ns = {'__name__': 'mdfc_capture', '__file__': COMP + '/projects/palletizer/captures/bag_gripper_exploded.py'}
    exec(compile(src.replace(old, new), ns['__file__'], 'exec'), ns)
    return ns


def items():
    import detail_inventory as D
    importlib.reload(D)
    nb, _ = D.numbering()
    rows = [(num, name) for num, name, q, kind in nb['bag'] if kind != 'assembly']
    missing = [name for num, name in rows if name not in SHOWN]
    if missing:
        raise RuntimeError('no component chosen to show: %s' % missing)
    return rows


def colour(des):
    import mdfc_parts as mp
    root = des.rootComponent
    for o in root.occurrences:
        o.appearance = None
    for o in root.allOccurrences:
        c = o.component
        if c.bRepBodies.count == 0 or not o.isLightBulbOn:
            continue
        if c.partNumber.startswith('S-') or c.description.startswith('with the '):
            mp.colour(des, o, BOUGHT)
        elif c.partNumber:
            mp.colour(des, o, MADE)


def run(_context):
    rows = items()
    ns = exploded()
    ns['ITEMS'] = [SHOWN[name] for num, name in rows]
    print('items: ' + json.dumps([[k, num, name] for k, (num, name) in enumerate(rows, 1)]))
    ns['run'](_context)                          # builds, numbers, takes apart, sets the camera, prints ANCHORS
    des = fc.design()
    colour(des)
    adsk.core.Application.get().activeViewport.refresh()
