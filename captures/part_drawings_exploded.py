# Pictures for Chapter 63: the opening stop of jaw A taken apart, built by RangeSweep.py: the bracket P13-018 off the
# end plate (hidden) along the bolts' line, the DYSC-25 and its lock nut out of the lug along the absorber's axis, the
# two M8 out past the bracket, their washers and nuts the other way. The joints deleted and the design turned to direct
# modelling. The camera and the anchors for the balloons come from Chapter 60's capture, loaded, not copied.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion
import range_sweep as rs

VIEW = (0.75, 0.25, 0.60)
ITEMS = ['Stop bracket open 1', 'Shock absorber DYSC-25-25-Y1F open 1', 'Lock nut DYSC-25-25-Y1F open 1',
         'Cap screw', 'Washer', 'Nut']


def run(_context):
    cap60 = fc.load('__COMPANION__/captures/fatigue_spots_exploded.py')
    cap60['ITEMS'] = ITEMS
    ns = fc.load('__COMPANION__/scripts/RangeSweep/RangeSweep.py')
    msg, _setter = ns['machine']('bag')
    print(msg.splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints, comp.joints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    o_ = rs.stop_layout()['open']
    cy, cz = o_['centre']
    n = (0.0, o_['axis'][0], o_['axis'][1])

    def centre(o):
        b = o.boundingBox
        return [(b.minPoint.x + b.maxPoint.x) * 5, (b.minPoint.y + b.maxPoint.y) * 5, (b.minPoint.z + b.maxPoint.z) * 5]

    def by_bracket(o):                          # a fastener of the opening bracket of jaw A: inside the foot's square
        c = centre(o)
        return c[0] > 380 and abs(c[1] - cy) < 35 and abs(c[2] - cz) < 35
    pick = {}
    for o in leaves:
        nm = o.component.name
        if nm in ITEMS[:3]:
            pick[nm] = o
        for pre in ('Cap screw ISO 4762 M8x40 stop', 'Washer ISO 7089 8 stop', 'Nut ISO 4032 M8 stop'):
            if nm.startswith(pre) and by_bracket(o):
                pick.setdefault(pre, []).append(o)
    moves = {'Stop bracket open 1': (90.0, 0, 0), 'Shock absorber DYSC-25-25-Y1F open 1': (90.0, 150.0 * n[1], 150.0 * n[2]),
             'Lock nut DYSC-25-25-Y1F open 1': (90.0, 60.0 * n[1], 60.0 * n[2]),
             'Cap screw ISO 4762 M8x40 stop': (170.0, 0, 0), 'Washer ISO 7089 8 stop': (-40.0, 0, 0),
             'Nut ISO 4032 M8 stop': (-75.0, 0, 0)}
    shown = []
    for key, val in pick.items():
        for o in (val if isinstance(val, list) else [val]):
            shown.append(o)
            v = moves[key]
            m = o.transform2
            t = m.translation
            m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
            o.transform2 = m
    for o in leaves:
        o.isLightBulbOn = o in shown
    for top in root.occurrences:
        if any(s.fullPathName.startswith(top.fullPathName) for s in shown):
            top.isLightBulbOn = True
    print('parts shown: %s' % sorted('%s x%d' % (k, len(v) if isinstance(v, list) else 1) for k, v in pick.items()))
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    nodes = {}
    names = {'Cap screw ISO 4762 M8x40 stop': 'Cap screw', 'Washer ISO 7089 8 stop': 'Washer', 'Nut ISO 4032 M8 stop': 'Nut'}
    for key, val in pick.items():
        first = val[0] if isinstance(val, list) else val
        nodes[names.get(key, key)] = cap60['world_nodes'](first)
    every = {k: v for k, v in nodes.items()}
    for key, val in pick.items():
        if isinstance(val, list):
            for o in val[1:]:
                every[names[key] + ' ' + o.name] = cap60['world_nodes'](o)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    nv = math.sqrt(sum(c * c for c in VIEW))
    allp = [p for pts in every.values() for p in pts]
    c = [sum(p[i] for p in allp) / len(allp) * 0.1 for i in range(3)]
    cam.target = adsk.core.Point3D.create(*c)
    cam.eye = adsk.core.Point3D.create(*(c[i] + 300 * VIEW[i] / nv for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
    cam = vp.camera
    cam.viewExtents = cam.viewExtents * 1.10
    vp.camera = cam
    vp.refresh()
    a = cap60['anchors'](vp.camera, nodes)
    a['extent'] = cap60['anchors'](vp.camera, every)['extent']
    import json
    print('ANCHORS ' + json.dumps(a))
