# Pictures for Chapter 64: the tool changer taken apart along its axis, built by RangeSweep.py, its balloons the items
# of its bill (BillOfMaterials.py's order: the made kinds by number, then the bought). The robot plate stays at the top;
# its dowel and the six flange screws come down out of it, the master's cover lifts off the body, the piston rises out
# of the body, the cam drops below it with the six segments moved out of their slots, and the tool plate leaves at the
# bottom with its two dowels. The joints deleted and the design turned to direct modelling. The anchors for the
# balloons come from Chapter 60's capture, loaded, not copied.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (1.0, -1.25, 0.55)
ITEMS = ['Cam', 'Master body', 'Master cover', 'Piston with rod', 'Robot plate ISO 9409-1-160-6-M10', 'Segment',
         'Tool plate ISO 9409-1-160-6-M10', 'Cap screw M10x35', 'Pin 10 robot', 'Pin 12 tool']
# mm along z, and out from the axis for the segments
MOVES = {'Pin 10 robot': -30.0, 'Cap screw M10x35': -70.0, 'Master cover': -110.0, 'Piston with rod': -160.0,
         'Master body': -260.0, 'Cam': -320.0, 'Segment': -320.0, 'Tool plate ISO 9409-1-160-6-M10': -400.0,
         'Pin 12 tool': -375.0, 'Robot plate ISO 9409-1-160-6-M10': 0.0}
OUT = 60.0


def run(_context):
    cap60 = fc.load('__COMPANION__/captures/fatigue_spots_exploded.py')
    cap60['ITEMS'] = ITEMS
    ns = fc.load('__COMPANION__/scripts/RangeSweep/RangeSweep.py')
    msg, _setter = ns['machine']('changer')
    print(msg.splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints, comp.joints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    leaves = [o for o in root.allOccurrences if o.bRepBodies.count]
    pick = {}
    for o in leaves:
        nm = o.component.name
        key = next(k for k in sorted(MOVES, key=len, reverse=True) if nm.startswith(k))
        pick.setdefault(key, []).append(o)
    for key, occs in pick.items():
        for o in occs:
            dz = MOVES[key]
            dx = dy = 0.0
            if key == 'Segment':                         # out of its slot, along its own radius
                b = o.bRepBodies.item(0).boundingBox
                cx, cy = (b.minPoint.x + b.maxPoint.x) * 5, (b.minPoint.y + b.maxPoint.y) * 5
                r = math.hypot(cx, cy)
                dx, dy = OUT * cx / r, OUT * cy / r
            m = o.transform2
            t = m.translation
            m.translation = adsk.core.Vector3D.create(t.x + dx * 0.1, t.y + dy * 0.1, t.z + dz * 0.1)
            o.transform2 = m
    print('parts moved: %s' % sorted('%s x%d' % (k, len(v)) for k, v in pick.items()))
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    nodes = {k: cap60['world_nodes'](v[0]) for k, v in pick.items()}
    every = dict(nodes)
    for k, v in pick.items():
        for o in v[1:]:
            every[k + ' ' + o.name] = cap60['world_nodes'](o)
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
