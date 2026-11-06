# Pictures for Chapter 65: the bag gripper as Parts 9 and 10 left it, built by RangeSweep.py through
# DesignForAssembly.py's own steps (the census, the bill's lines, the answers as attributes and colours) and stopped
# before the fixings: blue the pieces the test keeps, red the candidates, yellow the three kinds that nothing holds,
# the tines, the end blocks and the cylinder brackets. The bag and the station hidden; seen from the front, a little
# from +x and above.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.55, -1.00, 0.55)       # towards the camera


def run(_context):
    ns = fc.load('__COMPANION__/scripts/DesignForAssembly/DesignForAssembly.py')
    rs = ns['_load']('RangeSweep')
    msg, _ = rs.machine('bag')
    print(msg.splitlines()[0])
    fc.mark()
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    rows, occ = ns['census'](des)
    _, lines = ns['lines_of'](rows)
    tagged = ns['tag'](des, occ, lines)
    back = ns['read_back'](des)
    print('tagged %d, read back %d pieces, %d kept %s' % (len(tagged), back['N'], back['Nmin'], sorted(back['by_q'].items())))
    for o in root.occurrences:
        if o.component.name.startswith(('Bag', 'Station')):
            o.isLightBulbOn = False
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(c * c for c in VIEW))
    b = None
    for o in root.allOccurrences:
        if o.bRepBodies.count and o.isLightBulbOn and o.isVisible:
            bb = o.boundingBox
            if b is None:
                b = [bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z]
            else:
                b = [min(b[0], bb.minPoint.x), min(b[1], bb.minPoint.y), min(b[2], bb.minPoint.z),
                     max(b[3], bb.maxPoint.x), max(b[4], bb.maxPoint.y), max(b[5], bb.maxPoint.z)]
    c = [(b[i] + b[i + 3]) / 2 for i in range(3)]
    cam.target = adsk.core.Point3D.create(*c)
    cam.eye = adsk.core.Point3D.create(*(c[i] + 900 * VIEW[i] / n for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
    cam = vp.camera
    cam.viewExtents = cam.viewExtents * 1.05
    vp.camera = cam
    vp.refresh()
