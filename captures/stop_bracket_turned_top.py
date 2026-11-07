# Pictures for Chapter 62: the opening stop of jaw A with its arm turned, built by StopBracket.py: the bracket, its
# DYSC-25 and lock nut, its two M8 with their washers and nuts, the end plate see-through; everything else hidden.
# TOP = True: the bracket cut 15 mm above the foot's face, the lug and the absorber left out, seen down the arm (from
# +x), so that the arm's square shows turned on the foot's, its faces beside the M8 heads.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion
import mdfc_parts as mp
import range_sweep as rs

TOP = True
VIEW = (0.70, 0.35, 0.62)           # three-quarter view from the foot's side, above the absorber, as Chapter 63's
CUT = 452.0                         # mm: 15 above the foot's face, under the lug's lowest point (461)


def run(_context):
    ns = fc.load('__COMPANION__/scripts/StopBracket/StopBracket.py')
    print(ns['build'](2.5).splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    o_ = rs.stop_layout()['open']
    cy, cz = o_['centre']

    def centre(o):
        b = o.bRepBodies.item(0).boundingBox
        return [(b.minPoint.x + b.maxPoint.x) * 5, (b.minPoint.y + b.maxPoint.y) * 5, (b.minPoint.z + b.maxPoint.z) * 5]

    def by_bracket(o):
        c = centre(o)
        return c[0] > 380 and abs(c[1] - cy) < 35 and abs(c[2] - cz) < 35
    keep = []
    bracket = None
    for o in leaves:
        nm = o.component.name
        if nm == 'Stop bracket open 1':
            bracket = o
            keep.append(o)
        elif nm in ('Shock absorber DYSC-25-25-Y1F open 1', 'Lock nut DYSC-25-25-Y1F open 1', 'End plate 1') and not TOP:
            keep.append(o)
        elif nm.startswith(('Cap screw ISO 4762 M8x40 stop', 'Washer ISO 7089 8 stop', 'Nut ISO 4032 M8 stop')) and by_bracket(o):
            if not TOP or nm.startswith('Cap screw'):
                keep.append(o)
    if TOP:                          # the bracket below the cut, as a part of its own
        tb = mp.tbm().copy(bracket.bRepBodies.item(0))
        mp.intersect(tb, mp.box(400.0, CUT, cy - 60.0, cy + 60.0, cz - 60.0, cz + 60.0))
        cut = mp.new_occ(root, 'Stop bracket open 1, cut')
        mp.add_body(cut, tb, 'cut', None)
        keep.remove(bracket)
        keep.append(cut)
        leaves.append(cut)
    for o in leaves:
        o.isLightBulbOn = o in keep
    for top in root.occurrences:
        if any(s.fullPathName.startswith(top.fullPathName) for s in keep):
            top.isLightBulbOn = True
    for o in keep:
        if o.component.name == 'End plate 1':
            for b in o.bRepBodies:
                b.opacity = 0.25
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    print('shown: %s' % sorted(o.component.name for o in keep))
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    lo = [min(getattr(o.boundingBox.minPoint, a) for o in keep) for a in 'xyz']
    hi = [max(getattr(o.boundingBox.maxPoint, a) for o in keep) for a in 'xyz']
    c = [(lo[i] + hi[i]) / 2 for i in range(3)]
    view = (1.0, 0.0, 0.0) if TOP else VIEW
    n = math.sqrt(sum(v * v for v in view))
    cam.target = adsk.core.Point3D.create(*c)
    cam.eye = adsk.core.Point3D.create(*(c[i] + 300 * view[i] / n for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
