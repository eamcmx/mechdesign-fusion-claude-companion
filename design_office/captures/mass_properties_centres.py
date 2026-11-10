# Pictures for Chapter 57: the detailed palletizer robot at q2 = q3 = 0, built by RangeSweep.py, with a small red ball at
# the centre of mass of each of its moving groups and of the base, as Fusion weighs them; the robot's bodies made
# see-through and the balls drawn in a group of their own, for the picture only.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion
import mdfc_parts as mp

VIEW = (0.80, -1.00, 0.50)
R_BALL = 45.0                      # mm


def leaves_under(o):
    out, st = [], [o]
    while st:
        x = st.pop()
        if x.component.bRepBodies.count:
            out.append(x)
        st.extend(list(x.childOccurrences))
    return out


def run(_context):
    ns = fc.load('__COMPANION__/design_office/RangeSweep/RangeSweep.py')
    msg, setter = ns['machine']('robot')
    print(msg.splitlines()[0])
    fc.mark()
    setter(('outline', (0.0, 0.0)))
    des = fc.design()
    root = des.rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    centres = []
    for top in list(root.occurrences):
        ls = leaves_under(top)
        if not ls:
            continue
        m = sum(o.physicalProperties.mass for o in ls)
        c = [sum(o.physicalProperties.mass * getattr(o.physicalProperties.centerOfMass, a) * 10.0 for o in ls) / m for a in 'xyz']
        centres.append((top.component.name, m, c))
    for top in list(root.occurrences):                  # the robot see-through, so that the centres inside it show
        for o in leaves_under(top):
            for b in o.component.bRepBodies:
                b.opacity = 0.22
    g = mp.new_occ(root, 'Centres of mass')
    for name, m, c in centres:
        mp.add_body(g, mp.sphere(c, R_BALL), 'centre %s' % name)
    mp.colour(des, g, 'Paint - Enamel Glossy (Red)')
    print('centres: %s' % ['%s %.1f kg at (%.0f, %.0f, %.0f)' % (n, m, *c) for n, m, c in centres])
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(x * x for x in VIEW))
    bb = root.boundingBox
    ctr = [(getattr(bb.minPoint, a) + getattr(bb.maxPoint, a)) / 2.0 for a in 'xyz']
    cam.target = adsk.core.Point3D.create(*ctr)
    cam.eye = adsk.core.Point3D.create(*(ctr[i] + 900 * VIEW[i] / n for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
