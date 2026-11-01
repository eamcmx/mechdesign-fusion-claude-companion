# Pictures for Chapter 36: Palletizer with its defaults beside its pallet, posed by its inverse at BOX (1 to 24), the
# boxes before it stacked; with BOX = 0 the arm is posed at (t2, t3) = SIDE and no pallet is drawn.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

BOX, SIDE = 24, (60.0, -30.0)


def run(_context):
    ns = fc.load('__COMPANION__/scripts/Palletizer/Palletizer.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    pat = ns['pattern'](1200.0, 3)
    if BOX:
        tb = ns['box'](800.0, 1600.0, -600.0, 600.0, 0.0, 150.0)
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = 'Pallet'
        ns['add_body'](occ.component, tb, 'Pallet')
        ns['colour'](des, occ, 'Paint - Enamel Glossy (Grey)')
        for k, (x, y, z) in enumerate(pat[:BOX - 1]):
            b = ns['box'](x - 199.0, x + 199.0, y - 149.0, y + 149.0, z - 200.0, z - 1.0)
            o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            o.component.name = 'Box %d' % (k + 1)
            ns['add_body'](o.component, b, 'Box %d' % (k + 1))
            ns['colour'](des, o, 'Paint - Enamel Glossy (Yellow)')
        q1, t2, t3 = ns['ik'](*pat[BOX - 1])[0]
    else:
        q1, (t2, t3) = 0.0, SIDE
    occ = {n: fc.occ(n) for n in ('Turntable', 'Upper arm', 'Forearm')}
    R0 = {n: fc.rotation(o) for n, o in occ.items()}
    sign = {}
    for n, part, ax in (('J1 turntable', 'Turntable', (0, 0, 1)), ('J2 upper arm', 'Upper arm', (0, -1, 0)),
                        ('Elbow', 'Forearm', (0, -1, 0))):
        j = fc.joint(n)
        j.jointMotion.rotationValue = 0.1
        adsk.doEvents()
        t = fc.turn_deg(fc.rotation(occ[part]), R0[part], ax)
        if n == 'Elbow':
            t -= fc.turn_deg(fc.rotation(occ['Upper arm']), R0['Upper arm'], ax)
        sign[n] = 1 if t > 0 else -1
        j.jointMotion.rotationValue = 0.0
        adsk.doEvents()
    fc.joint('J1 turntable').jointMotion.rotationValue = math.radians(sign['J1 turntable'] * q1)
    fc.joint('Elbow').jointMotion.rotationValue = math.radians(sign['Elbow'] * (t3 - t2 + 90.0))   # the elbow first,
    adsk.doEvents()
    fc.joint('J2 upper arm').jointMotion.rotationValue = math.radians(sign['J2 upper arm'] * (t2 - 90.0))   # then J2
    adsk.doEvents()
    print('posed at J1 %.2f, t2 %.2f, t3 %.2f' % (q1, t2, t3))
    adsk.core.Application.get().activeViewport.fit()
