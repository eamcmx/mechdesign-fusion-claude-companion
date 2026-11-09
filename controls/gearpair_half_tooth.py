# GearPairOnPlate against the acceptance criteria of its prompt (Chapter 12), measured on what Fusion built:
#   1. the student path: Run, the dialog, OK with the defaults (Command.doExecute; message boxes printed)
#   2. five components, one body each, gear 2 and shaft 2 at x = a; two rigid and two revolute As-built Joints,
#      no plain Joints, one Motion Link, no interference at rest
#   3. Drive gear 1 by 90 deg: gear 2 turns 45 deg the other way (its joint value AND its body's real turn)
#   4. no two teeth collide while gear 1 turns through one tooth pitch, in 12 steps
#   5. z2 = 50 gives a = 70 mm; z1 = 12 is refused with a message
# Every design it opens is marked 'mdfc', so tools/run_check.py --cleanup closes only those.
import math, time, traceback
import adsk.core, adsk.fusion

PATH = "__COMPANION__/scripts/GearPairOnPlate/GearPairOnPlate.py"
fails = []


def ok(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        fails.append(msg)


def mark(doc):
    doc.attributes.add('mdfc', 'run', 'control')


def load():
    src = open(PATH, encoding='utf-8').read().replace('ui.messageBox(', 'print(')
    a = 'return math.pi - math.pi / z2'
    assert a in src
    src = src.replace(a, 'return 0.0')   # CONTROL: no mesh phase
    ns = {'__name__': 'gearpair_check', '__file__': PATH}
    exec(compile(src, PATH, 'exec'), ns)
    return ns


def occ_named(root, start):
    return [o for o in root.occurrences if o.component.name.startswith(start)][0]


def turn_about_z(occ):
    t = occ.transform2
    return math.degrees(math.atan2(t.getCell(1, 0), t.getCell(0, 0)))


def interference(root, occs):
    col = adsk.core.ObjectCollection.create()
    for o in occs:
        col.add(o)
    inp = root.parentDesign.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = False
    res = root.parentDesign.analyzeInterference(inp)
    return [(r.entityOne.name, r.entityTwo.name, r.interferenceBody.volume * 1000) for r in res] if res else []


def check_design(m, z1, z2):
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    a = m * (z1 + z2) / 2
    names = sorted(o.component.name for o in root.occurrences)
    ok(len(names) == 5, 'five components: %s' % names)
    ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each: %s'
       % [o.component.bRepBodies.count for o in root.occurrences])
    g1, g2 = occ_named(root, 'Gear 1'), occ_named(root, 'Gear 2')
    s2, plate = occ_named(root, 'Shaft 2'), occ_named(root, 'Base plate')
    x2, xs2 = g2.transform2.translation.x * 10, s2.transform2.translation.x * 10
    ok(abs(x2 - a) < 1e-6 and abs(xs2 - a) < 1e-6, 'gear 2 and shaft 2 at x = a = %.2f mm (measured %.3f, %.3f)' % (a, x2, xs2))
    ok(plate.isGrounded, 'the base plate is grounded')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion', 'RevoluteJointMotion', 'RigidJointMotion', 'RigidJointMotion'],
       'As-built Joints only: %s, plain Joints %d' % (kinds, root.joints.count))
    ok(root.motionLinks.count == 1, 'one Motion Link: %s' % [l.name for l in root.motionLinks])
    clash = interference(root, list(root.occurrences))
    ok(not clash, 'no interference at rest %s' % (clash or ''))
    j1 = [j for j in root.asBuiltJoints if j.name == 'Gear 1 turns'][0]
    j2 = [j for j in root.asBuiltJoints if j.name == 'Gear 2 turns'][0]
    r0 = turn_about_z(g2)
    j1.jointMotion.rotationValue = math.radians(90)
    adsk.doEvents()
    v2 = math.degrees(j2.jointMotion.rotationValue)
    real = (turn_about_z(g2) - r0 + 540) % 360 - 180
    want = -90.0 * z1 / z2
    ok(abs(v2 - want) < 0.01 and abs(real - want) < 0.01,
       'gear 1 driven 90 deg: gear 2 joint reads %.3f, its body turned %.3f deg about z (want %.3f)' % (v2, real, want))
    worst = []
    pitch = 360.0 / z1
    for k in range(13):
        j1.jointMotion.rotationValue = math.radians(k * pitch / 12)
        adsk.doEvents()
        c = interference(root, [g1, g2])
        if c:
            worst.append((round(k * pitch / 12, 2), c))
    ok(not worst, 'no tooth collision while gear 1 turns 0 to %.1f deg in 13 positions %s' % (pitch, worst or ''))
    j1.jointMotion.rotationValue = 0
    adsk.doEvents()
    return app.activeDocument


def run(_context):
    app = adsk.core.Application.get()
    ns = load()
    print(ns['VERSION'])
    # 1. the student path: the dialog, OK with the defaults
    seen = []
    base = ns['_Created']

    class Seen(base):
        def notify(self, args):
            base.notify(self, args)
            seen.append(args.command)
    ns['_Created'] = Seen
    n0 = app.documents.count
    ns['run'](None)
    for _ in range(200):
        if seen:
            break
        adsk.doEvents()
    ok(bool(seen), 'Run opens the dialog')
    defaults = {}
    if seen:
        ins = seen[0].commandInputs
        defaults = {ins.item(i).id: ins.item(i).value for i in range(ins.count)}
        print('  dialog defaults (API units):', defaults)
        t0 = time.time()
        seen[0].doExecute(True)
        for _ in range(30):
            adsk.doEvents()
        print('  OK pressed, built in %.0f s' % (time.time() - t0))
    ok(app.documents.count == n0 + 1, 'a new design opened (documents %d -> %d)' % (n0, app.documents.count))
    mark(app.activeDocument)
    check_design(2.0, 20, 40)
    print('CONTROL: gear 2 not turned half a tooth; the clash checks MUST fail')
    return
    # 5. a changed input, through build()
    msg = ns['build'](z2=50)
    mark(app.activeDocument)
    print('  z2 = 50: ' + msg.splitlines()[2])
    ok('= 70.00 mm' in msg, 'z2 = 50 gives a = 70 mm in the message')
    check_design(2.0, 20, 50)
    try:
        ns['build'](z1=12)
        ok(False, 'z1 = 12 refused')
    except ValueError as e:
        ok('undercut' in str(e), 'z1 = 12 refused: %s' % e)
    print('ALL PASS' if not fails else '%d FAILED' % len(fails))
