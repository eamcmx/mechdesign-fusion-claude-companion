# BevelWristPrompt against the acceptance criteria of its prompt (Topic 5), measured on what Fusion built:
#   1. the student path: Run, the dialog, OK with the defaults (Command.doExecute; message boxes printed)
#   2. Frame, Yoke, Output, Idler, Side L, Side R: one body each; frame grounded; 5 revolute As-built Joints, no plain
#      Joints; 3 Motion Links; no interference at rest
#   3. measured as the bodies' REAL turns about the world axes (not joint values):
#      pitch 30: yoke and all gears 30 about Y, the output still on the yoke;
#      roll 40: output 40 about Z, side L +40, side R -40 about Y, idler -40 about Z;
#      pitch 30 + roll 40: side gears 70 and -10 about Y, so (70 - 10)/2 = 30 and (70 + 10)/2 = 40
#   4. no interference at (30, 40), (-45, 120) and through one tooth pitch of roll in 6 steps
#   5. z = 30 and z = 41 refused
# Every design it opens is marked 'mdfc', so tools/run_check.py --cleanup closes only those.
import math, time
import adsk.core, adsk.fusion

PATH = "__COMPANION__/scripts/BevelWristPrompt/BevelWristPrompt.py"
fails = []


def ok(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        fails.append(msg)


def load():
    src = open(PATH, encoding='utf-8').read().replace('ui.messageBox(', 'print(')
    a = '    bevel_gear(out, g, math.pi / z, True)'
    assert a in src
    src = src.replace(a, '    bevel_gear(out, g, 0.0, True)')   # CONTROL: wrong phase on purpose
    ns = {'__name__': 'bevelwrist_prompt_check', '__file__': PATH}
    exec(compile(src, PATH, 'exec'), ns)
    return ns


def rot(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def turn(R1, R0, axis):
    """Angle (deg) of the rotation R1 R0^T about the world axis (right-handed)."""
    D = [[sum(R1[a][k] * R0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
    w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
    s = sum(w[i] * axis[i] for i in range(3)) / 2.0
    c = (D[0][0] + D[1][1] + D[2][2] - 1.0) / 2.0
    return math.degrees(math.atan2(s, c))


def interference(des, occs):
    col = adsk.core.ObjectCollection.create()
    for o in occs:
        col.add(o)
    inp = des.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = False
    res = des.analyzeInterference(inp)
    return [(r.entityOne.name, r.entityTwo.name, round(r.interferenceBody.volume * 1000, 3)) for r in res] if res else []


def run(_context):
    app = adsk.core.Application.get()
    ns = load()
    print(ns['VERSION'])
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
    if seen:
        t0 = time.time()
        seen[0].doExecute(True)
        for _ in range(30):
            adsk.doEvents()
        print('  OK pressed with the defaults, built in %.0f s' % (time.time() - t0))
    ok(app.documents.count == n0 + 1, 'a new design opened')
    app.activeDocument.attributes.add('mdfc', 'run', 'bevelwrist_prompt_check')
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    occ = {o.component.name: o for o in root.occurrences}
    ok(sorted(occ) == ['Frame', 'Idler gear', 'Output gear', 'Side gear L', 'Side gear R', 'Yoke'], 'parts: %s' % sorted(occ))
    ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each: %s'
       % {k: o.component.bRepBodies.count for k, o in occ.items()})
    ok(occ['Frame'].isGrounded, 'the frame is grounded')
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    ok(root.joints.count == 0 and kinds.count('RevoluteJointMotion') == 5, 'As-built Joints: %s' % [j.name for j in root.asBuiltJoints])
    ok(root.motionLinks.count == 3, 'Motion Links: %s' % [l.name for l in root.motionLinks])
    allo = list(root.occurrences)
    c = interference(des, allo)
    ok(not c, 'no interference at rest %s' % (c or ''))

    J = {j.name: j.jointMotion for j in root.asBuiltJoints}
    Y, Z = (0, 1, 0), (0, 0, 1)
    R0 = {k: rot(o) for k, o in occ.items()}

    def sign(jname, part, axis):
        J[jname].rotationValue = 0.2
        s = 1 if turn(rot(occ[part]), R0[part], axis) > 0 else -1
        J[jname].rotationValue = 0.0
        return s
    sp, sr = sign('Pitch', 'Yoke', Y), sign('Roll', 'Output gear', Z)

    def pose(p, r):
        J['Pitch'].rotationValue = sp * math.radians(p)
        J['Roll'].rotationValue = sr * math.radians(r)
        adsk.doEvents()

    def ang(part, axis):
        return turn(rot(occ[part]), R0[part], axis)

    pose(30, 0)
    a = {k: ang(k, Y) for k in ('Yoke', 'Side gear L', 'Side gear R', 'Output gear', 'Idler gear')}
    on_yoke = turn(rot(occ['Output gear']), R0['Output gear'], Y) - a['Yoke']
    ok(all(abs(v - 30) < 0.01 for v in a.values()), 'pitch 30: about Y %s' % {k: round(v, 3) for k, v in a.items()})
    ok(abs(on_yoke) < 0.01, 'pitch 30: the output did not turn on the yoke')
    pose(0, 40)
    got = (ang('Output gear', Z), ang('Side gear L', Y), ang('Side gear R', Y), ang('Idler gear', Z))
    ok(all(abs(g - w) < 0.01 for g, w in zip(got, (40, 40, -40, -40))),
       'roll 40: output %.3f about Z, side L %.3f and side R %.3f about Y, idler %.3f about Z' % got)
    pose(30, 40)
    tL, tR = ang('Side gear L', Y), ang('Side gear R', Y)
    ok(abs(tL - 70) < 0.01 and abs(tR + 10) < 0.01, 'pitch 30 + roll 40: thL = %.3f, thR = %.3f' % (tL, tR))
    ok(abs((tL + tR) / 2 - 30) < 0.01 and abs((tL - tR) / 2 - 40) < 0.01,
       'pitch = (thL + thR)/2 = %.3f, roll = (thL - thR)/2 = %.3f' % ((tL + tR) / 2, (tL - tR) / 2))
    clashes = []
    for p, r in [(30, 40), (-45, 120)] + [(0, 9.0 * i / 6) for i in range(1, 7)]:
        pose(p, r)
        c = interference(des, allo)
        if c:
            clashes.append(((p, round(r, 2)), c[:3]))
    ok(not clashes, 'no interference at (30, 40), (-45, 120) and roll 1.5 to 9 deg %s' % (clashes or ''))
    pose(0, 0)
    print('CONTROL: the output gear has the wrong mesh phase; the clash checks MUST fail')
    return
    for z, word in ((30, 'R / 3'), (41, 'odd')):
        try:
            ns['build'](z=z)
            ok(False, 'z = %d refused' % z)
        except ValueError as e:
            ok(word in str(e), 'z = %d refused: %s' % (z, e))
    print('ALL PASS' if not fails else '%d FAILED' % len(fails))
