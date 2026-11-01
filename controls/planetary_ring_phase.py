# PlanetaryPrompt against the acceptance criteria of its prompt (Topic 5), measured on what Fusion built:
#   1. the student path: Run, the dialog, OK with the defaults (Command.doExecute; message boxes printed)
#   2. Ring, Sun, Planet 1..N, Carrier, Sun bearing: one body each; the ring grounded; As-built Joints only
#      (N + 2 revolute, 1 rigid); N + 1 Motion Links; no interference at rest
#   3. the sun driven through one turn in 45 deg steps: at every step the carrier at sun / (1 + Zr/Zs) (joint value
#      AND the body's real turn), each planet at -Zr/Zp of the carrier relative to it, and no interference
#   4. Zs 24 / Zp 30 / N 4 builds and passes 2 and 3; Zs 20 / Zp 25 / N 4 and touching planets are refused
# Every design it opens is marked 'mdfc', so tools/run_check.py --cleanup closes only those.
import math, time
import adsk.core, adsk.fusion

PATH = "__COMPANION__/scripts/PlanetaryPrompt/PlanetaryPrompt.py"
fails = []
CONTROL = ('phi_r = 0.0 if zp % 2 else math.pi / zr', 'phi_r = math.pi / zr if zp % 2 else 0.0')   # WRONG ring phase on purpose


def ok(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        fails.append(msg)


def load():
    src = open(PATH, encoding='utf-8').read().replace('ui.messageBox(', 'print(')
    src = src.replace(CONTROL[0], CONTROL[1]); assert CONTROL[1] in src
    ns = {'__name__': 'planetary_prompt_check', '__file__': PATH}
    exec(compile(src, PATH, 'exec'), ns)
    return ns


def turn_about_z(occ):
    t = occ.transform2
    return math.degrees(math.atan2(t.getCell(1, 0), t.getCell(0, 0)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def interference(des, occs):
    col = adsk.core.ObjectCollection.create()
    for o in occs:
        col.add(o)
    inp = des.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = False
    res = des.analyzeInterference(inp)
    return [(r.entityOne.name, r.entityTwo.name, round(r.interferenceBody.volume * 1000, 4)) for r in res] if res else []


def check_design(zs, zp, n):
    app = adsk.core.Application.get()
    app.activeDocument.attributes.add('mdfc', 'run', 'planetary_prompt_check')
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    zr = zs + 2 * zp
    occ = {o.component.name: o for o in root.occurrences}
    want = ['Ring', 'Sun'] + ['Planet %d' % (i + 1) for i in range(n)] + ['Carrier', 'Sun bearing']
    ok(sorted(occ) == sorted(want), 'parts: %s' % sorted(occ))
    ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    ok(occ['Ring'].isGrounded, 'the ring is grounded')
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    ok(root.joints.count == 0 and kinds.count('RevoluteJointMotion') == n + 2 and kinds.count('RigidJointMotion') == 1,
       'As-built Joints only: %d revolute, %d rigid, plain Joints %d' % (kinds.count('RevoluteJointMotion'),
                                                                        kinds.count('RigidJointMotion'), root.joints.count))
    ok(root.motionLinks.count == n + 1, 'Motion Links: %s' % [l.name for l in root.motionLinks])
    c = interference(des, list(root.occurrences))
    ok(not c, 'no interference at rest %s' % (c or ''))
    J = {j.name: j for j in root.asBuiltJoints}
    js, jc = J['Sun (input)'].jointMotion, J['Carrier (output)'].jointMotion
    jp = [J['Planet %d on carrier' % (i + 1)].jointMotion for i in range(n)]
    c0 = turn_about_z(occ['Carrier'])
    worst_c, worst_p, clashes = 0.0, 0.0, []
    for k in range(1, 9):
        s = 45.0 * k
        js.rotationValue = math.radians(s)
        adsk.doEvents()
        car_j = math.degrees(jc.rotationValue)
        car_real = turn_about_z(occ['Carrier']) - c0
        want_c = s / (1 + zr / zs)
        worst_c = max(worst_c, abs(car_j - want_c), abs(wrap(car_real - want_c)))
        for m_ in jp:
            worst_p = max(worst_p, abs(math.degrees(m_.rotationValue) + zr / zp * want_c))
        c = interference(des, list(root.occurrences))
        if c:
            clashes.append((s, c[:3]))
    print('  one sun turn: carrier joint %.3f deg, planet 1 on the carrier %.3f deg'
          % (math.degrees(jc.rotationValue), math.degrees(jp[0].rotationValue)))
    ok(worst_c < 0.01, 'carrier = sun / %.3f at every 45 deg step (worst error %.4f deg)' % (1 + zr / zs, worst_c))
    ok(worst_p < 0.01, 'each planet = -%.3f x carrier, relative to the carrier (worst error %.4f deg)' % (zr / zp, worst_p))
    ok(not clashes, 'no interference at any of the 8 steps %s' % (clashes or ''))
    js.rotationValue = 0
    adsk.doEvents()


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
    check_design(16, 20, 3)
    print('CONTROL: the ring phase is deliberately wrong; the clash checks above MUST fail')
    return
    msg = ns['build'](zs=24, zp=30, n=4)
    print('  Zs 24, Zp 30, N 4: ' + msg.splitlines()[2])
    check_design(24, 30, 4)
    for args, word in (({'zs': 20, 'zp': 25, 'n': 4}, 'not whole'), ({'zs': 18, 'zp': 18, 'n': 6}, 'touch')):
        try:
            ns['build'](**args)
            ok(False, '%s refused' % args)
        except ValueError as e:
            ok(word in str(e), '%s refused: %s' % (args, e))
    print('ALL PASS' if not fails else '%d FAILED' % len(fails))
