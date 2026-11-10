# CONTROL for the prompt line "Create each component WHERE IT BELONGS and never move it afterwards": PlanetaryPrompt
# with the planets created at the origin and moved to their places right after (occurrence.transform), the way a
# script without that line does it. Then the carrier and the joints are built, as in the real script. Expected: the
# later features undo the moves, so the planets end up back on the sun's axis. Marked 'mdfc' for cleanup.
import adsk.core, adsk.fusion

PATH = "__COMPANION__/projects/planetary_gearbox/PlanetaryPrompt/PlanetaryPrompt.py"
OLD = "        p = place(root, 'Planet %d' % (i + 1), x, y)\n"
NEW = ("        p = place(root, 'Planet %d' % (i + 1))\n"
       "        mv = adsk.core.Matrix3D.create()\n"
       "        mv.translation = V3(x * CM, y * CM, 0)\n"
       "        p.transform = mv\n"
       "        print('  moved %s to (%.2f, %.2f) mm' % (p.component.name, p.transform2.translation.x * 10, p.transform2.translation.y * 10))\n")


def run(_context):
    src = open(PATH, encoding='utf-8').read().replace('ui.messageBox(', 'print(')
    assert OLD in src
    ns = {'__name__': 'control'}
    exec(compile(src.replace(OLD, NEW), PATH, 'exec'), ns)
    ns['build']()
    app = adsk.core.Application.get()
    app.activeDocument.attributes.add('mdfc', 'run', 'control')
    root = adsk.fusion.Design.cast(app.activeProduct).rootComponent
    for o in root.occurrences:
        if o.component.name.startswith('Planet'):
            t = o.transform2.translation
            print('  after the build: %s at (%.2f, %.2f) mm' % (o.component.name, t.x * 10, t.y * 10))
    print('CONTROL: planets moved after creation')
