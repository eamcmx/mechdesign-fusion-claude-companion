# CONTROL for the prompt line "a spacer each side of the rod end, so that the horn's cheeks stand clear of its
# shank": the cheeks are brought in against the rod end's inner ring, 8.2 mm from the middle, as a fork is first
# drawn, and the spacers left out. With the flap up everything fits; as the flap goes down, the horn turns towards the
# actuator and its cheeks run into the rod end's shank, which only a check made with the flap moved can see.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/FlapSystem/FlapSystem.py'
EDITS = [("    c0, c1 = S['horn_cheek']\n", "    c0, c1 = S['cheek']\n"),
         ("        ACT.part(asm, 'Spacer %s' % tag, along_x(tip[0], tip[1], a, b, 14.0, 10.0))\n", "        pass\n")]


def run(_context):
    ns = fc.load(PATH, EDITS)
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    near = [o for o in root.allOccurrences if o.component.bRepBodies.count and o.component.name in
            ('Horn', 'Rod end SI 20 ES', 'Horn pin', 'Horn pin nut', 'Piston rod', 'Jam nut M20x1.5')]
    for d in (0.0, 40.0):
        ns['pose'](d, ns['_last_joints'])
        adsk.doEvents()
        c = fc.interference(near)
        fc.ok(not c, 'flap at %.0f deg: no interference between the horn and the rod end %s' % (d, c or ''))
    print('CONTROL: the horn\'s cheeks are brought in and the spacers left out; the check at 40 deg MUST fail')
